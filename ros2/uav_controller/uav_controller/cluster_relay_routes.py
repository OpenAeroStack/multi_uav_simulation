"""
cluster_relay_routes.py
-----------------------
Two-hop relay routing through the elected cluster head.

Why this lives in Linux and not in ns-3
=======================================
`three_uav_tapbridge_integrated.cc` bridges each network namespace to its ns-3
WifiNetDevice with TapBridge in **UseLocal** mode. That is a layer-2 bridge:
traffic never reaches ns-3's `Ipv4L3Protocol`, so installing OLSR/AODV/DSDV in
the simulation would forward nothing. Every ns-3 node is a dumb radio, and all
forwarding decisions belong to the Linux namespaces.

Every namespace also sits on one flat `10.42.0.0/24`, so the kernel believes
each peer is directly connected. Making a packet take two hops therefore means
installing a **more specific /32 route** with an explicit next hop:

    gcsns    : ip route replace <member>/32 via <ch> dev wifi0 onlink
    uav<m>   : ip route replace <gcs>/32    via <ch> dev wifi0 onlink
    uav<ch>  : net.ipv4.ip_forward = 1

The /32 beats the connected /24, and `onlink` tells the kernel to ARP for the
cluster head even though the route looks like it points inside a subnet it is
already part of.

The redirect trap
=================
The cluster head forwards a packet back out the same interface it arrived on,
between two hosts on one subnet. Linux's default reaction is to send an ICMP
redirect saying "talk to the member directly" — which is precisely the path
that is broken, and which would silently dismantle the relay seconds after it
is installed. `send_redirects` is disabled on the head and `accept_redirects`
on both endpoints for exactly this reason.

Policy
======
A member relays only while its **direct** GCS link is bad AND its hop to the
head is good, both held for `consecutive` election ticks. Enter and exit use
different SNR thresholds so a link hovering at the boundary does not flap the
routing table every 2 s.
"""

import subprocess
import threading
from typing import Dict, List, Optional, Set, Tuple


class RelayRouteManager:
    """Owns the /32 relay routes for one fleet. Not thread-safe by itself;
    the owning node calls update() from its single election timer."""

    def __init__(
        self,
        logger,
        num_uavs: int,
        *,
        enabled: bool = True,
        gcs_netns: str = 'gcsns',
        uav_netns_prefix: str = 'uav',
        subnet_prefix: str = '10.42.0.',
        gcs_host: int = 10,
        iface: str = 'wifi0',
        enter_snr_db: float = 5.0,
        exit_snr_db: float = 10.0,
        min_hop_snr_db: float = 5.0,
        consecutive: int = 3,
        use_sudo: bool = True,
    ) -> None:
        self.log = logger
        self.num_uavs = num_uavs
        self.enabled = enabled

        self.gcs_netns = gcs_netns
        self.uav_netns_prefix = uav_netns_prefix
        self.subnet_prefix = subnet_prefix
        self.gcs_host = gcs_host
        self.iface = iface

        self.enter_snr_db = enter_snr_db
        self.exit_snr_db = exit_snr_db
        self.min_hop_snr_db = min_hop_snr_db
        self.consecutive = max(1, int(consecutive))
        self.use_sudo = use_sudo

        # member id -> cluster head id currently relaying it
        self.relayed: Dict[int, int] = {}
        # member id -> (desired_state, streak)
        self._pending: Dict[int, Tuple[bool, int]] = {}
        self._forwarding_ch: Optional[int] = None
        self._hardened: Set[str] = set()

        # ip/sysctl calls are slow enough to matter inside a 2 s election
        # timer, so they run off the timer thread. One worker keeps them
        # serialized, so routes are applied in the order they were decided.
        self._queue: List[List[str]] = []
        self._queue_cv = threading.Condition()
        self._stop = False
        self._worker = threading.Thread(
            target=self._drain, name='relay-routes', daemon=True)
        self._worker.start()

    # ── addressing ────────────────────────────────────────────────────────────

    def gcs_ip(self) -> str:
        return f'{self.subnet_prefix}{self.gcs_host}'

    def uav_ip(self, uav_id: int) -> str:
        return f'{self.subnet_prefix}{self.gcs_host + uav_id}'

    def uav_netns(self, uav_id: int) -> str:
        return f'{self.uav_netns_prefix}{uav_id}'

    # ── command plumbing ──────────────────────────────────────────────────────

    def _netns(self, namespace: str, *args: str) -> List[str]:
        # -n: never prompt. This runs from a ROS timer with no tty, so a stale
        # sudo timestamp must fail loudly and immediately rather than hang the
        # worker for the rest of the mission. The launcher holds the timestamp
        # fresh with a keep-alive for exactly this reason.
        prefix = ['sudo', '-n'] if self.use_sudo else []
        return prefix + ['ip', 'netns', 'exec', namespace] + list(args)

    def _enqueue(self, *commands: List[str]) -> None:
        with self._queue_cv:
            self._queue.extend(commands)
            self._queue_cv.notify()

    def _drain(self) -> None:
        while True:
            with self._queue_cv:
                while not self._queue and not self._stop:
                    self._queue_cv.wait()
                if self._stop and not self._queue:
                    return
                command = self._queue.pop(0)

            try:
                result = subprocess.run(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=10.0,
                )
                if result.returncode != 0:
                    detail = result.stderr.decode(errors='replace').strip()
                    # Deleting an absent route is normal during teardown.
                    if 'No such process' in detail:
                        pass
                    elif 'password is required' in detail:
                        self.log.error(
                            'relay routes need sudo but the sudo timestamp has '
                            'expired — the launcher keep-alive is not running, '
                            'so members will stay on their direct links')
                    else:
                        self.log.warning(
                            f'relay route command failed '
                            f'({" ".join(command)}): {detail}')
            except subprocess.TimeoutExpired:
                self.log.warning(
                    f'relay route command timed out: {" ".join(command)}')
            except OSError as exc:
                self.log.warning(
                    f'relay route command could not run '
                    f'({" ".join(command)}): {exc}')

    # ── kernel prerequisites ──────────────────────────────────────────────────

    def _harden(self, namespace: str, forwarding: bool) -> None:
        """Disable the ICMP-redirect behaviour that would undo the relay."""
        key = f'{namespace}:{forwarding}'
        if key in self._hardened:
            return
        self._hardened.add(key)

        settings = [
            'net.ipv4.conf.all.accept_redirects=0',
            f'net.ipv4.conf.{self.iface}.accept_redirects=0',
            # Traffic arrives from the head but the reply routes back through
            # it too, so strict reverse-path filtering would be consistent;
            # loose mode simply removes the failure mode entirely.
            'net.ipv4.conf.all.rp_filter=2',
            f'net.ipv4.conf.{self.iface}.rp_filter=2',
        ]

        if forwarding:
            settings += [
                'net.ipv4.ip_forward=1',
                'net.ipv4.conf.all.send_redirects=0',
                f'net.ipv4.conf.{self.iface}.send_redirects=0',
            ]

        self._enqueue(self._netns(namespace, 'sysctl', '-w', *settings))

    def _ensure_forwarding(self, ch_id: int) -> None:
        if self._forwarding_ch == ch_id:
            return
        self._forwarding_ch = ch_id
        self._harden(self.gcs_netns, forwarding=False)
        self._harden(self.uav_netns(ch_id), forwarding=True)

    # ── route application ─────────────────────────────────────────────────────

    def _install(self, member: int, ch_id: int) -> None:
        member_ip = self.uav_ip(member)
        ch_ip = self.uav_ip(ch_id)
        gcs_ip = self.gcs_ip()

        self._ensure_forwarding(ch_id)
        self._harden(self.uav_netns(member), forwarding=False)

        self._enqueue(
            self._netns(
                self.gcs_netns, 'ip', 'route', 'replace', f'{member_ip}/32',
                'via', ch_ip, 'dev', self.iface, 'onlink'),
            self._netns(
                self.uav_netns(member), 'ip', 'route', 'replace',
                f'{gcs_ip}/32', 'via', ch_ip, 'dev', self.iface, 'onlink'),
        )
        self.relayed[member] = ch_id

    def _remove(self, member: int) -> None:
        self._enqueue(
            self._netns(
                self.gcs_netns, 'ip', 'route', 'del',
                f'{self.uav_ip(member)}/32'),
            self._netns(
                self.uav_netns(member), 'ip', 'route', 'del',
                f'{self.gcs_ip()}/32'),
        )
        self.relayed.pop(member, None)

    # ── policy ────────────────────────────────────────────────────────────────

    def update(
        self,
        ch_id: int,
        direct_snr_db: Dict[int, float],
        hop_snr_db: Dict[int, float],
    ) -> List[str]:
        """
        Called once per election tick.

        ch_id          elected primary cluster head (0 = none)
        direct_snr_db  {uav_id: SNR of its direct GCS link}
        hop_snr_db     {uav_id: SNR of its link to ch_id}

        Returns human-readable descriptions of the changes made, for logging
        and for /cluster/relay.
        """
        if not self.enabled:
            return []

        changes: List[str] = []

        # No head, or the head itself: nothing can be relayed.
        if ch_id == 0:
            for member in list(self.relayed):
                self._remove(member)
                changes.append(f'UAV{member} direct (no cluster head)')
            self._pending.clear()
            return changes

        # A new head must inherit the relays immediately — the old head may
        # already be out of range, so waiting out a streak would strand them.
        for member, old_ch in list(self.relayed.items()):
            if old_ch != ch_id:
                if member == ch_id:
                    self._remove(member)
                    changes.append(f'UAV{member} direct (became cluster head)')
                else:
                    self._install(member, ch_id)
                    changes.append(
                        f'UAV{member} relay moved UAV{old_ch} -> UAV{ch_id}')

        for member in range(1, self.num_uavs + 1):
            if member == ch_id:
                self._pending.pop(member, None)
                continue

            direct = direct_snr_db.get(member, -100.0)
            hop = hop_snr_db.get(member, -100.0)
            active = member in self.relayed

            if active:
                # Leave the relay only once the direct link is comfortably
                # back, or the relay hop itself has gone bad.
                want = not (direct > self.exit_snr_db or
                            hop < self.min_hop_snr_db)
            else:
                want = (direct < self.enter_snr_db and
                        hop >= self.min_hop_snr_db)

            if want == active:
                self._pending.pop(member, None)
                continue

            desired, streak = self._pending.get(member, (want, 0))
            streak = streak + 1 if desired == want else 1
            self._pending[member] = (want, streak)

            if streak < self.consecutive:
                continue

            self._pending.pop(member, None)

            if want:
                self._install(member, ch_id)
                changes.append(
                    f'UAV{member} -> relay via UAV{ch_id} '
                    f'(direct {direct:.1f} dB < {self.enter_snr_db:.1f}, '
                    f'hop {hop:.1f} dB)')
            else:
                self._remove(member)
                changes.append(
                    f'UAV{member} -> direct '
                    f'(direct {direct:.1f} dB, hop {hop:.1f} dB)')

        return changes

    def describe(self) -> str:
        if not self.enabled:
            return 'relay disabled'
        if not self.relayed:
            return 'all direct'
        return ' '.join(
            f'UAV{member}->UAV{ch}'
            for member, ch in sorted(self.relayed.items()))

    def shutdown(self) -> None:
        """Restore plain direct routing so a killed run leaves no /32 behind."""
        for member in list(self.relayed):
            self._remove(member)
        with self._queue_cv:
            self._stop = True
            self._queue_cv.notify()
        self._worker.join(timeout=15.0)
