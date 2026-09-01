#!/usr/bin/env python3
"""
world_pos_publisher.py
──────────────────────
Fixes the missing position feed: nothing was publishing /uav_world_positions,
so NS-3's UAV nodes stayed frozen at their initial formation while the drones
actually flew (its distance path-loss never tracked the real separation).

This node is the single source of truth for node world positions. It reads
GROUND-TRUTH poses from Gazebo (gazebo_msgs/ModelStates, published by the
libgazebo_ros_state.so world plugin) and republishes them on
/uav_world_positions in the exact frame the obstacle ray-caster uses, as:

    [id, x, y, z, id, x, y, z, ...]

Both consumers subscribe to this:
    - NS-3  (three_uav_tapbridge_integrated) -> moves its nodes
    - the Gazebo obstacle plugin             -> ray-casts from these poses

NODE ID CONVENTION (CHANGED when the GCS was added -- ids are now identical to
NS-3 node ids, so nothing anywhere applies an offset):

    gcs_enabled=True  (default):  id 0 -> GCS,  id k>=1 -> model "<prefix>k"
                                  i.e. 0=gcs, 1=iris_1, 2=iris_2, 3=iris_3
    gcs_enabled=False (legacy):   id k    -> model "<prefix>(k+1)"
                                  i.e. 0=iris_1, 1=iris_2, 2=iris_3

The GCS is static, so its z is the model pose plus gcs_antenna_height -- the
link starts at the antenna on top of the mast, not at the base of the cabinet.
That offset must match <gcs_antenna_height> in the world's obstacle_raycast
plugin block, or NS-3 and the ray-caster will disagree about where the GCS is.

Prereq: the world must load the state plugin so /model_states exists, e.g. add
to your .world (inside <world>):
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>

Run:  source /opt/ros/humble/setup.bash ; python3 world_pos_publisher.py
Params (ros2 --ros-args -p name:=val):
    model_states_topic (default /gazebo/model_states)
    uav_prefix (default iris_)      n_uavs (default 3)   rate_hz (default 10.0)
    gcs_enabled (default True)      gcs_model (default gcs)
    gcs_antenna_height (default 2.9)   stale_after (default 2.0)
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    raise SystemExit("gazebo_msgs not found — source your ROS2/gazebo_ros install.")


class WorldPosPublisher(Node):
    def __init__(self):
        super().__init__('world_pos_publisher')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('uav_prefix', 'iris_')
        self.declare_parameter('n_uavs', 3)
        self.declare_parameter('rate_hz', 10.0)
        # ADDED: ground control station as node 0.
        self.declare_parameter('gcs_enabled', True)
        self.declare_parameter('gcs_model', 'gcs')
        self.declare_parameter('gcs_antenna_height', 2.9)
        # Seconds without /model_states before this node stops publishing.
        # Gazebo's state plugin runs at 20 Hz, so 2 s is ~40 missed frames.
        self.declare_parameter('stale_after', 2.0)
        # Copy one node's position onto another: "dst:src[,dst:src...]".
        #
        # An ns-3 node that the feed never covers keeps its CLI-default start
        # position for the whole run -- silently, since a frozen node still
        # carries traffic and still answers pings. It is only visible in
        # CheckIntegration()'s "missing node IDs" line.
        #
        # The case this exists for: the Pi edge node is ns-3 node 2 while the
        # aircraft it serves is node 1, so with one UAV in the world node 2 is
        # never fed. Both are on the SAME airframe, so "2:1" is not a fudge --
        # it is the physically correct position for a companion computer bolted
        # next to the autopilot. Remove it once the Pi shares node 1.
        self.declare_parameter('mirror', '')
        self.declare_parameter('node_map', '')

        topic = self.get_parameter('model_states_topic').value
        self.prefix = self.get_parameter('uav_prefix').value
        self.n_uavs = int(self.get_parameter('n_uavs').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.gcs_enabled = bool(self.get_parameter('gcs_enabled').value)
        self.gcs_model = self.get_parameter('gcs_model').value
        self.gcs_h = float(self.get_parameter('gcs_antenna_height').value)
        self.stale_after = float(self.get_parameter('stale_after').value)

        # "2:1,3:1" -> {2: 1, 3: 1}. Parsed once here so a malformed value fails
        # at startup rather than silently publishing nothing for those nodes.
        self.mirror = {}
        spec = str(self.get_parameter('mirror').value).strip()
        if spec:
            for pair in spec.split(','):
                dst, _, src = pair.partition(':')
                if not src:
                    raise ValueError(
                        f"mirror entry '{pair}' is not dst:src (e.g. 2:1)")
                self.mirror[int(dst)] = int(src)

        # "3:iris_2,5:iris_4" -> {3: 'iris_2', 5: 'iris_4'}. An EXPLICIT node ->
        # model assignment, for layouts the n_uavs loop below cannot express.
        #
        # That loop walks node id and model number together (node k <- iris_k),
        # which assumes aircraft occupy nodes 1, 2, 3 ... consecutively. The HITL
        # topology interleaves them with companion computers:
        #     node 1 = SITL1   node 2 = Pi 1   node 3 = SITL2   node 4 = Pi 2
        # so the second aircraft belongs on node 3, and no value of n_uavs can
        # say that -- n_uavs=2 would put it on node 2, the Pi's slot.
        #
        # Entries here WIN over the automatic rule, and mirrors fill the rest:
        #     -p n_uavs:=1 -p node_map:=3:iris_2 -p mirror:=2:1,4:3
        self.node_map = {}
        spec = str(self.get_parameter('node_map').value).strip()
        if spec:
            for pair in spec.split(','):
                nid, _, model = pair.partition(':')
                if not model:
                    raise ValueError(
                        f"node_map entry '{pair}' is not id:model (e.g. 3:iris_2)")
                self.node_map[int(nid)] = model.strip()

        self._latest = None          # last ModelStates msg
        self._latest_t = 0.0         # monotonic time it arrived
        self._stale = False
        self._warned = False
        self._gcs_warned = False
        self._mirror_warned = False
        self._map_warned = False
        self._latest_topic = topic
        self.sub = self.create_subscription(ModelStates, topic, self._on_states, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/uav_world_positions', 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Relaying {topic} -> /uav_world_positions "
            f"({self.n_uavs} UAVs, prefix '{self.prefix}', {rate:.0f} Hz)")
        if self.node_map:
            self.get_logger().info(
                "Node map: " + ", ".join(f"node {n} <- {m}"
                                         for n, m in self.node_map.items()))
        if self.mirror:
            self.get_logger().info(
                "Mirroring: " + ", ".join(f"node {d} <- node {s}"
                                          for d, s in self.mirror.items()))
        if self.gcs_enabled:
            self.get_logger().info(
                f"GCS enabled: model '{self.gcs_model}' -> id 0 "
                f"(antenna +{self.gcs_h:.2f} m); UAVs are ids 1..{self.n_uavs}")
        else:
            self.get_logger().warn(
                "GCS disabled -- legacy id convention (id 0 = first UAV). "
                "NS-3 uses ids as-is (0=GCS), so its node 0 will be given "
                "UAV1's position and node 3 will never be fed. Only use this "
                "with a matching NS-3 build; there is no offset flag anymore.")

    def _on_states(self, msg):
        self._latest = msg
        self._latest_t = time.monotonic()
        if self._stale:
            self.get_logger().info("/model_states is live again — resuming.")
            self._stale = False

    def _tick(self):
        if self._latest is None:
            if not self._warned:
                self.get_logger().warn(
                    "No /model_states yet — is libgazebo_ros_state.so loaded in the world?")
                self._warned = True
            return

        # ── Staleness guard ──────────────────────────────────────────────
        # WITHOUT THIS, killing Gazebo does not stop this node: `self._latest`
        # is never invalidated, so it happily rebroadcasts the last frame it
        # ever saw at `rate_hz` forever. Downstream that is indistinguishable
        # from a fleet hovering perfectly still — NS-3 keeps computing path
        # loss from frozen coordinates, the recorder keeps logging a healthy
        # `pos_age_s` because the TOPIC is live, and nothing anywhere reports
        # a problem.
        #
        # Observed for real on 2026-07-21: gzserver had exited but this script
        # was still publishing small_city_base.world's start-of-run poses, and
        # silently corrupted a live-recorder test.
        #
        # Going silent is the right failure mode: NS-3's ApplyFeed() simply
        # receives nothing, its integration check reports the missing nodes,
        # and the recorder's pos_age_s starts climbing.
        age = time.monotonic() - self._latest_t
        if age > self.stale_after:
            if not self._stale:
                self.get_logger().error(
                    f"/model_states has been silent for {age:.1f}s "
                    f"(> stale_after={self.stale_after}s). Gazebo has probably "
                    f"exited. NOT publishing stale positions — downstream would "
                    f"read them as a stationary fleet.")
                self._stale = True
            return

        # Build a name -> index lookup once per tick (model set can change).
        names = list(self._latest.name)
        out = []
        found = 0

        # ── node 0: GCS ──────────────────────────────────────────────────
        # Static, but relayed on the same topic as everything else so NS-3 has
        # ONE source of truth for every node position. Publishing it here also
        # means NS-3 can never disagree with the ray-caster about where the
        # ground station is.
        if self.gcs_enabled:
            idx = next((k for k, nm in enumerate(names)
                        if nm.startswith(self.gcs_model)), None)
            if idx is None:
                if not self._gcs_warned:
                    self.get_logger().warn(
                        f"GCS model '{self.gcs_model}' not found in {len(names)} "
                        f"Gazebo models — GCS links will not be published. "
                        f"Is the <model name=\"{self.gcs_model}\"> block in the world?")
                    self._gcs_warned = True
            else:
                p = self._latest.pose[idx].position
                out.extend([0.0, float(p.x), float(p.y), float(p.z) + self.gcs_h])
                found += 1

        # ── nodes 1..N: UAVs ─────────────────────────────────────────────
        # With the GCS present, node id k maps to model "<prefix>k" (the +1
        # that used to be here is absorbed by the shifted id).
        base = 1 if self.gcs_enabled else 0
        poses = {}                            # nid -> (x, y, z), for mirroring
        for k in range(self.n_uavs):
            # base + k, NOT base + 1: the id must advance with the loop. Pinned
            # to a constant it publishes every aircraft onto the SAME node and
            # leaves node 1 with no position at all -- ns-3 then keeps node 1 at
            # its initial formation coordinate, kilometres from where the drone
            # really is, and every link metric for it is fiction.
            nid = base + k
            if nid in self.node_map:
                continue                       # explicit assignment wins; see below
            want = f"{self.prefix}{k + 1}"     # always iris_1..iris_N
            idx = next((j for j, nm in enumerate(names) if nm.startswith(want)), None)
            if idx is None:
                continue
            p = self._latest.pose[idx].position
            poses[nid] = (float(p.x), float(p.y), float(p.z))
            out.extend([float(nid), float(p.x), float(p.y), float(p.z)])
            found += 1

        # ── explicitly mapped nodes ──────────────────────────────────────
        # For layouts the consecutive rule cannot express (see node_map above).
        # Emitted before the mirrors so a mirror can take its source from one.
        for nid, model in self.node_map.items():
            idx = next((j for j, nm in enumerate(names)
                        if nm.startswith(model)), None)
            if idx is None:
                if not self._map_warned:
                    self.get_logger().warn(
                        f"node_map {nid}:{model} — no model named '{model}*' in "
                        f"{self._latest_topic}; node {nid} stays frozen at its "
                        "ns-3 default.")
                continue
            p = self._latest.pose[idx].position
            poses[nid] = (float(p.x), float(p.y), float(p.z))
            out.extend([float(nid), float(p.x), float(p.y), float(p.z)])
            found += 1
        self._map_warned = True

        # ── mirrored nodes ───────────────────────────────────────────────
        # Emitted after the real ones so a mirror can never shadow a node the
        # feed actually covers.
        for dst, src in self.mirror.items():
            if dst in poses:
                continue                      # real position wins
            if src not in poses:
                if not self._mirror_warned:
                    self.get_logger().warn(
                        f"mirror {dst}:{src} — source node {src} has no position, "
                        f"so node {dst} stays frozen at its ns-3 default.")
                    self._mirror_warned = True
                continue
            x, y, z = poses[src]
            out.extend([float(dst), x, y, z])
            found += 1

        if found:
            self.pub.publish(Float32MultiArray(data=out))


def main():
    rclpy.init()
    node = WorldPosPublisher()
    try:
        rclpy.spin(node)
    # Ctrl+C raises KeyboardInterrupt; SIGTERM (how the launcher stops this)
    # raises ExternalShutdownException and already shut the context down, so
    # calling rclpy.shutdown() again below raises RCLError. Catching only the
    # first printed a traceback on every clean teardown.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
