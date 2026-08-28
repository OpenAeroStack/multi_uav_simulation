# Three-UAV namespace launcher

Run from the repository as a normal user:

```bash
bash scripts/netns_3uav/launch_three_uav_netns.sh
```

The launcher provisions `gcsns`, `uav1ns`, `uav2ns`, and `uav3ns`; four
bridge/TAP paths; three private SITL/Gazebo management links; the shared NS-3
channel; the three-UAV small-city Gazebo world; DDS agents on ports
2019/2020/2021; SITL instances 0/1/2; and three GCS-side drone bridges.

Set an NS-3 RNG run with:

```bash
RNG_RUN=2 bash scripts/netns_3uav/launch_three_uav_netns.sh
```

Logs are written under `/tmp/three_uav_netns` by default. The launcher does
not start a mission or vision workload.
