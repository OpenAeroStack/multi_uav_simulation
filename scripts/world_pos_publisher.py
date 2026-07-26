#!/usr/bin/env python3
"""
world_pos_publisher.py
──────────────────────
Fixes the missing position feed: nothing was publishing /uav_world_positions,
so NS-3's UAV nodes stayed frozen at their initial formation while the drones
actually flew (its distance path-loss never tracked the real separation).

This node is the single source of truth for node world positions. It reads the
drones' GROUND-TRUTH poses from Gazebo (gazebo_msgs/ModelStates) and the GCS
antenna pose from gazebo_msgs/LinkStates (both published by the
libgazebo_ros_state.so world plugin) and republishes them on
/uav_world_positions in the exact frame the obstacle ray-caster uses, as:

    [id, x, y, z, id, x, y, z, ...]     (id matches the shared NS-3 node ids)

Both consumers subscribe to this:
    - NS-3  (three_uav_tapbridge_integrated) -> moves its 4 nodes (GCS + UAVs)
    - the Gazebo obstacle plugin             -> ray-casts from these poses

Node id convention (GCS = node 0, shared across Gazebo/ROS/NS-3, no offset):
    id 0      -> GCS antenna, read from the mast LINK ("<gcs_model>::<gcs_link>")
                 so node 0 sits at the live antenna height, not the ground origin
    id k >= 1 -> UAV model "<prefix>k"   i.e. iris_1 -> id 1, iris_2 -> id 2, ...

Prereq: the world must load the state plugin so /model_states exists, e.g. add
to your .world (inside <world>):
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>

Run:  source /opt/ros/humble/setup.bash ; python3 world_pos_publisher.py
Params (ros2 --ros-args -p name:=val):
    model_states_topic (default /gazebo/model_states)
    link_states_topic  (default /gazebo/link_states)
    uav_prefix (default iris_)   n_uavs (default 3)   rate_hz (default 10.0)
    gcs_enabled (default true)   gcs_model (default gcs)   gcs_link (default mast)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from gazebo_msgs.msg import ModelStates, LinkStates
except ImportError:
    raise SystemExit("gazebo_msgs not found — source your ROS2/gazebo_ros install.")


class WorldPosPublisher(Node):
    def __init__(self):
        super().__init__('world_pos_publisher')
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('link_states_topic', '/gazebo/link_states')
        self.declare_parameter('uav_prefix', 'iris_')
        self.declare_parameter('n_uavs', 3)
        self.declare_parameter('rate_hz', 10.0)
        # GCS = node 0. Its RF origin is the top of the antenna mast, so its pose
        # is read from Gazebo dynamically like the UAVs -- but from the mast LINK
        # (/link_states) rather than the model origin, which sits on the ground.
        # This puts node 0 at the live mast pose with no hardcoded height. The
        # value is published on the feed so NS-3 -- which has no other source for
        # it -- can place node 0. gcs_model/gcs_link are lookup names, not a
        # hardcoded position.
        self.declare_parameter('gcs_enabled', True)
        self.declare_parameter('gcs_model', 'gcs')
        self.declare_parameter('gcs_link', 'mast')

        topic = self.get_parameter('model_states_topic').value
        link_topic = self.get_parameter('link_states_topic').value
        self.prefix = self.get_parameter('uav_prefix').value
        self.n_uavs = int(self.get_parameter('n_uavs').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.gcs_enabled = bool(self.get_parameter('gcs_enabled').value)
        self.gcs_model = self.get_parameter('gcs_model').value
        # Gazebo names links "<model>::<link>" in LinkStates.
        self.gcs_link_name = f"{self.gcs_model}::{self.get_parameter('gcs_link').value}"

        self._latest = None          # last ModelStates msg
        self._latest_links = None    # last LinkStates msg (for the GCS mast)
        self._warned = False
        self._gcs_warned = False
        self.sub = self.create_subscription(ModelStates, topic, self._on_states, 10)
        self.link_sub = self.create_subscription(LinkStates, link_topic, self._on_links, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/uav_world_positions', 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Relaying {topic} -> /uav_world_positions "
            f"({self.n_uavs} UAVs, prefix '{self.prefix}', {rate:.0f} Hz"
            + (f", GCS link '{self.gcs_link_name}' as node 0)"
               if self.gcs_enabled else ", no GCS)"))

    def _on_states(self, msg):
        self._latest = msg

    def _on_links(self, msg):
        self._latest_links = msg

    def _tick(self):
        if self._latest is None:
            if not self._warned:
                self.get_logger().warn(
                    "No /model_states yet — is libgazebo_ros_state.so loaded in the world?")
                self._warned = True
            return

        # Build a name -> index lookup once per tick (model set can change).
        names = list(self._latest.name)
        out = []
        found = 0

        # Node id convention (GCS = node 0, shared across Gazebo/ROS/NS-3, no
        # offset):  id 0 -> GCS mast link, id k>=1 -> UAV model "<prefix>k".
        # Publishing UAVs at id 0.. used to collide with the GCS: node 0 landed
        # on iris_1, so the GCS never got its own position and no GCS<->UAV ray
        # was drawn. Node 0 is now the GCS, read from Gazebo like the UAVs.
        def emit(node_id, name_match):
            idx = next((k for k, nm in enumerate(names) if nm.startswith(name_match)), None)
            if idx is None:
                return False
            p = self._latest.pose[idx].position
            out.extend([float(node_id), float(p.x), float(p.y), float(p.z)])
            return True

        # id 0 -> GCS antenna mast, read from /link_states (an exact link-name
        # match). Using the mast LINK rather than the model origin puts node 0 at
        # the antenna height dynamically, with no hardcoded offset.
        if self.gcs_enabled:
            link_idx = None
            if self._latest_links is not None:
                link_idx = next((k for k, nm in enumerate(self._latest_links.name)
                                 if nm == self.gcs_link_name), None)
            if link_idx is not None:
                p = self._latest_links.pose[link_idx].position
                out.extend([0.0, float(p.x), float(p.y), float(p.z)])
                found += 1
            elif not self._gcs_warned:
                self.get_logger().warn(
                    f"GCS link '{self.gcs_link_name}' not in /link_states — node 0 "
                    f"will be uncovered (NS-3 CheckIntegration will flag it). "
                    f"Is libgazebo_ros_state.so loaded and the mast link named correctly?")
                self._gcs_warned = True

        # id k>=1 -> UAV model "<prefix>k"  (iris_1 -> id 1, iris_2 -> id 2, ...)
        for model_num in range(1, self.n_uavs + 1):
            if emit(model_num, f"{self.prefix}{model_num}"):
                found += 1

        if found:
            self.pub.publish(Float32MultiArray(data=out))


def main():
    rclpy.init()
    node = WorldPosPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()