#!/usr/bin/env python3
"""
world_pos_publisher.py
──────────────────────
Fixes the missing position feed: nothing was publishing /uav_world_positions,
so NS-3's UAV nodes stayed frozen at their initial formation while the drones
actually flew (its distance path-loss never tracked the real separation).

This node is the single source of truth for UAV world positions. It reads the
drones' GROUND-TRUTH poses from Gazebo (gazebo_msgs/ModelStates, published by
the libgazebo_ros_state.so world plugin) and republishes them on
/uav_world_positions in the exact frame the obstacle ray-caster uses, as:

    [id, x, y, z, id, x, y, z, ...]     (id 0-based, matching NS-3 node ids)

Both consumers subscribe to this:
    - NS-3  (three_uav_tapbridge_obstacle_loss) -> moves its UAV nodes
    - the Gazebo obstacle plugin                -> ray-casts from these poses

Model naming convention (same as the plugin): UAV id k  ->  model "<prefix><k+1>"
i.e. id 0 -> iris_1, id 1 -> iris_2, id 2 -> iris_3.

Prereq: the world must load the state plugin so /model_states exists, e.g. add
to your .world (inside <world>):
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>

Run:  source /opt/ros/humble/setup.bash ; python3 world_pos_publisher.py
Params (ros2 --ros-args -p name:=val):
    model_states_topic (default /gazebo/model_states)
    uav_prefix (default iris_)   n_uavs (default 3)   rate_hz (default 10.0)
"""

import rclpy
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

        topic = self.get_parameter('model_states_topic').value
        self.prefix = self.get_parameter('uav_prefix').value
        self.n_uavs = int(self.get_parameter('n_uavs').value)
        rate = float(self.get_parameter('rate_hz').value)

        self._latest = None          # last ModelStates msg
        self._warned = False
        self.sub = self.create_subscription(ModelStates, topic, self._on_states, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/uav_world_positions', 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Relaying {topic} -> /uav_world_positions "
            f"({self.n_uavs} UAVs, prefix '{self.prefix}', {rate:.0f} Hz)")

    def _on_states(self, msg):
        self._latest = msg

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
        for uid in range(self.n_uavs):
            want = f"{self.prefix}{uid + 1}"   # id 0 -> iris_1
            idx = next((k for k, nm in enumerate(names) if nm.startswith(want)), None)
            if idx is None:
                continue
            p = self._latest.pose[idx].position
            out.extend([float(uid), float(p.x), float(p.y), float(p.z)])
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
