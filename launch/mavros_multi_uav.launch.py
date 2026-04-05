from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # MAVROS for UAV 1 - connects to SITL port 5760
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_uav1',
            namespace='/uav1',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5760',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }],
            output='screen'
        ),

        # MAVROS for UAV 2 - connects to SITL port 5770
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_uav2',
            namespace='/uav2',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5770',
                'gcs_url': '',
                'target_system_id': 2,
                'target_component_id': 1,
            }],
            output='screen'
        ),

        # MAVROS for UAV 3 - connects to SITL port 5780
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_uav3',
            namespace='/uav3',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5780',
                'gcs_url': '',
                'target_system_id': 3,
                'target_component_id': 1,
            }],
            output='screen'
        ),

    ])