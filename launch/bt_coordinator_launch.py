import V2_2.launch.bt_coordinator_launch as bt_coordinator_launch
from V2_2.launch.bt_coordinator_launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('robot_ip_1', default_value='192.168.0.101', description='IP address of robot 1'),
        DeclareLaunchArgument('robot_ip_2', default_value='192.168.0.102', description='IP address of robot 2'),
        DeclareLaunchArgument('bt_xml_file', default_value='$(find bt_coordinator)/config/bt_struct.xml', description='Path to Behavior Tree XML'),

        # Log info action
        LogInfo(msg="Starting BT Coordinator Node..."),

        # Launch the coordinator node
        Node(
            package='bt_coordinator',
            executable='bt_coordinator_node',
            name='bt_coordinator_node',
            output='screen',
            parameters=[{
                'robot_ip_1': LaunchConfiguration('robot_ip_1'),
                'robot_ip_2': LaunchConfiguration('robot_ip_2'),
                'bt_xml_file': LaunchConfiguration('bt_xml_file')
            }],
            remappings=[],
        )
    ])
