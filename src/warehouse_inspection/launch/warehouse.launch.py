from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('warehouse_amr'))
    mission = Path(get_package_share_directory('warehouse_inspection'))
    # The fast profile keeps the same geometry but reduces sensor rendering load.
    urdf = [str(share / 'urdf') + '/warehouse_amr', LaunchConfiguration('profile_suffix'), '.urdf']
    from launch.substitutions import Command, FindExecutable
    from launch_ros.parameter_descriptions import ParameterValue
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('profile_suffix', default_value='', choices=['', '_fast']),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('software_rendering', default_value='1',
                              description='1 for VMware software rendering; 0 for working GPU'),
        DeclareLaunchArgument('partition', default_value='inspection_mission'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', LaunchConfiguration('software_rendering')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(mission.parent)),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='dock_contact_bridge',
             arguments=['/inspection_dock/touched@std_msgs/msg/Bool[gz.msgs.Boolean']),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '3', str(mission / 'worlds' / 'inspection.sdf')],
                       output='screen', condition=IfCondition(LaunchConfiguration('gazebo_gui'))),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', str(mission / 'worlds' / 'inspection.sdf')],
                       output='screen', condition=UnlessCondition(LaunchConfiguration('gazebo_gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(Command([FindExecutable(name='cat'), ' ', *urdf]), value_type=str), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        Node(package='warehouse_amr', executable='cmd_watchdog.py', output='screen'),
        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'inspection', '-name', 'warehouse_amr',
                            '-file', urdf, '-x', '0', '-y', '0', '-z', '0.025']),
        ]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'warehouse.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
