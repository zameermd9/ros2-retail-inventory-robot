from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('warehouse_amr'))
    urdf = share / 'urdf' / 'warehouse_amr.urdf'
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('software_rendering', default_value='1',
                              description='1 for VMware software rendering; 0 for working GPU'),
        DeclareLaunchArgument('partition', default_value='warehouse_custom_amr'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', LaunchConfiguration('software_rendering')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-v', '3', str(share / 'worlds' / 'warehouse.sdf')],
                       output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf.read_text(), 'use_sim_time': True}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'), 'use_sim_time': True}],
             output='screen'),
        Node(package='warehouse_amr', executable='cmd_watchdog.py', output='screen'),
        TimerAction(period=3.0, actions=[
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'warehouse', '-name', 'warehouse_amr',
                            '-file', str(urdf), '-x', '0', '-y', '0', '-z', '0.025']),
        ]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'warehouse.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
