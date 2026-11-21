from pathlib import Path
import atexit
import shutil
from tempfile import mkdtemp
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction, SetEnvironmentVariable, AppendEnvironmentVariable, GroupAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

# Keep rewritten parameter files for the lifetime of this launch.
scratch = mkdtemp(prefix="apparel_nav_")
atexit.register(shutil.rmtree, scratch, ignore_errors=True)


def generate_launch_description():
    store = Path(share("apparel_store"))
    robot = Path(share("warehouse_amr"))
    nav = Path(share("nav2_bringup"))
    slam = Path(share("slam_toolbox"))
    tmp = Path(scratch)
    params = yaml.safe_load((nav/"params/nav2_params.yaml").read_text())
    params["amcl"]["ros__parameters"].update(
        set_initial_pose=True, initial_pose=dict(x=0.,y=0.,z=0.,yaw=0.),
        base_frame_id="base_footprint", scan_topic="/scan", laser_max_range=12.)
    for key in ("global_costmap","local_costmap"):
        p=params[key][key]["ros__parameters"]
        p.pop("footprint",None)
        p.update(robot_radius=.57,robot_base_frame="base_footprint")
        p["inflation_layer"]["inflation_radius"]=.8
        # Keep live LaserScan marking + clearing in both costmaps.
        layers=["static_layer"] if key=="global_costmap" else []
        p["plugins"]=layers+["obstacle_layer","inflation_layer"]
        p["obstacle_layer"]={"plugin":"nav2_costmap_2d::ObstacleLayer","enabled":True,
                             "observation_sources":"scan","scan":{
                                  "topic":"/scan","data_type":"LaserScan","marking":True,
                                  "clearing":True,"inf_is_valid":True,"obstacle_max_range":8.,
                                  "raytrace_max_range":10.,"max_obstacle_height":2.}}
    controller=params["controller_server"]["ros__parameters"]
    controller["general_goal_checker"].update(xy_goal_tolerance=.12,yaw_goal_tolerance=.10)
    if "MPPI" in controller["FollowPath"]["plugin"]:
        controller["FollowPath"].update(batch_size=650,vx_max=.45,wz_max=.9)
    for key in ("controller_server","velocity_smoother","behavior_server","collision_monitor"):
        params[key]["ros__parameters"]["enable_stamped_cmd_vel"]=False
    (tmp/"nav.yaml").write_text(yaml.safe_dump(params))
    mapping=yaml.safe_load((slam/"config/mapper_params_online_async.yaml").read_text())
    mapping["slam_toolbox"]["ros__parameters"].update(
        base_frame="base_footprint", odom_frame="odom", map_frame="survey_map",
        scan_topic="/scan", map_name="/survey/map", transform_publish_period=0.,
        map_update_interval=3.,resolution=.05,max_laser_range=12.,use_sim_time=True)
    (tmp/"survey.yaml").write_text(yaml.safe_dump(mapping))
    rv=yaml.safe_load((robot/"rviz/warehouse.rviz").read_text())
    manager=rv["Visualization Manager"]
    manager["Global Options"]["Fixed Frame"]="map"
    manager["Displays"].insert(0,{"Class":"rviz_default_plugins/Map","Name":"Store map",
        "Enabled":True,"Value":True,"Alpha":.6,"Topic":{"Value":"/map",
        "Durability Policy":"Transient Local","Reliability Policy":"Reliable","Depth":1}})
    (tmp/"store.rviz").write_text(yaml.safe_dump(rv))
    gui=LaunchConfiguration("gazebo_gui")
    map_file=LaunchConfiguration("map")
    rfid = Path(share("apparel_rfid"))
    world=str(rfid/"worlds/store.sdf")
    urdf=rfid/"urdf/rfid.urdf"
    navlaunch=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/"launch/bringup_launch.py")),
        launch_arguments={"map":map_file,"params_file":str(tmp/"nav.yaml"),"slam":"False",
                          "use_sim_time":"True","autostart":"True","use_composition":"False"}.items())
    survey=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/"launch/online_async_launch.py")),
        launch_arguments={"slam_params_file":str(tmp/"survey.yaml"),"use_sim_time":"true"}.items())
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui",default_value="true"),
        DeclareLaunchArgument("rviz",default_value="true"),
        DeclareLaunchArgument("map",default_value=str(store/"maps/store.yaml")),
        SetEnvironmentVariable("GZ_PARTITION","apparel_store"),
        SetEnvironmentVariable("QT_QPA_PLATFORM","xcb"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",str(store.parent)),
        ExecuteProcess(cmd=["gz","sim","-r",world],output="screen",condition=IfCondition(gui)),
        ExecuteProcess(cmd=["gz","sim","-r","-s",world],output="screen",condition=UnlessCondition(gui)),
        Node(package="robot_state_publisher",executable="robot_state_publisher",
             parameters=[{"use_sim_time":True,"robot_description":urdf.read_text()}]),
        Node(package="ros_gz_bridge",executable="parameter_bridge",
             parameters=[{"config_file":str(robot/"config/bridge.yaml"),"use_sim_time":True}]),
        Node(package="warehouse_amr",executable="cmd_watchdog.py"),
        TimerAction(period=4.,actions=[Node(package="ros_gz_sim",executable="create",output="screen",
            arguments=["-world","apparel","-name","warehouse_amr","-file",str(urdf),"-z","0.025"])]),
        Node(package="ros_gz_bridge",executable="parameter_bridge",name="rfid_pose_bridge",
             arguments=["/rfid/model_pose@nav_msgs/msg/Odometry[gz.msgs.Odometry"],parameters=[{"use_sim_time":True}]),
        Node(package="apparel_rfid",executable="sensor.py",parameters=[{"use_sim_time":True}],output="screen"),
        navlaunch,
        GroupAction(actions=[SetRemap(src="/map",dst="/survey/map"),
                             SetRemap(src="/map_metadata",dst="/survey/map_metadata"),survey]),
        Node(package="rviz2",executable="rviz2",arguments=["-d",str(tmp/"store.rviz")],
             parameters=[{"use_sim_time":True}],condition=IfCondition(LaunchConfiguration("rviz")))
    ])
