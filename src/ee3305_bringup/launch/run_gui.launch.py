import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution, NotEqualsSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    pkg_ee3305_bringup = FindPackageShare("ee3305_bringup")
    nodes_param_file = PathJoinSubstitution([pkg_ee3305_bringup, "params", "run.yaml"])
    map_param_file = PathJoinSubstitution([pkg_ee3305_bringup, "params", "map_loader.yaml"])

    # Launch Args
    arg_x = DeclareLaunchArgument("x", default_value="-2.0", description="Initial x position")
    arg_y = DeclareLaunchArgument("y", default_value="-0.5", description="Initial y position")
    arg_map = DeclareLaunchArgument("map", default_value="turtlebot3_house", description="Map name")
    arg_rviz = DeclareLaunchArgument("rviz", default_value="True", description="Open RViz2")

    ld.add_action(arg_x)
    ld.add_action(arg_y)
    ld.add_action(arg_map)
    ld.add_action(arg_rviz)

    # 1. Map Loader Node
    node_map_loader = Node(
        package="ee3305_map_loader",
        executable="map_loader",
        output="screen",
        parameters=[
            map_param_file,
            {"filepath": PathJoinSubstitution([pkg_ee3305_bringup, "maps", LaunchConfiguration("map")])},
        ],
    )
    ld.add_action(node_map_loader)

    # 2. Static TF: map -> odom
    node_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "/map", "/odom"],
    )
    ld.add_action(node_static_tf)

    # 3. Vehicle Kinematics Simulator (Publishes /odom and odom->base_footprint TF based on /cmd_vel)
    node_sim_vehicle = Node(
        package="ee3305_py",
        executable="sim_vehicle",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": 0.0,
        }],
    )
    ld.add_action(node_sim_vehicle)

    # 4. Behavior Node
    node_behavior = Node(
        package="ee3305_py",
        executable="behavior",
        output="screen",
        emulate_tty=True,
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # 5. Planner Node (Dijkstra)
    node_planner = Node(
        package="ee3305_py",
        executable="planner",
        output="screen",
        emulate_tty=True,
        parameters=[nodes_param_file],
    )
    ld.add_action(node_planner)

    # 6. Controller Node (Pure Pursuit)
    node_controller = Node(
        package="ee3305_py",
        executable="controller",
        output="screen",
        emulate_tty=True,
        parameters=[nodes_param_file],
    )
    ld.add_action(node_controller)

    # 7. Web Mission Control GUI (http://localhost:8080)
    node_web_gui = Node(
        package="ee3305_py",
        executable="web_gui",
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(node_web_gui)

    # 8. RViz2 (Optional)
    if_rviz = IfCondition(EqualsSubstitution(LaunchConfiguration("rviz"), "True"))
    node_rviz = Node(
        condition=if_rviz,
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_ee3305_bringup, "rviz", "sim_truth.rviz"])],
        output="screen",
    )
    ld.add_action(node_rviz)

    return ld
