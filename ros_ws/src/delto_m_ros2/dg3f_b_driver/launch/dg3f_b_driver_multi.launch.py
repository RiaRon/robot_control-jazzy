# Copyright 2025 TESOLLO
#
# BSD 3-Clause License (same as your header)
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def make_group(ns, delto_ip_lc, delto_port_lc):
    # URDF/Xacro → robot_description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("dg3f_b_driver"), "urdf", "dg3f_b_ros2_control.xacro"]),
        " ",
        "delto_ip:=", delto_ip_lc,
        " ",
        "delto_port:=", delto_port_lc,
    ])
    robot_description = {"robot_description": robot_description_content}

    # 컨트롤러 YAML
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("dg3f_b_driver"), "config", "dg3f_b_controller.yaml"]
    )

    return GroupAction([
        PushRosNamespace(ns),

        # controller_manager (ros2_control_node)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            remappings=[
                ("~/robot_description", "/robot_description"),
            ],
            output="screen",
        ),

        # robot_state_publisher
        #(ex /left/joint_states, /right/joint_states)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
            remappings=[("/joint_states", f"/{ns}/joint_states")],
        ),

        # joint_state_broadcaster spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
            #
        ),

        # Delto Controller spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dg3f_b_controller"],
            output="screen",
        ),
    ])


def generate_launch_description():
    
    declared_args = [
        DeclareLaunchArgument("left_delto_ip",  default_value="192.168.9.51", description="Left gripper IP"),
        DeclareLaunchArgument("left_delto_port", default_value="502",            description="Left gripper port"),
        DeclareLaunchArgument("right_delto_ip", default_value="192.168.9.52", description="Right gripper IP"),
        DeclareLaunchArgument("right_delto_port", default_value="502",          description="Right gripper port"),
        # DeclareLaunchArgument("left_ns",  default_value="left"),
        # DeclareLaunchArgument("right_ns", default_value="right"),
    ]

    left_delto_ip   = LaunchConfiguration("left_delto_ip")
    left_delto_port = LaunchConfiguration("left_delto_port")
    right_delto_ip   = LaunchConfiguration("right_delto_ip")
    right_delto_port = LaunchConfiguration("right_delto_port")

    # ns is fixed to "left" and "right" for simplicity, if needed, can be changed to LaunchConfiguration 
    left_group  = make_group("left",  left_delto_ip,  left_delto_port)
    right_group = make_group("right", right_delto_ip, right_delto_port)

    return LaunchDescription(declared_args + [left_group, right_group])
