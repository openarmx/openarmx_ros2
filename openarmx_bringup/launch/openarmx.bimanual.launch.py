# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
# To view a copy of this license, visit:
#
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip('/')
    return None


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, can_fd, right_can_interface, left_can_interface, control_mode, arm_prefix):
    """Generate robot description using xacro processing."""

    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    can_fd_str = context.perform_substitution(can_fd)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    control_mode_str = context.perform_substitution(control_mode)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    node_namespace_str = arm_prefix_str.strip('/')

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf", "robot", description_file_str
    )

    # Process xacro with required arguments
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "can_fd": can_fd_str,
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
            "control_mode": control_mode_str,
            "node_namespace": node_namespace_str,
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                        arm_type, use_fake_hardware, controllers_file, can_fd, right_can_interface, left_can_interface, arm_prefix, control_mode):
    """Spawn both robot state publisher and control nodes with shared robot description."""
    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type, use_fake_hardware, can_fd, right_can_interface, left_can_interface, control_mode, arm_prefix,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    if namespace:
        controllers_file_str = controllers_file_str.replace(
            "openarmx_v10_bimanual_controllers.yaml", "openarmx_v10_bimanual_controllers_namespaced.yaml"
        )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        namespace=namespace,
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn controller based on robot_controller argument."""
    namespace = namespace_from_context(context, arm_prefix)

    controller_manager_ref = f"/{namespace}/controller_manager" if namespace else "/controller_manager"

    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        robot_controller_left = "left_forward_position_controller"
        robot_controller_right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        robot_controller_left = "left_joint_trajectory_controller"
        robot_controller_right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[robot_controller_left,
                   robot_controller_right, "-c", controller_manager_ref],
    )

    return [robot_controller_spawner]


def effort_controller_spawner(context: LaunchContext, arm_prefix):
    """Spawn effort controllers for gravity compensation feedforward."""
    namespace = namespace_from_context(context, arm_prefix)
    controller_manager_ref = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )
    return [Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "left_forward_effort_controller",
            "right_forward_effort_controller",
            "-c", controller_manager_ref,
        ],
    )]


def gravity_comp_node_launcher(context: LaunchContext, description_package,
                                description_file, arm_type, use_fake_hardware,
                                can_fd, right_can_interface, left_can_interface,
                                control_mode, arm_prefix):
    """Write URDF to /tmp and launch gravity_comp_node."""
    namespace = namespace_from_context(context, arm_prefix)

    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type,
        use_fake_hardware, can_fd, right_can_interface, left_can_interface,
        control_mode, arm_prefix,
    )

    # Write URDF to a temp file so gravity_comp_node can load it via KDL
    urdf_path = "/tmp/v10_bimanual_gravity.urdf"
    with open(urdf_path, "w") as f:
        f.write(robot_description)

    return [Node(
        package="openarmx_gravity_comp",
        executable="gravity_comp_node",
        name="gravity_comp_node",
        namespace=namespace,
        output="screen",
        parameters=[{
            "urdf_path": urdf_path,
            "g_scale": 1.05,
            "enable_left": True,
            "enable_right": True,
            "verbose": False,
        }],
    )]


def generate_launch_description():
    """Generate launch description for OpenArmX bimanual configuration."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarmx_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm (e.g., v10).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarmx_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarmx_v10_bimanual_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),
        DeclareLaunchArgument(
            "can_fd",
            default_value="false",
            description="Enable CAN-FD for both arms (true) or use classic CAN (false).",
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="mit",
            choices=["mit", "csp"],
            description="Low-level control mode for motors: 'mit' motion-control or 'csp' position mode.",
        ),
        DeclareLaunchArgument(
            "enable_forward_effort",
            default_value="false",
            description="Enable gravity compensation feedforward (effort controllers + gravity_comp_node).",
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    rightcan_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")
    can_fd = LaunchConfiguration("can_fd")
    control_mode = LaunchConfiguration("control_mode")
    enable_forward_effort = LaunchConfiguration("enable_forward_effort")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, controllers_file, can_fd, rightcan_interface, left_can_interface, arm_prefix, control_mode]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "bimanual.rviz"]
    )

    # RViz node with dynamic namespace support
    rviz_node_func = OpaqueFunction(
        function=lambda context: [Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace_from_context(context, arm_prefix),
            output="log",
            arguments=["-d", rviz_config_file],
        )]
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=["joint_state_broadcaster",
                       "--controller-manager",
                       f"/{namespace_from_context(context, arm_prefix)}/controller_manager" if namespace_from_context(context, arm_prefix) else "/controller_manager"],
        )]
    )

    # Controller spawners
    controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[robot_controller, arm_prefix]
    )

    def gripper_controller_spawner_func(context):
        """只有在非forward_position_controller模式下才启动gripper_controller"""
        robot_controller_str = context.perform_substitution(robot_controller)

        # 如果是forward_position_controller，夹爪已经在手臂控制器中，不需要单独启动
        if robot_controller_str == "forward_position_controller":
            return []

        # 其他模式（如joint_trajectory_controller）需要启动gripper_controller
        return [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=["left_gripper_controller",
                       "right_gripper_controller", "-c",
                       f"/{namespace_from_context(context, arm_prefix)}/controller_manager" if namespace_from_context(context, arm_prefix) else "/controller_manager"],
        )]

    gripper_controller_spawner = OpaqueFunction(function=gripper_controller_spawner_func)

    effort_controller_spawner_func = OpaqueFunction(
        function=effort_controller_spawner,
        args=[arm_prefix]
    )

    gravity_comp_func = OpaqueFunction(
        function=gravity_comp_node_launcher,
        args=[description_package, description_file, arm_type,
              use_fake_hardware, can_fd, rightcan_interface,
              left_can_interface, control_mode, arm_prefix]
    )

    # Timing and sequencing
    LAUNCH_DELAY_SECONDS = 1.0
    delayed_joint_state_broadcaster = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_robot_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[controller_spawner_func],
    )
    delayed_gripper_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[gripper_controller_spawner],
    )

    delayed_effort_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[effort_controller_spawner_func],
    )

    delayed_gravity_comp = TimerAction(
        period=LAUNCH_DELAY_SECONDS + 1.0,
        actions=[gravity_comp_func],
    )

    forward_effort_group = GroupAction(
        condition=IfCondition(enable_forward_effort),
        actions=[
            delayed_effort_controller,
            delayed_gravity_comp,
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node_func,
        ] +
        [
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            delayed_gripper_controller,
            forward_effort_group,
        ]
    )
