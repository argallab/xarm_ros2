#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='')
    report_type = LaunchConfiguration('report_type', default='normal')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    dof = LaunchConfiguration('dof', default=7)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    # controllers_name = LaunchConfiguration('controllers_name', default='/joint_trajectory_controller/JointTrajectoryController')
    controllers_name = 'controllers'
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    # 1: xbox360 wired
    # 2: xbox360 wireless
    # 3: spacemouse wireless
    joystick_type = LaunchConfiguration('joystick_type', default=1)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    model1300 = LaunchConfiguration('model1300', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    robot_type = LaunchConfiguration('robot_type', default='xarm')

    moveit_config_package_name = 'xarm_moveit_config'
    moveit_controller_manager_key = 'moveit_simple_controller_manager'
    moveit_controller_manager_value = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'

    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    # robot_description_parameters
    # xarm_moveit_config/launch/lib/robot_moveit_config_lib.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory(moveit_config_package_name), 'launch', 'lib', 'robot_moveit_config_lib.py'))
    get_xarm_robot_description_parameters = getattr(mod, 'get_xarm_robot_description_parameters')
    robot_description_parameters = get_xarm_robot_description_parameters(
        xacro_urdf_file=PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
        # xacro_srdf_file=PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'srdf', 'xarm.srdf.xacro']),
        urdf_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns.perform(context).strip('/'),
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_d435i_links,
            'model1300': model1300,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'kinematics_suffix': kinematics_suffix,
        },
        srdf_arguments={
            'prefix': prefix,
            'dof': dof,
            'robot_type': robot_type,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_other_geometry': add_other_geometry,
        },
        arguments={
            'context': context,
            'xarm_type': xarm_type,
        }
    )

    load_yaml = getattr(mod, 'load_yaml')
    servo_yaml = load_yaml('xarm_moveit_servo', "config/xarm_moveit_servo_config.yaml")
    servo_yaml['move_group_name'] = xarm_type
    xarm_traj_controller = '{}{}_traj_controller'.format(prefix.perform(context), xarm_type)
    servo_yaml['command_out_topic'] = '/{}/joint_trajectory'.format(xarm_traj_controller)
    servo_params = {"moveit_servo": servo_yaml}
    controllers = ['joint_state_broadcaster', xarm_traj_controller]

    # # controllers_yaml = load_yaml(moveit_config_package_name, 'config', xarm_type, '{}.yaml'.format(controllers_name.perform(context)))
    # controllers_yaml = load_yaml(moveit_config_package_name, 'config', 'controllers_servo.yaml')
    # kinematics_yaml = robot_description_parameters['robot_description_kinematics']
    # joint_limits_yaml = robot_description_parameters.get('robot_description_planning', None)
    # # if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
    # #     controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    # # elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
    # #     controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    # if add_gripper.perform(context) in ('True', 'true'):
    #     gripper_controllers_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), '{}.yaml'.format(controllers_name.perform(context)))
    #     gripper_ompl_planning_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'ompl_planning.yaml')
    #     gripper_joint_limits_yaml = load_yaml(moveit_config_package_name, 'config', '{}_gripper'.format(robot_type.perform(context)), 'joint_limits.yaml')

    #     if gripper_controllers_yaml and 'controller_names' in gripper_controllers_yaml:
    #         for name in gripper_controllers_yaml['controller_names']:
    #             if name in gripper_controllers_yaml:
    #                 if name not in controllers_yaml['controller_names']:
    #                     controllers_yaml['controller_names'].append(name)
    #                 controllers_yaml[name] = gripper_controllers_yaml[name]
    #     if joint_limits_yaml and gripper_joint_limits_yaml:
    #         joint_limits_yaml['joint_limits'].update(gripper_joint_limits_yaml['joint_limits'])

    # add_prefix_to_moveit_params = getattr(mod, 'add_prefix_to_moveit_params')
    # add_prefix_to_moveit_params(
    #     controllers_yaml=controllers_yaml, 
    #     kinematics_yaml=kinematics_yaml, joint_limits_yaml=joint_limits_yaml, 
    #     prefix=prefix.perform(context))

    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'moveit.rviz'])
    # rviz_config_file = PathJoinSubstitution([FindPackageShare('xarm_moveit_servo'), 'rviz', 'servo.rviz'])
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',src/map_comparison/xarm_ros2/xarm_moveit_config/launch/_robot_moveit_common.launch.py
    #     arguments=['-d', rviz_config_file],
    #     parameters=[
    #         robot_description_parameters,
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static'),
    #     ]
    # )

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            # 'add_gripper': add_gripper if robot_type.perform(context) == 'xarm' else 'false',
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value,
            'add_realsense_d435i': add_realsense_d435i,
            'model1300': model1300,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'kinematics_suffix': kinematics_suffix,
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['{}{}/joint_states'.format(prefix.perform(context), hw_ns.perform(context))]}],
        remappings=[
            ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix.perform(context), xarm_type)),
        ],
    )


    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'dof': dof,
            'robot_type': robot_type,
            'ros2_control_plugin': ros2_control_plugin,
            'add_realsense_d435i': add_realsense_d435i,
            'add_d435i_links': add_d435i_links,
            'model1300': model1300,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'kinematics_suffix': kinematics_suffix,
        }.items(),
    )

    # Load controllers
    load_controllers = []
    for controller in controllers:
        load_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name='xarm_moveit_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[robot_description_parameters],
            ),
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='static_tf2_broadcaster',
                parameters=[{'child_frame_id': 'link_base', 'frame_id': 'world'}],
            ),
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_server',
                parameters=[
                    servo_params,
                    robot_description_parameters,
                ],
                # extra_arguments=[{'use_intra_process_comms': True}],
            )
            # ,
            # ComposableNode(
            #     package='xarm_moveit_servo',
            #     plugin='xarm_moveit_servo::JoyToServoPub',
            #     name='joy_to_servo_node',
            #     parameters=[
            #         servo_params,
            #         {
            #             'dof': dof, 
            #             'ros_queue_size': 10,
            #             'joystick_type': joystick_type,
            #         },
            #     ],
            #     # extra_arguments=[{'use_intra_process_comms': True}],
            # ),
            # ComposableNode(
            #     package='joy',
            #     plugin='joy::Joy',
            #     name='joy_node',
            #     parameters=[
            #         # {'autorepeat_rate': 50.0},
            #     ],
            #     # extra_arguments=[{'use_intra_process_comms': True}],
            # ),
        ],
        output='screen',
    )

    return [
        # rviz_node,
        robot_moveit_common_launch,
        joint_state_publisher_node,
        ros2_control_launch,
        container,
    ] + load_controllers


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
