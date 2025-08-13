#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare('yuilrobotics_description').find('yuilrobotics_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'yuilrobotics.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 1) Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'pause': 'false'}.items()
    )

    # 2) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        remappings=[('joint_states', '/joint_states_calib')],
        output='screen'
    )

    # 3) spawn_entity
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'yuilrobotics'],
        output='screen'
    )

    # 4) parameter_manager (오프셋 보정 퍼블리셔)
    parameter_manager = Node(
        package='yuilrobotics_description',
        executable='parameter_manager',
        name='parameter_manager',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('yuilrobotics_description'),
                'config',
                'parameter_manager.yaml'
            ])
        ]
    )

    # 5) controller_manager 파라미터 먼저 로드 (YAML)
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_j1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint1_position_controller'],
        output='screen'
    )
    load_j2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint2_position_controller'],
        output='screen'
    )
    load_j3 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint3_position_controller'],
        output='screen'
    )
    load_j4 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint4_position_controller'],
        output='screen'
    )
    load_j5 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint5_position_controller'],
        output='screen'
    )
    load_j6 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint6_position_controller'],
        output='screen'
    )

    # 7) 순서: spawn → (cm yaml load) → jsb → j1 → j2 → ... → j6
    ev1 = RegisterEventHandler(OnProcessExit(target_action=spawn,       on_exit=[load_jsb]))
    ev2 = RegisterEventHandler(OnProcessExit(target_action=load_jsb,    on_exit=[load_j1]))
    ev3 = RegisterEventHandler(OnProcessExit(target_action=load_j1,     on_exit=[load_j2]))
    ev4 = RegisterEventHandler(OnProcessExit(target_action=load_j2,     on_exit=[load_j3]))
    ev5 = RegisterEventHandler(OnProcessExit(target_action=load_j3,     on_exit=[load_j4]))
    ev6 = RegisterEventHandler(OnProcessExit(target_action=load_j4,     on_exit=[load_j5]))
    ev7 = RegisterEventHandler(OnProcessExit(target_action=load_j5,     on_exit=[load_j6]))

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        parameter_manager,
        ev1, ev2, ev3, ev4, ev5, ev6, ev7
    ])
