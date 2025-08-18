#!/usr/bin/env python3
import os, tempfile, xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 공통 인자
    entity = LaunchConfiguration('entity')
    action = LaunchConfiguration('action')  # 'start' | 'respawn'

    # 길이 인자들 (xacro의 $(arg ...)와 1:1)
    L1x  = LaunchConfiguration('L1x')
    L1y  = LaunchConfiguration('L1y')
    L1z  = LaunchConfiguration('L1z')
    L2y  = LaunchConfiguration('L2y')
    L2z  = LaunchConfiguration('L2z')
    L3x = LaunchConfiguration('L3x')
    L3y = LaunchConfiguration('L3y')
    L3z = LaunchConfiguration('L3z')
    L4x = LaunchConfiguration('L4x')
    L4y = LaunchConfiguration('L4y')
    L5x = LaunchConfiguration('L5x')
    L5y = LaunchConfiguration('L5y')

    def _make_urdf(context):
        """xacro -> urdf 문자열/파일 생성 + L* 매핑 주입"""
        pkg_share = FindPackageShare('yuilrobotics_description').find('yuilrobotics_description')
        xacro_file = os.path.join(pkg_share, 'urdf', 'yuilrobotics.xacro')
        mappings = {
            'L1x': L1x.perform(context),
            'L1y': L1y.perform(context),
            'L1z': L1z.perform(context),
            'L2y': L2y.perform(context),
            'L2z': L2z.perform(context),
            'L3x': L3x.perform(context),
            'L3y': L3y.perform(context),
            'L3z': L3z.perform(context),
            'L4x': L4x.perform(context),
            'L4y': L4y.perform(context),
            'L5x': L5x.perform(context),
            'L5y': L5y.perform(context),
        }
        urdf_xml = xacro.process_file(xacro_file, mappings=mappings).toxml()
        tmp_urdf = os.path.join(tempfile.gettempdir(), 'yuilrobotics_lengths.urdf')
        with open(tmp_urdf, 'w') as f:
            f.write(urdf_xml)
        return urdf_xml, tmp_urdf

    def _start_flow(context, *args, **kwargs):
        """Gazebo 처음 시작 + 스폰"""
        urdf_xml, _ = _make_urdf(context)        

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
            parameters=[{'robot_description': urdf_xml}],
            remappings=[('joint_states', '/joint_states_calib')],
            output='screen'
        )

        # 3) spawn_entity
        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', entity.perform(context)],
            output='screen'
        )

        # 4) joint offset parameter_manager
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


        # 5) controller_manager 준비 기다렸다가 spawner로 로드(권장)
        wait_cm = ExecuteProcess(
            cmd=['bash','-lc',
                 'until ros2 service type /controller_manager/list_controllers >/dev/null 2>&1; do sleep 0.2; done; echo CM_READY'],
            output='screen'
        )
        sp_jsb = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint_state_broadcaster','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j1 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint1_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j2 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint2_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j3 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint3_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j4 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint4_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j5 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint5_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j6 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint6_position_controller','--controller-manager','/controller_manager'
        ], output='screen')

        ev1 = RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn,   on_exit=[wait_cm]))
        ev2 = RegisterEventHandler(event_handler=OnProcessExit(target_action=wait_cm, on_exit=[sp_jsb]))
        ev3 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_jsb,  on_exit=[sp_j1]))
        ev4 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j1,   on_exit=[sp_j2]))
        ev5 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j2,   on_exit=[sp_j3]))
        ev6 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j3,   on_exit=[sp_j4]))
        ev7 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j4,   on_exit=[sp_j5]))
        ev8 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j5,   on_exit=[sp_j6]))

        return [gazebo, rsp, spawn, parameter_manager,
                ev1, ev2, ev3, ev4, ev5, ev6, ev7, ev8]

    def _respawn_flow(context, *args, **kwargs):
        """실행 중 길이(L*)만 바꿔서 리스폰"""
        urdf_xml, tmp_urdf = _make_urdf(context)

        pause = ExecuteProcess(
            cmd=['ros2','service','call','/pause_physics','std_srvs/srv/Empty','{}'],
            output='screen'
        )
        delete = ExecuteProcess(
            cmd=['ros2','service','call','/delete_entity','gazebo_msgs/srv/DeleteEntity',
                 "{name: '"+entity.perform(context)+"'}"],
            output='screen'
        )
        set_rsp = ExecuteProcess(
            cmd=['bash','-lc', f'ros2 param set /robot_state_publisher robot_description "$(<{tmp_urdf})"'],
            output='screen'
        )
        spawn = ExecuteProcess(
            cmd=['ros2','run','gazebo_ros','spawn_entity.py','-file', tmp_urdf, '-entity', entity.perform(context)],
            output='screen'
        )
        wait_cm = ExecuteProcess(
            cmd=['bash','-lc',
                 'until ros2 service type /controller_manager/list_controllers >/dev/null 2>&1; do sleep 0.2; done; echo CM_READY'],
            output='screen'
        )
        unpause = ExecuteProcess(
            cmd=['ros2','service','call','/unpause_physics','std_srvs/srv/Empty','{}'],
            output='screen'
        )
        sp_jsb = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint_state_broadcaster','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j1 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint1_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j2 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint2_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j3 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint3_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j4 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint4_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j5 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint5_position_controller','--controller-manager','/controller_manager'
        ], output='screen')
        sp_j6 = ExecuteProcess(cmd=[
            'ros2','run','controller_manager','spawner',
            'joint6_position_controller','--controller-manager','/controller_manager'
        ], output='screen')

        ev1  = RegisterEventHandler(event_handler=OnProcessExit(target_action=pause,   on_exit=[delete]))
        ev2  = RegisterEventHandler(event_handler=OnProcessExit(target_action=delete,  on_exit=[set_rsp]))
        ev3  = RegisterEventHandler(event_handler=OnProcessExit(target_action=set_rsp, on_exit=[spawn]))
        ev4  = RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn,   on_exit=[wait_cm]))
        ev5  = RegisterEventHandler(event_handler=OnProcessExit(target_action=wait_cm, on_exit=[unpause]))
        ev6  = RegisterEventHandler(event_handler=OnProcessExit(target_action=unpause, on_exit=[sp_jsb]))
        ev7  = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_jsb,  on_exit=[sp_j1]))
        ev8  = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j1,   on_exit=[sp_j2]))
        ev9  = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j2,   on_exit=[sp_j3]))
        ev10 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j3,   on_exit=[sp_j4]))
        ev11 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j4,   on_exit=[sp_j5]))
        ev12 = RegisterEventHandler(event_handler=OnProcessExit(target_action=sp_j5,   on_exit=[sp_j6]))

        return [pause, ev1, ev2, ev3, ev4, ev5, ev6, ev7, ev8, ev9, ev10, ev11, ev12]

    # 인자 선언 + 분기
    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='yuilrobotics'),
        DeclareLaunchArgument('action', default_value='start'),  # 'start' | 'respawn'

        DeclareLaunchArgument('L1x',  default_value='0.16'),
        DeclareLaunchArgument('L1y',  default_value='-0.0769'),
        DeclareLaunchArgument('L1z',  default_value='0.235'),
        DeclareLaunchArgument('L2y',  default_value='0.0295'),
        DeclareLaunchArgument('L2z',  default_value='0.76'),
        DeclareLaunchArgument('L3x', default_value='0.3005'),
        DeclareLaunchArgument('L3y', default_value='0.1064'),
        DeclareLaunchArgument('L3z', default_value='0.2'),
        DeclareLaunchArgument('L4x', default_value='0.5165'),
        DeclareLaunchArgument('L4y', default_value='-0.0685'),
        DeclareLaunchArgument('L5x', default_value='0.063'),
        DeclareLaunchArgument('L5y', default_value='0.0685'),

        OpaqueFunction(function=lambda ctx:
            _start_flow(ctx) if action.perform(ctx) == 'start' else _respawn_flow(ctx))
    ])