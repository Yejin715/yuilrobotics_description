1. ROS 2 ros2_control 컨트롤러 정리 (사용 예시 포함)
  A. JointTrajectoryController
    - 타입: joint_trajectory_controller/JointTrajectoryController
    - 기능: trajectory action을 이용한 부드러운 다관절 제어
    - URDF: PositionJointInterface 필요
    - 입력 타입: FollowJointTrajectory 액션

    ✅ controller_manager.yaml
      controller_manager:
        ros__parameters:
          joint_state_broadcaster:
            type: joint_state_broadcaster/JointStateBroadcaster
          joint_trajectory_controller:
            type: joint_trajectory_controller/JointTrajectoryController

      joint_trajectory_controller:
        ros__parameters:
          joints: [Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6]
          command_interfaces: [position]
          state_interfaces: [position, velocity]

    ✅ 실행 예시
      ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
        trajectory_msgs/action/FollowJointTrajectory \
        "{
          goal: {
            trajectory: {
              joint_names: ['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Joint_6'],
              points: [
                { positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 0} },
                { positions: [1.0, 0.5, -0.5, 1.2, 0.0, -1.0], time_from_start: {sec: 2} }
              ]
            }
          }
        }"

  B. JointGroupPositionController
    - 타입: position_controllers/JointGroupPositionController
    - 기능: 여러 관절 위치를 Float64MultiArray로 제어
    - URDF: PositionJointInterface 필요

    ✅ controller_manager.yaml
      controller_manager:
        ros__parameters:
          position_controller:
            type: position_controllers/JointGroupPositionController

      position_controller:
        ros__parameters:
          joints: [Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6]

    ✅ 실행 예시
      ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
        "{ data: [0.5, -0.2, 1.0, 0.0, -0.5, 0.3] }" --once
      
  C. JointGroupVelocityController
    - 타입: velocity_controllers/JointGroupVelocityController
    - 기능: 여러 관절의 속도를 배치 제어
    - URDF: VelocityJointInterface 필요

    ✅ controller_manager.yaml
      controller_manager:
        ros__parameters:
          velocity_controller:
            type: velocity_controllers/JointGroupVelocityController

      velocity_controller:
        ros__parameters:
          joints: [Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6]

    ✅ 실행 예시
      ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
        "{ data: [0.1, -0.1, 0.2, 0.0, -0.05, 0.1] }" --once

  D. JointGroupEffortController
    - 타입: effort_controllers/JointGroupEffortController
    - 기능: 여러 관절의 토크(힘)를 제어
    - URDF: EffortJointInterface 필요

    ✅ controller_manager.yaml
      controller_manager:
        ros__parameters:
          effort_controller:
            type: effort_controllers/JointGroupEffortController

      effort_controller:
        ros__parameters:
          joints: [Joint_1, Joint_2, Joint_3, Joint_4, Joint_5, Joint_6]

    ✅ 실행 예시
      ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
        "{ data: [0.5, -0.3, 0.2, 0.0, -0.1, 0.4] }" --once

  E. ForwardCommandController (단일 관절 제어)
    - 타입: forward_command_controller/ForwardCommandController
    - 기능: 단일 관절 제어 (입력은 Float64MultiArray 사용)
    - 장점: interface 자유롭게 지정 가능 (position, velocity, effort)
    - URDF: 해당 인터페이스 설정 필요

    ✅ controller_manager.yaml
      controller_manager:
        ros__parameters:
          joint3_position_controller:
            type: forward_command_controller/ForwardCommandController

      joint3_position_controller:
        ros__parameters:
          joint: [Joint_3]
          interface_name: position

    ✅ 실행 예시
      ros2 topic pub /joint3_position_controller/commands std_msgs/msg/Float64MultiArray \
        "{ data: [-0.7] }" --once

✅ URDF 인터페이스 예시 (transmission)
  <transmission name="Joint_1_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="Joint_1">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="Joint_1_actr">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
  ※ 필요한 인터페이스는 Position, Velocity, Effort 중 하나로 설정

✅ 컨트롤러 로드 예시 (gazebo.launch.py 내 ExecuteProcess)
  # joint_state_broadcaster (필수)
  load_jsb = ExecuteProcess(
      cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
      output='screen'
  )

  # 예시: JointGroupPositionController 로드
  load_pos = ExecuteProcess(
      cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'position_controller'],
      output='screen'
  )

  # 예시: 단일 포워드 컨트롤러 로드 (Joint_3)
  load_joint3 = ExecuteProcess(
      cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint3_position_controller'],
      output='screen'
  )
  ※ 필요 시 RegisterEventHandler로 순서 지정 가능
  ※ joint_state_broadcaster는 항상 가장 먼저 로드해야 함