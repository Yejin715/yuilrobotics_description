import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, ParameterType
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from gazebo_msgs.srv import SetModelConfiguration

import math

class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')

        # 파라미터 선언
        self.declare_parameter(
            'joint_names',
            # 기본값은 실제 조인트 이름으로 넣어두면 안전 (아래 참고)
            ['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Joint_6'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'joint_offsets',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            'publish_topic',
            '/joint_states_calib',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            'reseed_on_offset_change',
            True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        )
        self.declare_parameter(
            'jump_threshold_deg',
            7.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        # 파라미터 로드 (이제 .value 로 안전하게)
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.offsets     = list(self.get_parameter('joint_offsets').value)
        self.pub_topic   = self.get_parameter('publish_topic').value
        self.reseed      = bool(self.get_parameter('reseed_on_offset_change').value)
        self.jump_threshold_deg = float(self.get_parameter('jump_threshold_deg').value)
        self.name_to_idx = {n: i for i, n in enumerate(self.joint_names)}
        self._last_js = None

        # 통신
        qos = QoSProfile(depth=50)
        self.pub = self.create_publisher(JointState, self.pub_topic, qos)
        self.sub = self.create_subscription(JointState, '/joint_states', self._on_js, qos)
        self.smc_cli = self.create_client(SetModelConfiguration, '/set_model_configuration')

        # 파라미터 set 콜백(검증+반영)
        self.add_on_set_parameters_callback(self._on_set)

        self.get_logger().info(f'joint_names={self.joint_names}')
        self.get_logger().info(f'offsets(init)={self.offsets}')
        self.get_logger().info(f'publishing corrected states to {self.pub_topic}')

    def _on_js(self, msg: JointState):
        self._last_js = msg
        corr = JointState()
        corr.header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        corr.name = list(msg.name)
        corr.position = list(msg.position) if msg.position else []
        corr.velocity = list(msg.velocity) if msg.velocity else []
        corr.effort = list(msg.effort) if msg.effort else []

        if corr.position:
            for i, jn in enumerate(corr.name):
                if jn in self.name_to_idx and i < len(corr.position):
                    di = self.offsets[self.name_to_idx[jn]]
                    corr.position[i] = corr.position[i] + di  # 상태에 +δ

        self.pub.publish(corr)

    def _on_set(self, params):
        new_offsets = None
        for p in params:
            if p.name == 'joint_offsets':
                # 타입 강제 검사
                if p.type_ != Parameter.Type.DOUBLE_ARRAY:
                    return SetParametersResult(
                        successful=False,
                        reason='joint_offsets must be a double[]'
                    )
                new_offsets = list(p.value)  # p.value는 array('d', [...]) → list 변환

        if new_offsets is not None:
            if len(new_offsets) != len(self.joint_names):
                return SetParametersResult(
                    successful=False,
                    reason='joint_offsets length mismatch'
                )

            # (선택) 점프 크기 계산 후 reseed
            max_jump_deg = 0.0
            if self.offsets:
                import math
                max_jump_deg = max(abs((n - o) * 180.0 / math.pi) for n, o in zip(new_offsets, self.offsets))

            self.offsets = new_offsets
            self.get_logger().info(f'offsets updated -> {self.offsets} (max Δ≈{max_jump_deg:.2f} deg)')

            if self.reseed and max_jump_deg > self.jump_threshold_deg:
                self._reseed_pose()

        return SetParametersResult(successful=True)

    def _reseed_pose(self):
        if self._last_js is None:
            self.get_logger().warn('No joint_states yet; skip reseed.')
            return
        if not self.smc_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_model_configuration not available; skip reseed.')
            return

        req = SetModelConfiguration.Request()
        req.model_name = 'yuilrobotics'              # ← Gazebo 모델명에 맞게 수정
        req.urdf_param_name = 'robot_description' # robot_description 사용 시 권장
        req.joint_names = list(self._last_js.name)
        req.joint_positions = list(self._last_js.position)

        self.get_logger().info('Reseeding joints via /set_model_configuration...')
        fut = self.smc_cli.call_async(req)
        fut.add_done_callback(lambda f: self.get_logger().info('Reseed complete.' if f.result() else 'Reseed failed.'))

def main(args=None):
    rclpy.init(args=args)
    node = ParameterManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
