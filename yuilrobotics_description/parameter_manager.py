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

        # 파라미터 로드
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.offsets     = list(self.get_parameter('joint_offsets').value)
        self.pub_topic   = self.get_parameter('publish_topic').value
        self.reseed      = bool(self.get_parameter('reseed_on_offset_change').value)
        self.jump_threshold_deg = float(self.get_parameter('jump_threshold_deg').value)
        
        # 길이 보정(배열 길이와 조인트 수 일치)
        if len(self.offsets) != len(self.joint_names):
            self.offsets = (self.offsets + [0.0] * len(self.joint_names))[:len(self.joint_names)]

        self.name_to_idx = {n: i for i, n in enumerate(self.joint_names)}
        self._last_js = None

        # 조인트별 개별 파라미터 선언
        for i, name in enumerate(self.joint_names):
            self.declare_parameter(
                f'offset.{name}',
                float(self.offsets[i]),
                ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
            )

        # 동기화 제어 플래그/타이머 (배열 ↔ 개별 동기화)
        self._suppress_callback = False
        self._pending_sync = False
        self._sync_timer = self.create_timer(0.2, self._sync_params)  # 5Hz

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

    # 배열(joint_offsets)와 개별(offset.<name>)을 모두 지원
    def _on_set(self, params):
        if self._suppress_callback:
            return SetParametersResult(successful=True)

        old = list(self.offsets)
        changed = False
        new_offsets_from_array = None
        
        for p in params:
            # 1) joint_offsets 전체 배열 갱신
            if p.name == 'joint_offsets':
                if p.type_ != Parameter.Type.DOUBLE_ARRAY:
                    return SetParametersResult(successful=False, reason='joint_offsets must be double[]')
                vals = list(p.value)
                if len(vals) != len(self.joint_names):
                    return SetParametersResult(successful=False, reason='joint_offsets length mismatch')
                new_offsets_from_array = vals
                changed = True

            # 2) 개별 갱신: offset.<JointName>
            elif p.name.startswith('offset.'):
                jname = p.name.split('offset.', 1)[1]
                if jname in self.name_to_idx:
                    if p.type_ != Parameter.Type.DOUBLE:
                        return SetParametersResult(successful=False, reason=f'{p.name} must be double')
                    idx = self.name_to_idx[jname]
                    self.offsets[idx] = float(p.value)
                    changed = True
                else:
                    return SetParametersResult(successful=False, reason=f'Unknown joint name: {jname}')

        if new_offsets_from_array is not None:
            self.offsets = new_offsets_from_array

        if changed:
            max_jump_deg = 0.0
            if old and len(old) == len(self.offsets):
                max_jump_deg = max(abs((n - o) * 180.0 / math.pi) for n, o in zip(self.offsets, old))

            self.get_logger().info(f'offsets updated -> {self.offsets} (max Δ≈{max_jump_deg:.2f} deg)')

            if self.reseed and max_jump_deg > self.jump_threshold_deg:
                self._reseed_pose()

            self._pending_sync = True

        return SetParametersResult(successful=True)

    # 주기적으로 배열/개별 파라미터를 서로 동기화(무한 루프 방지 플래그 포함)
    def _sync_params(self):
        if not self._pending_sync:
            return
        self._pending_sync = False

        self._suppress_callback = True
        try:
            params = []
            # 배열 업데이트
            params.append(Parameter('joint_offsets', Parameter.Type.DOUBLE_ARRAY, list(self.offsets)))
            # 개별 업데이트
            for i, name in enumerate(self.joint_names):
                params.append(Parameter(f'offset.{name}', Parameter.Type.DOUBLE, float(self.offsets[i])))
            self.set_parameters(params)
        finally:
            self._suppress_callback = False

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
