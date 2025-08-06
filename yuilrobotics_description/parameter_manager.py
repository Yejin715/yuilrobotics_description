import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterEvent

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 1) 파라미터 선언
        self.declare_parameter('gain', 1.0)

        # 2) set-parameter 콜백 (검증만 수행)
        self.add_on_set_parameters_callback(self.validate_params)

        # 3) 파라미터 이벤트 구독 (실제 반영)
        self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.on_parameter_event,
            10
        )

        # 내부 변수 초기화
        self.gain = self.get_parameter('gain').value
        self.get_logger().info(f'초기 gain = {self.gain}')

    def validate_params(self, params):
        """ add_on_set_parameters_callback 에 등록하는 검증 콜백 """
        result = SetParametersResult()
        result.successful = True
        for p in params:
            if p.name == 'gain' and p.value < 0.0:
                result.successful = False
                result.reason = 'gain must be >= 0'
                return result
        return result

    def on_parameter_event(self, event):
        """ ParameterEvent 핸들러에서 실제 파라미터 값 반영 """
        for changed in event.changed_parameters:
            if changed.name == 'gain':
                new_gain = changed.value.double_value
                self.gain = new_gain
                self.get_logger().info(f'gain이 {new_gain} 으로 업데이트되었습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
