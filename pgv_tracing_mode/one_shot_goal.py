import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

class OneShotGoal(Node):
    """
    노드 시작 후 일정 시간 대기 -> (옵션) 정렬 서비스 호출 -> 목표를 한 번 발행하고 종료.
    - relative_goal: 상대 dx (Float64) / absolute_goal: 절대 x* 중 하나만 사용
    """
    def __init__(self):
        super().__init__('one_shot_goal')

        self.declare_parameter('use_relative', True)
        self.declare_parameter('relative_goal', 1.0)
        self.declare_parameter('absolute_goal', 5.0)
        self.declare_parameter('goal_delay_sec', 1.0)
        self.declare_parameter('call_align_first', True)

        self.use_relative = bool(self.get_parameter('use_relative').value)
        self.rel = float(self.get_parameter('relative_goal').value)
        self.abs = float(self.get_parameter('absolute_goal').value)
        self.delay = float(self.get_parameter('goal_delay_sec').value)
        self.call_align = bool(self.get_parameter('call_align_first').value)

        self.pub_rel = self.create_publisher(Float64, '/line_drive/relative_x_goal', 10)
        self.pub_abs = self.create_publisher(Float64, '/line_drive/absolute_x_goal', 10)

        if self.call_align:
            self.align_cli = self.create_client(Trigger, '/line_drive/align_to_line')
        else:
            self.align_cli = None

        self.timer = self.create_timer(0.1, self._tick)
        self.deadline = self.get_clock().now() + Duration(seconds=self.delay)
        self.state = 0  # 0: wait, 1: align, 2: publish, 3: done

        self.get_logger().info(f"[one_shot_goal] waiting {self.delay:.2f}s, then call_align={self.call_align}, use_relative={self.use_relative}")

    def _tick(self):
        now = self.get_clock().now()
        if self.state == 0:
            if now >= self.deadline:
                self.state = 1 if self.call_align else 2

        if self.state == 1:
            if self.align_cli is None:
                self.state = 2
            else:
                if not self.align_cli.service_is_ready():
                    self.get_logger().info("[one_shot_goal] waiting align service...")
                    return
                req = Trigger.Request()
                fut = self.align_cli.call_async(req)
                fut.add_done_callback(self._after_align)
                self.state = 99  # wait future

        if self.state == 2:
            if self.use_relative:
                self.pub_rel.publish(Float64(data=self.rel))
                self.get_logger().info(f"[one_shot_goal] published relative goal dx={self.rel:.3f}")
            else:
                self.pub_abs.publish(Float64(data=self.abs))
                self.get_logger().info(f"[one_shot_goal] published absolute goal x*={self.abs:.3f}")
            self.state = 3

        if self.state == 3:
            self.get_logger().info("[one_shot_goal] done; shutting down")
            rclpy.shutdown()

    def _after_align(self, fut):
        try:
            resp = fut.result()
            self.get_logger().info(f"[one_shot_goal] align_to_line: success={resp.success} msg={resp.message}")
        except Exception as e:
            self.get_logger().warn(f"[one_shot_goal] align_to_line failed: {e}")
        self.state = 2


def main(args=None):
    rclpy.init(args=args)
    node = OneShotGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
