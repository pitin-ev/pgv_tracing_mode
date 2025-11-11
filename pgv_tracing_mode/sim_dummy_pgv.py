import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from builtin_interfaces.msg import Time
import random
import time

def ang_norm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class DummyPgvSim(Node):
    """
    /cmd_vel을 받아 간단 2D 운동학으로 상태(x,y,yaw)를 적분하고,
    PGV가 보는 PoseStamped를 /amr1/bcd_pose 등으로 퍼블리시하는 더미 센서 시뮬레이터.

    - holonomic_sim = True: (vx, vy, wz) 명령을 '로봇 프레임' 기준으로 해석
        world xdot = vx*cos(yaw) - vy*sin(yaw)
        world ydot = vx*sin(yaw) + vy*cos(yaw)
        yawdot = wz
    - holonomic_sim = False: (v, 0, wz)만 사용 (차동/비홀로노믹 근사)
        world xdot = v*cos(yaw)
        world ydot = v*sin(yaw)
        yawdot = wz

    발행되는 PoseStamped는 센서 오프셋/노이즈를 적용.
    """
    def __init__(self):
        super().__init__('dummy_pgv_sim')

        # ---- Parameters ----
        self.declare_parameter('pose_topic', '/amr1/bcd_pose')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rate', 50.0)  # Hz

        self.declare_parameter('holonomic_sim', True)   # 시뮬레이터 운동학 선택
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw_deg', 15.0)    # 일부러 기울여 시작해 정렬 동작 확인

        # 센서(카메라/PGV) 오프셋 (로봇 기준, m / rad)
        self.declare_parameter('sensor_x_offset', 0.0)
        self.declare_parameter('sensor_y_offset', 0.0)
        self.declare_parameter('sensor_yaw_offset_deg', 0.0)

        # 센서 노이즈
        self.declare_parameter('pos_noise_std', 0.0)     # [m]
        self.declare_parameter('yaw_noise_std_deg', 0.0) # [deg]

        # 속도 제한(선택, 기본은 컨트롤러가 처리하므로 넉넉히)
        self.declare_parameter('max_v', 1.0)
        self.declare_parameter('max_w', 2.0)

        # Fetch
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate').value)

        self.holo = bool(self.get_parameter('holonomic_sim').value)
        self.x = float(self.get_parameter('init_x').value)
        self.y = float(self.get_parameter('init_y').value)
        self.yaw = math.radians(float(self.get_parameter('init_yaw_deg').value))

        self.sensor_x_offset = float(self.get_parameter('sensor_x_offset').value)
        self.sensor_y_offset = float(self.get_parameter('sensor_y_offset').value)
        self.sensor_yaw_offset = math.radians(float(self.get_parameter('sensor_yaw_offset_deg').value))

        self.pos_noise_std = float(self.get_parameter('pos_noise_std').value)
        self.yaw_noise_std = math.radians(float(self.get_parameter('yaw_noise_std_deg').value))

        self.max_v = float(self.get_parameter('max_v').value)
        self.max_w = float(self.get_parameter('max_w').value)

        # ---- I/O ----
        qos_cmd = 10
        qos_pose = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=5)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, qos_cmd)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, qos_pose)

        # State: last cmd
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self.dt = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(self.dt, self._step)

        self.get_logger().info(f"[dummy_pgv_sim] holonomic_sim={self.holo}, publishing Pose to {self.pose_topic}, listening {self.cmd_topic}")

    # ---- Callbacks ----
    def _on_cmd(self, msg: Twist):
        # Saturate a bit
        vx = max(-self.max_v, min(self.max_v, msg.linear.x))
        vy = max(-self.max_v, min(self.max_v, msg.linear.y))
        wz = max(-self.max_w, min(self.max_w, msg.angular.z))
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_wz = wz

    def _step(self):
        # Integrate kinematics in world (line) frame
        if self.holo:
            # robot-frame -> world
            c = math.cos(self.yaw)
            s = math.sin(self.yaw)
            xdot = self.cmd_vx * c - self.cmd_vy * s
            ydot = self.cmd_vx * s + self.cmd_vy * c
            w    = self.cmd_wz
        else:
            v = self.cmd_vx
            w = self.cmd_wz
            xdot = v * math.cos(self.yaw)
            ydot = v * math.sin(self.yaw)

        self.x += xdot * self.dt
        self.y += ydot * self.dt
        self.yaw = ang_norm(self.yaw + w * self.dt)

        # Compose sensor reading = robot pose (+ sensor offset) with noise
        sx = self.x + (self.sensor_x_offset * math.cos(self.yaw) - self.sensor_y_offset * math.sin(self.yaw))
        sy = self.y + (self.sensor_x_offset * math.sin(self.yaw) + self.sensor_y_offset * math.cos(self.yaw))
        syaw = ang_norm(self.yaw + self.sensor_yaw_offset)

        if self.pos_noise_std > 0.0:
            sx += random.gauss(0.0, self.pos_noise_std)
            sy += random.gauss(0.0, self.pos_noise_std)
        if self.yaw_noise_std > 0.0:
            syaw = ang_norm(syaw + random.gauss(0.0, self.yaw_noise_std))

        # Publish PoseStamped (z=0, q from yaw)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(sx)
        ps.pose.position.y = float(sy)
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = math.sin(syaw/2.0)
        ps.pose.orientation.w = math.cos(syaw/2.0)

        self.pose_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = DummyPgvSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
