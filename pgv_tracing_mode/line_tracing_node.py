import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger


def ang_norm(a):
    """wrap to [-pi, pi]"""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a


class Phase(Enum):
    IDLE = 0
    ALIGN_YAW = 1
    SLOW_START = 2
    RUNNING = 3
    DONE = 4
    ALIGN_Y_ONLY = 5  # NEW: holonomic 전용 y축 정렬 단계


class LineDriveNode(Node):
    def __init__(self):
        super().__init__('line_drive')

        # ---------------- Parameters ----------------
        self.declare_parameter('pose_topic', '/amr1/bcd_pose')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.declare_parameter('holonomic', True)  # True: vx,vy only (w=0 after align). False: v,w only
        self.declare_parameter('yaw_align_threshold_deg', 2.0)
        self.declare_parameter('tolerance_yaw_deg', 2.0)
        self.declare_parameter('tolerance_xy', 0.01)  # [m]
        self.declare_parameter('pose_timeout_sec', 0.2)  # 200ms 동안 pose 안 들어오면 stop

        self.declare_parameter('control_rate', 50.0)  # Hz

        self.declare_parameter('max_lin_vel', 0.6)
        self.declare_parameter('max_ang_vel', 1.2)
        self.declare_parameter('accel_lin', 0.8)
        self.declare_parameter('accel_ang', 1.5)

        self.declare_parameter('slow_start_duration', 1.0)  # [s] after yaw align
        self.declare_parameter('slow_start_speed', 0.05)    # [m/s], +x micro feed

        # Manual teleop-like line-tracing (hold-to-run)
        self.declare_parameter('teleop_speed', 0.15)        # [m/s] go_forward/backward 기본 속도
        self.declare_parameter('manual_timeout', 0.1)       # [s] TRUE 미수신 시 자동 OFF

        # Sensor offset (sensor mounted at robot side)
        self.declare_parameter('sensor_yaw_offset_deg', 0.0)
        self.declare_parameter('sensor_x_offset', 0.0)
        self.declare_parameter('sensor_y_offset', 0.0)

        # Simple P gains
        self.declare_parameter('kp_x', 1.0)
        self.declare_parameter('kp_y', 1.0)
        self.declare_parameter('kp_yaw', 2.0)

        # Fetch
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.holonomic = bool(self.get_parameter('holonomic').value)


        self.yaw_align_threshold = math.radians(float(self.get_parameter('yaw_align_threshold_deg').value))
        self.tol_yaw = math.radians(float(self.get_parameter('tolerance_yaw_deg').value))
        self.tol_xy = float(self.get_parameter('tolerance_xy').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout_sec').value)

        self.rate_hz = float(self.get_parameter('control_rate').value)

        self.max_v = float(self.get_parameter('max_lin_vel').value)
        self.max_w = float(self.get_parameter('max_ang_vel').value)
        self.acc_v = float(self.get_parameter('accel_lin').value)
        self.acc_w = float(self.get_parameter('accel_ang').value)

        self.slow_start_duration = float(self.get_parameter('slow_start_duration').value)
        self.slow_start_speed = float(self.get_parameter('slow_start_speed').value)

        self.teleop_speed = float(self.get_parameter('teleop_speed').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)

        self.sensor_yaw_offset = math.radians(float(self.get_parameter('sensor_yaw_offset_deg').value))
        self.sensor_x_offset = float(self.get_parameter('sensor_x_offset').value)
        self.sensor_y_offset = float(self.get_parameter('sensor_y_offset').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)

        # QoS: R4 노드가 best-effort일 수도 있어 파라미터 변경을 고려할 수 있게 최소값으로 설정
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

        # ---------------- I/O ----------------
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, qos)
        self.rel_goal_sub = self.create_subscription(Float64, '/line_drive/relative_x_goal', self._on_rel_goal, 10)
        self.abs_goal_sub = self.create_subscription(Float64, '/line_drive/absolute_x_goal', self._on_abs_goal, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.srv_align = self.create_service(Trigger, '/line_drive/align_to_line', self._srv_align)
        self.srv_nudge = self.create_service(Trigger, '/line_drive/nudge_forward', self._srv_nudge)
        self.srv_align_y_only = self.create_service(Trigger, '/line_drive/align_y_only', self._srv_align_y_only)

        # NEW: manual forward/backward subscribers (Bool)
        self.fwd_sub = self.create_subscription(Bool, '/line_drive/go_forward', self._on_go_forward, 10)
        self.back_sub = self.create_subscription(Bool, '/line_drive/go_backward', self._on_go_backward, 10)

        # ---------------- State ----------------
        self.phase = Phase.IDLE
        self.last_twist = Twist()
        self.last_pose_time = self.get_clock().now()

        # PGV pose (line frame): assume pose of sensor in line frame
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # radians, angle around +z, yaw=0 is aligned with line +x

        # goals
        self.goal_active = False
        self.goal_abs_x = None  # absolute x target (in line frame)
        self.goal_rel_dx = None # relative x target (delta)

        self.start_x_for_rel = None
        self.slow_start_until = None

        # Manual flags (timestamps of last TRUE)
        self.last_fwd_true = None
        self.last_back_true = None

        # y-align 요청 플래그
        self.y_align_requested = False

        # Timers
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(f"[line_drive] holonomic={self.holonomic}, cmd_topic={self.cmd_topic}, pose_topic={self.pose_topic}")

    # ---------------- Subscribers ----------------
    def _on_pose(self, msg: PoseStamped):
        # Extract x,y,yaw from PoseStamped (sensor frame in line frame)
        # Convert builtin_interfaces.msg.Time -> rclpy Time for arithmetic compatibility
        self.last_pose_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self.x = msg.pose.position.x + self.sensor_x_offset
        self.y = msg.pose.position.y + self.sensor_y_offset

        # yaw from quaternion (assuming z,w only valid; still do robust)
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        # yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.yaw = ang_norm(yaw + self.sensor_yaw_offset)

    def _on_rel_goal(self, msg: Float64):
        self.goal_rel_dx = float(msg.data)
        self.goal_abs_x = None
        self.start_x_for_rel = self.x
        self.goal_active = True
        self._enter_align_phase()
        self.get_logger().info(f"[goal] relative dx={self.goal_rel_dx:.3f} m")
        self.get_logger().info(f"[goal] start_x_for_rel={self.start_x_for_rel:.3f} m")
        self.get_logger().info(f"[goal] target x*={self.start_x_for_rel + self.goal_rel_dx:.3f} m")

    def _on_abs_goal(self, msg: Float64):
        self.goal_abs_x = float(msg.data)
        self.goal_rel_dx = None
        self.start_x_for_rel = None
        self.goal_active = True
        self._enter_align_phase()
        self.get_logger().info(f"[goal] absolute x*={self.goal_abs_x:.3f} m")

    # manual Bool callbacks
    def _on_go_forward(self, msg: Bool):
        self.get_logger().info(f"Received go_forward: {msg.data}")
        if msg.data:
            self.last_fwd_true = self.get_clock().now()

    def _on_go_backward(self, msg: Bool):
        self.get_logger().info(f"Received go_backward: {msg.data}")
        if msg.data:
            self.last_back_true = self.get_clock().now()

    # ---------------- Services ----------------
    def _srv_align(self, req, resp):
        self._enter_align_phase()
        resp.success = True
        resp.message = "Align-to-line initiated"
        return resp

    def _srv_nudge(self, req, resp):
        # one-shot micro +x
        self.phase = Phase.SLOW_START
        self.slow_start_until = self.get_clock().now() + Duration(seconds=self.slow_start_duration)
        resp.success = True
        resp.message = f"Nudge forward for {self.slow_start_duration:.2f}s at {self.slow_start_speed:.3f} m/s"
        return resp
    
    # holonomic 전용 y축 정렬 서비스
    def _srv_align_y_only(self, req, resp):
        if not self.holonomic:
            resp.success = False
            resp.message = "align_y_only is supported only in holonomic mode."
            return resp

        # 목표 주행 중지하고, y-align 요청 플래그 설정
        self.goal_active = False
        self.y_align_requested = True

        # yaw 먼저 정렬해야 하면 ALIGN_YAW부터, 아니면 바로 ALIGN_Y_ONLY
        yaw_err = abs(ang_norm(0.0 - self.yaw))
        if yaw_err > self.yaw_align_threshold:
            self._enter_align_phase()
            msg = "Yaw out of threshold; ALIGN_YAW first, then ALIGN_Y_ONLY."
        else:
            self._enter_align_y_only()
            msg = "Yaw within threshold; entering ALIGN_Y_ONLY."

        resp.success = True
        resp.message = msg
        self.get_logger().info(f"[align_y_only] request accepted: {msg}")
        return resp

    # ---------------- Phase helpers ----------------
    def _enter_align_phase(self):
        self.phase = Phase.ALIGN_YAW
        self.slow_start_until = None

    def _enter_align_y_only(self):
        self.phase = Phase.ALIGN_Y_ONLY
        self.get_logger().info("[align_y_only] started")

    def _enter_slow_start(self):
        self.phase = Phase.SLOW_START
        self.slow_start_until = self.get_clock().now() + Duration(seconds=self.slow_start_duration)

    def _enter_running(self):
        self.phase = Phase.RUNNING

    def _enter_done(self):
        self.phase = Phase.DONE
        self.goal_active = False
        self._publish_twist(0.0, 0.0, 0.0)

    # ---------------- Manual helpers ----------------
    def _manual_active(self):
        """Return +1 if forward active, -1 if backward active, 0 otherwise.
           TRUE가 0.1s 내에 들어온 쪽만 활성. 둘 다 있으면 0(안전 정지)."""
        now = self.get_clock().now()
        tol = Duration(seconds=self.manual_timeout)

        fwd_on = self.last_fwd_true is not None and (now - self.last_fwd_true) <= tol
        back_on = self.last_back_true is not None and (now - self.last_back_true) <= tol

        if fwd_on and back_on:
            return 0
        if fwd_on:
            return +1
        if back_on:
            return -1
        return 0

    def _do_manual(self, direction: int):
        """direction: +1 forward, -1 backward"""
        # 목표 주행은 일시 중지
        # (원한다면 여기서 self.goal_active를 False로 만들어도 됨)
        speed = self.teleop_speed * (1.0 if direction > 0 else -1.0)

        y_err = - self.y
        yaw_err = ang_norm(0.0 - self.yaw)

        if self.holonomic:
            # vx = ±teleop_speed, vy는 y 보정, ω는 필요 시 미세 보정
            vx_cmd = speed
            vy_cmd = self.kp_y * y_err

            vx_cmd = self._slew_limit(vx_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            vy_cmd = self._slew_limit(vy_cmd, self.last_twist.linear.y, self.acc_v / self.rate_hz)

            spd = math.hypot(vx_cmd, vy_cmd)
            if spd > self.max_v:
                scale = self.max_v / max(spd, 1e-6)
                vx_cmd *= scale
                vy_cmd *= scale

            w_cmd = 0.0
            if abs(yaw_err) > self.tol_yaw:
                w_cmd = max(-0.2, min(0.2, self.kp_yaw * yaw_err))

            self._publish_twist(vx_cmd, vy_cmd, w_cmd)
        else:
            # Non-holo: v = ±teleop_speed, ω로 y,yaw 수렴
            v_cmd = speed
            w_cmd = self.kp_y * y_err + self.kp_yaw * yaw_err

            v_cmd = self._slew_limit(v_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)

            v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

            self._publish_twist(v_cmd, 0.0, w_cmd)

    # ---------------- Control loop ----------------
    def _control_loop(self):
        now = self.get_clock().now()
        if (now - self.last_pose_time) > Duration(seconds=self.pose_timeout):
            if (self.last_twist.linear.x != 0.0) or (self.last_twist.linear.y != 0.0) or (self.last_twist.angular.z != 0.0):
                self._publish_twist(0.0, 0.0, 0.0)
                self.get_logger().warn("[safety] pose timeout detected -> STOP")
            return
        # 0) 수동(hold-to-run) 우선 처리
        manual_dir = self._manual_active()
        if manual_dir != 0:
            self._do_manual(manual_dir)
            return

        # 1) 수동 아님 + "아무 목표/페이즈도 없음" => 정지하고 리턴
        if (not self.goal_active) and (self.phase not in (Phase.ALIGN_YAW, Phase.SLOW_START, Phase.RUNNING, Phase.ALIGN_Y_ONLY)):
            if (self.last_twist.linear.x != 0.0) or (self.last_twist.linear.y != 0.0) or (self.last_twist.angular.z != 0.0):
                self._publish_twist(0.0, 0.0, 0.0)
            return

        # 2) 나머지는 기존 상태기계대로
        if self.phase == Phase.ALIGN_YAW:
            self._do_align_yaw()
            return

        if self.phase == Phase.ALIGN_Y_ONLY:
            self._do_align_y_only()
            return

        if self.phase == Phase.SLOW_START:
            self._do_slow_start()
            return

        if self.phase == Phase.RUNNING:
            self._do_running()
            return

        if self.phase == Phase.DONE:
            # keep stopped
            self._publish_twist(0.0, 0.0, 0.0)
            return

    # ---------------- Behaviors ----------------
    def _do_align_yaw(self):
        yaw_err = ang_norm(0.0 - self.yaw)
        # P controller on yaw
        w_cmd = self.kp_yaw * yaw_err
        w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
        # keep linear zero during alignment
        self._publish_twist(0.0, 0.0, w_cmd)

        if abs(yaw_err) <= self.yaw_align_threshold:
            # Done: enter slow start micro +x
            self._enter_slow_start()
            self.get_logger().info("[align] yaw aligned; entering slow-start")

    def _do_align_y_only(self):
        # holonomic만 허용 (안전 guard)
        if not self.holonomic:
            self.get_logger().warn("[align_y_only] called in non-holonomic; stopping.")
            self._enter_done()
            return

        # yaw 드리프트가 커지면 미세 보정 허용
        yaw_err = ang_norm(0.0 - self.yaw)
        w_cmd = 0.0
        if abs(yaw_err) > self.tol_yaw:
            w_cmd = max(-0.2, min(0.2, self.kp_yaw * yaw_err))

        # vy만 사용해서 y를 0으로 수렴 (vx=0)
        y_err = -self.y
        vx_cmd = 0.0
        vy_cmd = self.kp_y * y_err

        vy_cmd = self._slew_limit(vy_cmd, self.last_twist.linear.y, self.acc_v / self.rate_hz)
        # 속도 상한 (전체 속도 대신 vy만 클램프)
        vy_cmd = max(-self.max_v, min(self.max_v, vy_cmd))

        self._publish_twist(vx_cmd, vy_cmd, w_cmd)

        # 완료 판정
        if abs(self.y) <= self.tol_xy:
            self.y_align_requested = False
            self._enter_done()
            self.get_logger().info("[align_y_only] done; y≈0 reached")

    def _do_slow_start(self):
        now = self.get_clock().now()
        if self.slow_start_until is None:
            self.slow_start_until = now + Duration(seconds=self.slow_start_duration)

        if self.holonomic:
            self._publish_twist(self.slow_start_speed, 0.0, 0.0)
        else:
            # non-holo: use v only, w=0
            self._publish_twist(self.slow_start_speed, 0.0, 0.0)

        if now >= self.slow_start_until:
            self._enter_running()
            self.get_logger().info("[slow-start] complete; entering RUNNING")

    def _do_running(self):
        # Compute target x*
        if self.goal_abs_x is not None:
            x_target = self.goal_abs_x
        elif self.goal_rel_dx is not None and self.start_x_for_rel is not None:
            x_target = self.start_x_for_rel + self.goal_rel_dx
        else:
            # no valid goal → stop
            self._enter_done()
            return

        x_err = x_target - self.x
        y_err = - self.y           # we want y -> 0
        yaw_err = ang_norm(0.0 - self.yaw)  # we want yaw -> 0

        # Goal check
        if abs(x_err) <= self.tol_xy and abs(self.y) <= self.tol_xy:
            self._enter_done()
            self.get_logger().info("[goal] reached")
            self.get_logger().info(f"[goal] final pose: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.2f} deg")
            return

        if self.holonomic:
            # ω=0 유지, vx, vy만 사용
            vx_cmd = self.kp_x * x_err
            vy_cmd = self.kp_y * y_err

            # 가속/속도 제한
            vx_cmd = self._slew_limit(vx_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            vy_cmd = self._slew_limit(vy_cmd, self.last_twist.linear.y, self.acc_v / self.rate_hz)

            spd = math.hypot(vx_cmd, vy_cmd)
            if spd > self.max_v:
                scale = self.max_v / max(spd, 1e-6)
                vx_cmd *= scale
                vy_cmd *= scale

            # 정렬 이후이므로 yaw 오차가 크면 보정 한 번 더(옵션)
            w_cmd = 0.0
            if abs(yaw_err) > self.tol_yaw:
                # 아주 미세 보정만 허용 (클램프)
                w_cmd = max(-0.2, min(0.2, self.kp_yaw * yaw_err))

            self._publish_twist(vx_cmd, vy_cmd, w_cmd)

        else:
            # Non-holonomic: v, ω만 사용
            # x_err로 전진속도, (y_err, yaw_err)로 각속도
            v_cmd = self.kp_x * x_err
            w_cmd = self.kp_y * y_err + self.kp_yaw * yaw_err

            v_cmd = self._slew_limit(v_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)

            v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

            self._publish_twist(v_cmd, 0.0, w_cmd)

    # ---------------- Utils ----------------
    def _publish_twist(self, vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)
        self.last_twist = t

    @staticmethod
    def _slew_limit(target, current, step):
        """limit change per cycle (simple slew rate limiter)"""
        delta = target - current
        if delta > step:
            return current + step
        if delta < -step:
            return current - step
        return target


def main(args=None):
    rclpy.init(args=args)
    node = LineDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_twist(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
