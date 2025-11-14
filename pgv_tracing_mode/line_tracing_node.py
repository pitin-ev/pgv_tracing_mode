"""라인 트레이싱 / 정렬 ROS2 노드 (LineDriveNode).

이 노드는 PGV 등에서 발행하는 `geometry_msgs/PoseStamped` 센서 포즈를 이용하여
로봇을 추상적인 "라인 좌표계" 상에서 주행하거나 정렬합니다.

주요 기능:
    * 라인 방향으로의 요(yaw) 정렬 (Phase.ALIGN_YAW)
    * 요 정렬 후 짧은 마이크로 전진(SLOW_START)으로 부드러운 시작
    * 절대 또는 상대 X 목표 주행 + Y=0 유지 (Phase.RUNNING)
    * 홀로노믹 전용 Y축 정렬 서비스 (ALIGN_Y_ONLY) - 전진 없이 횡방향 보정
    * Bool 토픽을 이용한 수동 hold-to-run 방식 전/후진 오버라이드
    * Pose 업데이트 타임아웃 시 안전 정지

상태 머신 (Phase):
    IDLE         : 대기 상태 (정지), 목표나 정렬 요청 없음
    ALIGN_YAW    : 요 오차를 기준 임계값 이하로 줄이기 위한 제자리 회전
    SLOW_START   : 짧은 시간 저속 전진으로 기계적/센서적 워밍업
    RUNNING      : 목표 X로 이동하면서 Y, Yaw 안정화
    DONE         : 목표 또는 정렬 완료 후 정지
    ALIGN_Y_ONLY : 홀로노믹 전용 Y축 정렬 (Yaw 보정 히스테리시스 포함)

서비스:
    /line_drive/align_to_line    (Trigger): 일반 요 정렬 시작 (ALIGN_YAW → SLOW_START → RUNNING)
    /line_drive/nudge_forward    (Trigger): 즉시 SLOW_START 수행 (마이크로 전진)
    /line_drive/align_y_only     (Trigger): 필요 시 요 정렬 후 Y축 정렬 (ALIGN_YAW → ALIGN_Y_ONLY)

토픽:
    pose_topic (PoseStamped)           : 센서 포즈 (라인 프레임)
    /line_drive/relative_x_goal (Float64): 현재 x 기준 상대 이동 목표 (Δx)
    /line_drive/absolute_x_goal (Float64): 절대 x 목표
    /line_drive/go_forward (Bool)        : 수동 전진 hold 신호 (TRUE 펄스 유지)
    /line_drive/go_backward (Bool)       : 수동 후진 hold 신호
    cmd_topic (Twist)                    : 속도 명령 (vx, vy, wz)

수동 hold-to-run 로직:
    TRUE 메시지 수신 시 타임스탬프 저장. `manual_timeout` 초 이내면 활성 유지.
    전진/후진 둘 다 활성 시 안전을 위해 0 (정지) 처리.
    수동 입력은 각 제어 주기에서 자동 목표보다 우선.

주요 파라미터 (모두 __init__에서 declare):
    pose_topic (str)                : 입력 포즈 토픽
    cmd_topic (str)                 : 출력 Twist 토픽
    holonomic (bool)                : True면 vx, vy 사용; False면 v, w 사용
    yaw_align_threshold_deg (float) : ALIGN_YAW 완료 기준 (deg)
    tolerance_yaw_deg (float)       : 안정 yaw 판정 허용 오차 (deg)
    yaw_hysteresis_factor (float)   : ALIGN_Y_ONLY에서 yaw 보정 활성 임계 다중값
    tolerance_xy (float, m)         : x,y 목표/정렬 완료 허용 오차
    pose_timeout_sec (float)        : pose 미수신 안전 정지 시간
    control_rate (Hz)               : 제어 루프 주파수
    max_lin_vel, max_ang_vel        : 최대 선속도 / 각속도
    accel_lin, accel_ang            : 제어 주기당 가속(슬루) 제한
    slow_start_duration (s)         : SLOW_START 지속 시간
    slow_start_speed (m/s)          : SLOW_START 속도
    teleop_speed (m/s)              : 수동 전/후진 기본 속도
    manual_timeout (s)              : hold-to-run TRUE 유지 시간
    sensor_* offsets                : 센서 포즈 보정 오프셋 및 yaw 오프셋
    kp_x, kp_y, kp_yaw              : 단순 P 게인

Yaw 히스테리시스 (ALIGN_Y_ONLY):
    |yaw_err| > tol_yaw * yaw_hysteresis_factor 일 때 보정 활성
    |yaw_err| <= tol_yaw 일 때 비활성
    매우 작은 w_cmd (<0.02 rad/s) 데드밴드로 지터 억제

완료 조건:
    RUNNING 목표: |x_err| <= tol_xy AND |y| <= tol_xy
    Y 정렬: |y| <= tol_xy AND yaw 안정(보정 비활성 & |yaw_err| <= tol_yaw)

사용 예시:
    기본 실행:
        ros2 launch pgv_tracing_mode line_drive.launch.py
    파라미터 오버라이드:
        ros2 run pgv_tracing_mode line_drive --ros-args -p holonomic:=true -p yaw_hysteresis_factor:=2.0
    상대 목표 설정:
        ros2 topic pub /line_drive/relative_x_goal std_msgs/Float64 "data: 0.5"
    Y 정렬 서비스 호출:
        ros2 service call /line_drive/align_y_only std_srvs/srv/Trigger {}
    수동 전진 펄스:
        ros2 topic pub /line_drive/go_forward std_msgs/Bool "data: true"

향후 개선 아이디어 (미구현):
    * Y / Yaw 안정성 시간 기반 카운트 후 완료 처리
    * Yaw 에러 EMA 필터로 각속도 부드럽게
    * 남은 거리/속도 기반 적응형 게인

본 파일은 가독성을 위해 많은 주석이 포함되어 있으며,
상태 전이 로직을 변경할 때는 문서화된 흐름을 유지하는 것이 좋습니다.
"""

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
    """각도를 [-pi, pi] 범위로 정규화.

    모듈로 연산을 사용하여 어떤 실수 입력에도 안정적으로 동작합니다.
    """
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a


def clamp(val, lo, hi):
    """값을 [lo, hi] 범위로 제한 (포함)."""
    return max(lo, min(hi, val))


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

    # ---------------- 파라미터 선언 ----------------
    # (상단 모듈 주석에 상세 설명 있음)
        self.declare_parameter('pose_topic', '/amr1/bcd_pose')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.declare_parameter('holonomic', True)  # True: vx,vy only (w=0 after align). False: v,w only
        self.declare_parameter('yaw_align_threshold_deg', 2.0)
        self.declare_parameter('tolerance_yaw_deg', 2.0)
        # Hysteresis factor for yaw correction in ALIGN_Y_ONLY (start > tol*factor, stop < tol)
        self.declare_parameter('yaw_hysteresis_factor', 1.5)
        self.declare_parameter('tolerance_xy', 0.01)  # [m]
        self.declare_parameter('pose_timeout_sec', 0.2)  # 200ms 동안 pose 안 들어오면 stop
        # Timeout 후 제어 재개 유예시간 (초) - pose 다시 들어와도 이 시간 동안은 정지 유지
        self.declare_parameter('post_timeout_grace_sec', 1.0)

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
        # 최대 미세 yaw 속도 (ALIGN_Y_ONLY에서 사용, soft clamp 적용)
        self.declare_parameter('align_y_only_max_ang_vel', 0.2)
        # 래핑 경계 진동 억제 구간 (deg): |yaw|가 (pi - band) 이상이면 부호 고정
        self.declare_parameter('yaw_wrap_band_deg', 12.0)
    # 역방향(±180°)도 정렬된 것으로 허용할지 여부 (라인 방향성 없을 때 true)
        self.declare_parameter('allow_reverse_heading', True)

    # ---------------- 파라미터 값 조회 ----------------
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.holonomic = bool(self.get_parameter('holonomic').value)


    # deg → rad 변환
        self.yaw_align_threshold = math.radians(float(self.get_parameter('yaw_align_threshold_deg').value))
        self.tol_yaw = math.radians(float(self.get_parameter('tolerance_yaw_deg').value))
        self.yaw_hysteresis_factor = float(self.get_parameter('yaw_hysteresis_factor').value)
        self.tol_xy = float(self.get_parameter('tolerance_xy').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout_sec').value)
        self.post_timeout_grace = float(self.get_parameter('post_timeout_grace_sec').value)

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
        self.align_y_only_max_w = float(self.get_parameter('align_y_only_max_ang_vel').value)
        self.yaw_wrap_band = math.radians(float(self.get_parameter('yaw_wrap_band_deg').value))
        self.allow_reverse_heading = bool(self.get_parameter('allow_reverse_heading').value)

    # QoS: 고속/간헐 손실 허용 센서 대비 BEST_EFFORT + 작은 depth
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

    # ---------------- I/O (토픽/서비스) ----------------
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

    # ---------------- 상태 변수 ----------------
        self.phase = Phase.IDLE
        self.last_twist = Twist()
        self.last_pose_time = self.get_clock().now()
        # Pose 미수신 timeout 상태 및 재개 grace 상태 관리
        self.timeout_active = False
        self.resume_block_until = None  # rclpy Time; None이면 grace 없음
        self._last_grace_log_time = None

    # PGV 센서 포즈 (오프셋 적용 후)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # radians, angle around +z, yaw=0 is aligned with line +x
        # 래핑 경계에서 부호 고정용
        self.yaw_sign_fixed = 0  # -1 / +1 / 0

    # 목표 관리: 절대 x 또는 상대 Δx (start_x_for_rel 기준)
        self.goal_active = False
        self.goal_abs_x = None  # absolute x target (in line frame)
        self.goal_rel_dx = None # relative x target (delta)

        self.start_x_for_rel = None
        self.slow_start_until = None

    # 수동 hold-to-run 타임스탬프 (None이면 비활성)
        self.last_fwd_true = None
        self.last_back_true = None

    # Y 정렬 서비스 요청 플래그 (완료 시 해제)
        self.y_align_requested = False
        # Internal flag for yaw correction hysteresis in ALIGN_Y_ONLY
        self.yaw_correction_active = False

    # 타이머: 주기적 제어 루프 실행
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(f"[line_drive] holonomic={self.holonomic}, cmd_topic={self.cmd_topic}, pose_topic={self.pose_topic}")

    # ---------------- Subscribers ----------------
    def _on_pose(self, msg: PoseStamped):
        # PoseStamped에서 x,y,yaw 추출
        # builtin_interfaces.msg.Time → rclpy Time 변환 (시간 연산 가능하도록)
        self.last_pose_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self.x = msg.pose.position.x + self.sensor_x_offset
        self.y = msg.pose.position.y + self.sensor_y_offset

        # yaw from quaternion (assuming z,w only valid; still do robust)
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        # yaw from quaternion
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 센서 yaw 오프셋 적용 후 정규화된 yaw 저장
        yaw_meas = ang_norm(yaw + self.sensor_yaw_offset)

        # 경계 근처에서 sign flip 방지: |yaw|가 pi - wrap_band 이상이면 이전 부호 유지
        abs_yaw = abs(yaw_meas)
        if abs(abs_yaw - math.pi) < self.yaw_wrap_band:
            # 경계 영역: 이전에 고정된 부호가 없다면 현재 측정 부호를 저장
            if self.yaw_sign_fixed == 0:
                self.yaw_sign_fixed = 1 if yaw_meas >= 0.0 else -1
            # 고정된 부호 적용 (값 자체는 측정 그대로 두되 yaw_err 계산에서 사용할 예정)
            self.yaw = yaw_meas
        else:
            # 경계 영역 벗어나면 부호 고정 해제 및 최신 값 반영
            self.yaw_sign_fixed = 0
            self.yaw = yaw_meas

    def _on_rel_goal(self, msg: Float64):
        """상대 x 목표 처리: 현재 x를 기준으로 Δx 설정 후 목표 활성화."""
        self.goal_rel_dx = float(msg.data)
        self.goal_abs_x = None
        self.start_x_for_rel = self.x
        self.goal_active = True
        self._enter_align_phase()
        self.get_logger().info(f"[goal] relative dx={self.goal_rel_dx:.3f} m")
        self.get_logger().info(f"[goal] start_x_for_rel={self.start_x_for_rel:.3f} m")
        self.get_logger().info(f"[goal] target x*={self.start_x_for_rel + self.goal_rel_dx:.3f} m")

    def _on_abs_goal(self, msg: Float64):
        """절대 x 목표 처리: 절대 x 설정 후 목표 활성화."""
        self.goal_abs_x = float(msg.data)
        self.goal_rel_dx = None
        self.start_x_for_rel = None
        self.goal_active = True
        self._enter_align_phase()
        self.get_logger().info(f"[goal] absolute x*={self.goal_abs_x:.3f} m")

    # manual Bool callbacks
    def _on_go_forward(self, msg: Bool):
        """전진 TRUE 펄스 수신 시 타임스탬프 갱신."""
        self.get_logger().info(f"Received go_forward: {msg.data}")
        if msg.data:
            self.last_fwd_true = self.get_clock().now()

    def _on_go_backward(self, msg: Bool):
        """후진 TRUE 펄스 수신 시 타임스탬프 갱신."""
        self.get_logger().info(f"Received go_backward: {msg.data}")
        if msg.data:
            self.last_back_true = self.get_clock().now()

    # ---------------- Services ----------------
    def _srv_align(self, req, resp):
        """서비스: 일반 yaw 정렬 시작."""
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
        """서비스: Y축 정렬 (필요 시 먼저 yaw 정렬)."""
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
        """ALIGN_YAW 단계로 전이."""
        self.phase = Phase.ALIGN_YAW
        self.slow_start_until = None
        # 정렬 시작 시 경계 근처라면 현재 고정 부호 채택, 아니면 초기화
        if abs(abs(self.yaw) - math.pi) < self.yaw_wrap_band and self.yaw_sign_fixed == 0:
            self.yaw_sign_fixed = 1 if self.yaw >= 0.0 else -1

    def _enter_align_y_only(self):
        """ALIGN_Y_ONLY 단계로 전이; yaw 히스테리시스 상태 초기화."""
        self.phase = Phase.ALIGN_Y_ONLY
        # initialize yaw correction state based on current yaw error
        yaw_err = abs(ang_norm(0.0 - self.yaw))
        self.yaw_correction_active = yaw_err > self.tol_yaw
        self.get_logger().info("[align_y_only] started")

    def _enter_slow_start(self):
        """SLOW_START (마이크로 전진) 단계로 전이."""
        self.phase = Phase.SLOW_START
        self.slow_start_until = self.get_clock().now() + Duration(seconds=self.slow_start_duration)

    def _enter_running(self):
        """RUNNING (목표 추종) 단계로 전이."""
        self.phase = Phase.RUNNING

    def _enter_done(self):
        """DONE 단계: 0 속도 명령 발행 및 목표 비활성화."""
        self.phase = Phase.DONE
        self.goal_active = False
        self._publish_twist(0.0, 0.0, 0.0)

    # ---------------- Manual helpers ----------------
    def _manual_active(self):
        """전진 활성 시 +1, 후진 활성 시 -1, 둘 다/없으면 0 반환.
        TRUE 펄스가 manual_timeout 이내에 들어온 방향만 유지, 둘 다면 안전 정지(0)."""
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
        """수동 제어 실행: direction +1 전진 / -1 후진."""
        # 목표 주행은 일시 중지
        # (원한다면 여기서 self.goal_active를 False로 만들어도 됨)
        speed = self.teleop_speed * (1.0 if direction > 0 else -1.0)

        y_err = -self.y
        # 역방향 허용 시 0 또는 ±pi 중 더 가까운 목표까지의 (yaw - target) 에러
        yaw_err = self._yaw_err(consider_reverse=True)
        reverse_ok = self.allow_reverse_heading and abs(abs(self.yaw) - math.pi) < self.tol_yaw
        if reverse_ok:
            # 역방향 허용 + ±π 근처면 yaw 보정 생략
            yaw_err = 0.0

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
            # if (not reverse_ok) and abs(yaw_err) > self.tol_yaw:
            #     # negative feedback: w = -kp * (yaw - target)
            #     w_cmd_raw = -self.kp_yaw * yaw_err
            #     # 작은 yaw 에러에서는 미세한 값 유지 (과도한 포화 방지)
            #     self.get_logger().info(f"[manual holo] yaw_err={math.degrees(yaw_err):.2f} deg -> w_cmd_raw={w_cmd_raw:.3f} rad/s")
            #     w_cmd = clamp(w_cmd_raw, -0.2, 0.2)

            self._publish_twist(vx_cmd, vy_cmd, w_cmd)
        else:
            # Non-holo: v = ±teleop_speed, ω로 y,yaw 수렴
            v_cmd = speed
            w_cmd = self.kp_y * y_err + (0.0 if reverse_ok else -self.kp_yaw * yaw_err)

            v_cmd = self._slew_limit(v_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)

            v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

            self._publish_twist(v_cmd, 0.0, w_cmd)

    # ---------------- Control loop ----------------
    def _control_loop(self):
        """주기적 제어 루프 (우선순위 순서):
        1) 안전 타임아웃 검사
        2) 수동 hold-to-run 처리
        3) 상태(Phase)별 자동 동작
        """
        now = self.get_clock().now()
        # 1) Pose timeout 감지: 멈추고 timeout_active 설정
        if (now - self.last_pose_time) > Duration(seconds=self.pose_timeout):
            if not self.timeout_active:
                self.timeout_active = True
                self.resume_block_until = None  # 새 timeout 발생 시 이전 grace 폐기
                self.get_logger().warn("[safety] pose timeout detected -> STOP")
            if (self.last_twist.linear.x != 0.0) or (self.last_twist.linear.y != 0.0) or (self.last_twist.angular.z != 0.0):
                self._publish_twist(0.0, 0.0, 0.0)
            return
        # 2) Timeout에서 회복된 첫 주기: grace 설정
        if self.timeout_active:
            self.timeout_active = False
            self.resume_block_until = now + Duration(seconds=self.post_timeout_grace)
            self._last_grace_log_time = now
            self.get_logger().warn(f"[safety] pose recovered; holding still for {self.post_timeout_grace:.2f}s grace")
        # 3) Grace 기간 동안은 제어 차단 (강제 정지 유지)
        if self.resume_block_until is not None and now < self.resume_block_until:
            # 주기적으로 남은 시간 로그 (0.25s 간격)
            if self._last_grace_log_time is None or (now - self._last_grace_log_time) > Duration(seconds=0.25):
                remaining = (self.resume_block_until - now).nanoseconds / 1e9
                self.get_logger().info(f"[safety] grace active: {remaining:.2f}s remaining")
                self._last_grace_log_time = now
            if (self.last_twist.linear.x != 0.0) or (self.last_twist.linear.y != 0.0) or (self.last_twist.angular.z != 0.0):
                self._publish_twist(0.0, 0.0, 0.0)
            return
        # Grace 종료
        if self.resume_block_until is not None and now >= self.resume_block_until:
            self.resume_block_until = None
            self.get_logger().info("[safety] grace ended; resuming control")
        # 0) 수동(hold-to-run) 우선 처리 (grace 기간엔 이미 return 되었음)
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
        """Yaw 정렬: P 제어로 임계값 이하까지 회전.

        y-only 정렬 요청이면 ALIGN_Y_ONLY로, 아니면 SLOW_START로 분기.
        """
        # 순수 +X 기준 yaw 에러만 사용하여 진동 최소화
        forward_yaw_err = ang_norm(self.yaw - 0.0)  # (yaw - target)
        # 역방향 허용 시 ±π 근처를 별도 정렬 완료 조건으로 취급
        reverse_aligned = False
        if self.allow_reverse_heading:
            if abs(abs(self.yaw) - math.pi) < self.yaw_align_threshold:
                reverse_aligned = True

        # 제어용 에러: 역방향 정렬 완료 시 추가 회전 중지 위해 0, 아니면 forward 에러 사용
        ctrl_yaw_err = 0.0 if reverse_aligned else forward_yaw_err
        self.get_logger().info(
            f"[align] yaw={math.degrees(self.yaw):.1f} deg, yaw_err={math.degrees(forward_yaw_err):.2f} deg, rev_allowed={self.allow_reverse_heading}, rev_aligned={reverse_aligned}")
        # P controller on yaw (역방향 정렬이면 0)
        w_cmd = -self.kp_yaw * ctrl_yaw_err  # negative feedback
        w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
        # keep linear zero during alignment
        self._publish_twist(0.0, 0.0, w_cmd)

        aligned_direct = abs(forward_yaw_err) <= self.yaw_align_threshold
        aligned = aligned_direct or reverse_aligned
        if aligned:
            # 완료 시 분기: y-align 요청이 있으면 ALIGN_Y_ONLY로, 아니면 기존 slow-start
            if self.y_align_requested:
                self._enter_align_y_only()
                self.get_logger().info(f"[align] {'reverse ' if reverse_aligned and not aligned_direct else ''}yaw aligned; entering ALIGN_Y_ONLY")
            else:
                self._enter_slow_start()
                self.get_logger().info(f"[align] {'reverse ' if reverse_aligned and not aligned_direct else ''}yaw aligned; entering slow-start")

    def _do_align_y_only(self):
        """홀로노믹 Y축 정렬 (yaw 히스테리시스 포함).

        vy로 y 보정, 히스테리시스 활성 시에만 w_cmd 최소 적용.
        완료: y 허용오차 & yaw 안정.
        """
        # holonomic만 허용 (안전 guard)
        if not self.holonomic:
            self.get_logger().warn("[align_y_only] called in non-holonomic; stopping.")
            self._enter_done()
            return

        # yaw 드리프트가 커지면 미세 보정 허용 (hysteresis 적용)
        yaw_err_raw = self._yaw_err(consider_reverse=True)
        # 역방향 허용 시 ±π 근처면 yaw 보정 비활성 처리
        if self.allow_reverse_heading and abs(abs(self.yaw) - math.pi) < self.tol_yaw:
            yaw_err_raw = 0.0
        yaw_abs = abs(yaw_err_raw)
        # Activate correction only if beyond outer band
        if not self.yaw_correction_active and yaw_abs > (self.tol_yaw * self.yaw_hysteresis_factor):
            self.yaw_correction_active = True
        # Deactivate when within inner tolerance
        if self.yaw_correction_active and yaw_abs <= self.tol_yaw:
            self.yaw_correction_active = False

        w_cmd = 0.0
        # if self.yaw_correction_active:
        #     w_cmd_raw = self.kp_yaw * yaw_err_raw
        #     # small deadband to avoid jitter near zero
        #     if abs(w_cmd_raw) < 0.02:
        #         w_cmd_raw = 0.0
        #     # Soft saturation: 출력은 [-align_y_only_max_w, +align_y_only_max_w]로 부드럽게 제한
        #     # tanh를 이용해 포화 구간에서도 연속적인 변화량 제공 (갑작스런 -0.2 고정 감소)
        #     max_w = self.align_y_only_max_w
        #     w_cmd = max_w * math.tanh(w_cmd_raw / max_w)  # smooth clamp

        # vy만 사용해서 y를 0으로 수렴 (vx=0)
        y_err = -self.y
        vx_cmd = 0.0
        vy_cmd = self.kp_y * y_err

        vy_cmd = self._slew_limit(vy_cmd, self.last_twist.linear.y, self.acc_v / self.rate_hz)
        # 속도 상한 (전체 속도 대신 vy만 클램프)
        vy_cmd = max(-self.max_v, min(self.max_v, vy_cmd))

        self._publish_twist(vx_cmd, vy_cmd, w_cmd)

        # 완료 판정: y 정렬 AND yaw within tolerance (안정 상태)
        if abs(self.y) <= self.tol_xy and (not self.yaw_correction_active) and yaw_abs <= self.tol_yaw:
            self.y_align_requested = False
            self._enter_done()
            self.get_logger().info("[align_y_only] done; y≈0 & yaw stable")

    def _do_slow_start(self):
        """Yaw 정렬 직후 짧은 저속 전진으로 부드러운 시작."""
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
        """RUNNING: x 목표로 이동하며 y, yaw 안정화."""
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
        y_err = -self.y            # we want y -> 0
        yaw_err = self._yaw_err(consider_reverse=True)  # (yaw - target)
        reverse_ok = self.allow_reverse_heading and abs(abs(self.yaw) - math.pi) < self.tol_yaw

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
            # if (abs(yaw_err) > self.tol_yaw) and (not reverse_ok):
            #     # negative feedback: w = -kp * (yaw - target)
            #     w_cmd_raw = -self.kp_yaw * yaw_err
            #     w_cmd = clamp(w_cmd_raw, -0.2, 0.2)

            self._publish_twist(vx_cmd, vy_cmd, w_cmd)

        else:
            # Non-holonomic: v, ω만 사용
            # x_err로 전진속도, (y_err, yaw_err)로 각속도
            v_cmd = self.kp_x * x_err
            w_cmd = self.kp_y * y_err + (0.0 if reverse_ok else -self.kp_yaw * yaw_err)

            v_cmd = self._slew_limit(v_cmd, self.last_twist.linear.x, self.acc_v / self.rate_hz)
            w_cmd = self._slew_limit(w_cmd, self.last_twist.angular.z, self.acc_w / self.rate_hz)

            v_cmd = max(-self.max_v, min(self.max_v, v_cmd))
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

            self._publish_twist(v_cmd, 0.0, w_cmd)

    # ---------------- Utils ----------------
    def _publish_twist(self, vx, vy, wz):
        """Twist 발행 및 마지막 명령 저장 (slew 제한에 사용)."""
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)
        self.last_twist = t

    def _yaw_err(self, consider_reverse: bool = False):
        """(yaw - target) 형태의 heading 오차 반환.

        target 기본값은 0 rad(+X). consider_reverse=True이고 allow_reverse_heading=True이면
        0 또는 ±π 중에서 |yaw - target_candidate|가 더 작은 목표를 선택.

        제어식은 w = -kp * yaw_err 로 사용하여 음의 피드백이 되도록 한다.

        경계(±π) 부근 sign flip 억제를 위해 yaw_sign_fixed 적용.
        """
        # 측정 yaw 안정화
        if self.yaw_sign_fixed != 0 and abs(abs(self.yaw) - math.pi) < self.yaw_wrap_band:
            yaw_meas = self.yaw_sign_fixed * abs(self.yaw)
        else:
            yaw_meas = self.yaw

        # 후보 1: forward (0)
        forward_err = ang_norm(yaw_meas - 0.0)
        if not (consider_reverse and self.allow_reverse_heading):
            return forward_err

        # 후보 2: reverse (±π same sign as yaw)
        target_rev = math.copysign(math.pi, yaw_meas if yaw_meas != 0 else 1.0)
        err_rev = ang_norm(yaw_meas - target_rev)

        return err_rev if abs(err_rev) < abs(forward_err) else forward_err

    @staticmethod
    def _slew_limit(target, current, step):
        """한 주기당 변화량 제한 (간단한 slew rate 제한)."""
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
