아래는 지금까지 우리가 만든 **라인 주행 제어 패키지(line_drive_controller)** + **더미 PGV 시뮬레이터** 내용을 반영한 완성형 `README.md` 예시야.
PI AMR v2 프로젝트 상위 저장소(`PI_amr_v2_navigation`)용으로 작성되어 있으며, 실제 코드 구조 및 실행 예시를 모두 포함했어.

---

````markdown
# [PI_amr_v2_navigation]

**PI_amr_v2_navigation** is the top-level integrated repository (super-repo) for the **PI AMR v2 autonomous mobile robot project**.  
This repository includes a modular navigation and control stack built on ROS 2 Humble, designed for line-following and precision docking using PGV (R4) sensors.

---

## 🧭 Overview

이 저장소는 **PGV(R4) 라인 센서 데이터를 기반으로 주행 제어를 수행하는 ROS 2 패키지**입니다.  
`line_drive_controller` 패키지는 센서에서 제공하는 PoseStamped(x, y, yaw) 값을 이용해  
홀로노믹(holonomic) 및 비홀로노믹(non-holonomic) 모드 모두에서 주행 제어를 수행합니다.

- 센서가 라인 중앙에 위치하도록 제어함으로써 라인 추종 및 정렬이 가능  
- 센서 위치 기준 제어를 기본으로 하며, 별도의 오프셋 보정 없이도 안정적 동작  
- YAML 파라미터 파일 기반 설정 및 launch 통합 지원  
- Dummy PGV 시뮬레이터 포함 → 실제 하드웨어 없이 폐루프 테스트 가능

본 연구는 **PITIN EV “PI AMR V2 자율주행 로봇 프로젝트”**의 일부로 수행되었습니다.

---

## ⚙️ Dependencies

- **ROS 2 Humble** (>= 2022.05)
- **Python 3.10**
- **C++17 toolchain**
- ROS 2 packages:
  - `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`

---

## 🧩 Packages included

| Package | Description |
|----------|--------------|
| **line_drive_controller** | PGV PoseStamped 입력을 기반으로 라인 주행 제어를 수행 (홀로/비홀로 선택 가능). |
| **dummy_pgv_sim** | `/cmd_vel`을 적분하여 가상의 PGV PoseStamped 데이터를 퍼블리시하는 더미 시뮬레이터. |
| **one_shot_goal** | 테스트용 단일 주행 목표 명령 노드 (relative / absolute / align service call). |

---

## 🛠️ Installation

```bash
# 워크스페이스 생성 및 clone
mkdir -p ~/amr_ws/src && cd ~/amr_ws/src
git clone --recurse-submodules https://github.com/pitin-ev/PI_amr_v2_navigation.git

# (만약 --recurse-submodules 없이 clone 했다면)
git submodule update --init --recursive

# 의존성 설치
cd ~/amr_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install
source install/setup.bash
````

---

## 🚀 Usage

### ▶ 기본 실행 (YAML 파라미터 기반)

```bash
ros2 launch line_drive_controller line_drive.launch.py
```

### ▶ 통합 시뮬레이션 (Dummy PGV + Controller + Goal)

```bash
ros2 launch line_drive_controller test_line_drive_sim.launch.py
```

### ▶ 주요 Launch Arguments

| Name               | Default                         | Description                                     |
| ------------------ | ------------------------------- | ----------------------------------------------- |
| `holonomic`        | `true`                          | 제어 모드 선택 (true=holonomic / false=non-holonomic) |
| `params_file`      | `config/line_drive.params.yaml` | 컨트롤러 파라미터 파일                                    |
| `sim_params`       | `config/sim.params.yaml`        | 시뮬레이터 파라미터 파일                                   |
| `use_relative`     | `true`                          | 목표 타입 선택 (true=상대 거리, false=절대 좌표)              |
| `relative_goal`    | `1.5`                           | [m] 상대 이동 거리                                    |
| `absolute_goal`    | `5.0`                           | [m] 절대 목표 x좌표                                   |
| `goal_delay_sec`   | `1.0`                           | [s] 목표 발행 전 대기 시간                               |
| `call_align_first` | `true`                          | 시작 시 yaw 정렬 서비스 먼저 호출 여부                        |

---

## 📦 Parameter Highlights

모든 파라미터는 `config/line_drive.params.yaml`에서 설정합니다.

| Parameter                                 | Unit         | Description                                    |
| ----------------------------------------- | ------------ | ---------------------------------------------- |
| `holonomic`                               | bool         | 제어 모드 선택 (`true`: vx,vy만 사용, `false`: v,ω만 사용) |
| `yaw_align_threshold_deg`                 | deg          | 초기 yaw 정렬 완료 허용 오차                             |
| `tolerance_xy`                            | m            | 목표 도달 허용 거리                                    |
| `control_rate`                            | Hz           | 제어 루프 주기                                       |
| `max_lin_vel`, `max_ang_vel`              | m/s, rad/s   | 선형/각속도 상한                                      |
| `accel_lin`, `accel_ang`                  | m/s², rad/s² | 속도 변화 제한(부드러운 가속/감속)                           |
| `slow_start_duration`, `slow_start_speed` | s, m/s       | 정렬 후 미세 전진(방향성 확인용)                            |
| `sensor_*_offset`                         | m, deg       | 센서 오프셋(기본 0: 센서 중심 기준 제어)                      |

> 💡 **기본 설계 원칙**
> 센서(R4)가 라인 중앙에 위치하도록 제어하는 것이 목표이므로
> 일반적으로 `sensor_*_offset = 0` 그대로 사용하면 됩니다.
> 오프셋을 지정하면 “로봇 중심”이 라인 중앙에 오도록 제어 기준이 바뀝니다.

---

## 🎯 Control Interface

### Topic Commands

```bash
# (상대) +1.2 m 전진
ros2 topic pub /line_drive/relative_x_goal std_msgs/Float64 '{data: 1.2}'

# (절대) x = 5.0 m 위치까지 이동
ros2 topic pub /line_drive/absolute_x_goal std_msgs/Float64 '{data: 5.0}'
```

### Service Commands

```bash
# Yaw 정렬만 다시 실행
ros2 service call /line_drive/align_to_line std_srvs/srv/Trigger {}

# 미세 +x 전진 (slow start 속도/시간으로 단발 수행)
ros2 service call /line_drive/nudge_forward std_srvs/srv/Trigger {}
```

---

## 🧪 Example Simulation Flow

1. **Dummy PGV 시뮬레이터 실행** → `/cmd_vel` 명령을 받아 PoseStamped를 퍼블리시
2. **Line Drive Controller** → PoseStamped를 받아 제어(정렬 → 전진)
3. **One-shot Goal Node** → 1 초 후 `/line_drive/relative_x_goal` 발행

```bash
ros2 launch line_drive_controller test_line_drive_sim.launch.py \
  holonomic:=false use_relative:=true relative_goal:=2.0
```

실행 후 콘솔에서 다음 순서를 볼 수 있습니다:

* `[align] yaw aligned; entering slow-start`
* `[slow-start] complete; entering RUNNING`
* `[goal] reached`

---

## 📚 Directory Structure

```
PI_amr_v2_navigation/
├── line_drive_controller/
│   ├── line_drive_node.py          # Core controller node
│   ├── sim_dummy_pgv.py            # Dummy PGV simulator
│   ├── one_shot_goal.py            # Single-goal publisher
│   ├── launch/
│   │   ├── line_drive.launch.py
│   │   └── test_line_drive_sim.launch.py
│   └── config/
│       ├── line_drive.params.yaml
│       └── sim.params.yaml
└── ...
```

---

## 🧠 Design Notes

* **센서 기준 제어 (Default)**
  센서가 라인 중앙에 오도록 제어 → PGV 데이터를 직접 사용하므로 반응 빠름, 설정 단순.
  오프셋 0으로 두면 로봇은 “센서 중심이 라인 위”가 되는 위치에서 멈춤.

* **로봇 중심 기준 제어 (Optional)**
  만약 “로봇 중심(base_link)”을 라인 중앙에 정렬해야 한다면,
  센서의 실제 장착 위치를 `sensor_x_offset`, `sensor_y_offset`에 입력해 보정.

* **Holonomic vs Non-holonomic**
  Holonomic → 측면 이동 가능 플랫폼(예: Mecanum, Omni).
  Non-holonomic → 차동·조향형(예: Double Steering Drive).
  동일한 제어 구조로 모두 대응.

---

## 📞 Contact

**Maintainer:** Jaerak Son  (손재락)
📧 **[jr@pitin-ev.com](mailto:jr@pitin-ev.com)**

> Please leave any questions or issues as [GitHub Issues](https://github.com/pitin-ev/PI_amr_v2_navigation/issues)!

```