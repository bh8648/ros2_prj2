# A4 Cobot2 — 음성 명령 기반 작업공간 정리 협동로봇

> 두산 협동로봇(M0609) + RealSense(eye-in-hand) 카메라 + 딥러닝 비전 + LLM으로,
> **자연어 음성 명령을 받아 흩어진 물체를 인식하고 올바른 구역으로 집어 정리하는** ROS2 시스템.

사용자가 `"작업공간 확인해줘"`라고 말하면 로봇이 3자세로 작업대를 스캔해 물체를 인식하고 정상/오배치를 판정해 음성으로 안내하며, `"정리 시작해줘"`라고 말하면 오배치 물체를 집어 올바른 구역에 내려놓은 뒤 다시 확인하고 최종 결과를 자연어로 보고한다.

---

## 1. 프로젝트 개요

이 프로젝트는 사람의 음성 한마디로 작업공간을 점검·정리하는 협동로봇 시스템이다. 전체 흐름은 아래 두 명령으로 요약된다.

- **"작업공간 확인해줘"** → 3자세 스캔 → 물체 인식·3D 위치 계산 → 구역 판정 → 음성 안내
- **"정리 시작해줘"** → 오배치 물체 pick-and-place → 재검증 → VLM 최종 보고

물체 인식은 YOLO-seg + RT-DETR + SAM2.1 앙상블을 쓰고, 파지 좌표는 point cloud에서 물체 윗면 중심을, 파지 각도는 물체 긴 축(PCA)을 계산해 정한다. 안전 정지, PyQt HMI, SQLite 이력 + rosbag 자동 녹화까지 포함한다.

---

## 2. 시스템 구성

### 하드웨어

| 구성 | 내용 |
| --- | --- |
| 로봇 | Doosan M0609 협동로봇 (bringup: `dsr01` / `m0609`) |
| 그리퍼 | OnRobot 그리퍼 (Modbus 제어, `robot_arm/onrobot.py`) |
| 카메라 | Intel RealSense (그리퍼에 장착된 eye-in-hand 구성) |
| 마이크/스피커 | 음성 명령 입력(STT) 및 안내(TTS)용 |

### 소프트웨어 환경

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10

---

## 3. 노드 구성 (11개)

| 노드 (실행 이름) | 패키지 모듈 | 역할 |
| --- | --- | --- |
| `command_input_node` | `voice` | 웨이크워드("hello rokey") → Whisper STT → GPT-4o 명령 분류 → `/task_command` 발행 |
| `task_manager_node` | `task_manager` | 전체 흐름 제어(스캔/판정/정리 요청, 재검증, 상태·안내 발행) |
| `object_detection_node` | `object_detection` | YOLO-seg 앙상블 인식, 3자세 스캔, camera→base 변환, 파지 좌표·각도 계산 |
| `workspace_judge_node` | `workspace` | 구역(zone) 기반 정상/오배치 판정 |
| `robot_arm_node` | `robot_arm` | 3자세 스캔 이동 주도, 정리 action, 실제 pick-and-place |
| `safety_node` | `task_manager.safety` | stop/clear 명령을 `/emergency_stop`·`/safety_state`로 이원화 |
| `status_notifier_node` | `notification` | 상태·안내 메시지를 TTS 음성으로 출력 |
| `vlm_report_node` | `notification` | 재검증 이미지 + 판정 JSON으로 GPT-4o 최종 보고문 생성 |
| `directed_move_vlm_node` | `notification` | "이 물건을 저 구역으로" 같은 지정 이동 명령을 GPT-4o로 해석 |
| `hmi_interface_node` | `hmi` | PyQt HMI. 스캔 결과 물체 현황(PLACED/MISPLACED) 표시, WAKE UP 버튼 |
| `db_node` | `database` | 작업 이력 SQLite 로깅 + 작업 단위 `ros2 bag record` 자동 녹화 |

---

## 4. 동작 흐름

```
음성("작업공간 확인해줘")
  → command_input_node (STT + LLM 분류) → /task_command: check_workspace
  → task_manager_node → /start_workspace_scan
  → robot_arm_node : 3자세(중앙/좌/우)로 이동하며 각 자세의 base←camera 변환 발행
  → object_detection_node : 앙상블 감지 → base 좌표/각도 계산 → 3자세 point cloud 병합
  → /scanned_objects_base → task_manager_node → /judge_workspace
  → workspace_judge_node : 물체가 올바른 구역에 있는지 판정(정상/오배치)
  → /user_notice 로 음성 안내

음성("정리 시작해줘")
  → /task_command: start_organize
  → task_manager_node → /organize_objects (action) → robot_arm_node
  → 오배치 물체를 각도에 맞춰 집어 해당 구역에 내려놓음
  → 같은 파이프라인으로 재검증(recheck) 1회
  → vlm_report_node 가 최종 보고문 생성 → 음성 안내
```

---

## 5. 주요 기술 특징

- **LLM 의도 분류**: STT 원문을 GPT-4o로 고정 명령(`check_workspace`/`start_organize`/`stop`/`shutdown`)으로 정규화. 규격 밖 응답은 화이트리스트로 다시 걸러 `unknown` 처리.
- **윗면 중심 파지**: mask 안 모든 픽셀을 depth와 결합해 point cloud를 만들고, base Z축 상위 슬라이스에서 윗면 중심을 파지 좌표로 사용 (open3d).
- **3자세 스캔 통합**: 로봇을 중앙/좌/우로 옮겨 각 자세의 `base←camera` 4×4 변환으로 좌표를 통합하고, 세 자세의 point cloud를 병합해 물체 형상을 보완.
- **파지 각도(PCA)**: 물체 긴 축을 공분산 고유분해로 구해 그리퍼 손목 회전에 반영, 짧은 축으로 벌림폭 조절.
- **오검출 필터**: 1초간 다중 프레임을 IoU로 묶어 반복 감지된 것만 채택(frame_support).
- **인식 앙상블**: YOLO-seg(후보·mask) + RT-DETR(검증·보완) + SAM2.1(mask 정밀화).
- **구역 판정 + 그리드 배치**: 4개 구역 AABB로 정상/오배치 판정, 구역을 격자로 나눠 겹치지 않는 빈 슬롯에 배치.
- **힘 제어 안착**: 내려놓기 단계에서 순응 제어 + 외력 감지로 접촉을 느끼며 하강.
- **이중 정지**: soft 플래그 + 컨트롤러 `move_stop`(QSTOP) 직접 호출로 진행 중 모션 즉시 절단.
- **graceful degradation**: VLM/TTS/rosbag 등 일부가 없어도 규칙 기반으로 격하해 계속 동작.

---

## 6. 설치 (git clone 부터)

### 6-1. 사전 요구사항

ROS2 Humble이 설치된 Ubuntu 22.04 환경을 전제한다. 아래 apt 패키지를 먼저 설치한다.

```bash
# ROS 이미지 변환 / 메시지
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
# RealSense ROS2 드라이버
sudo apt install ros-humble-realsense2-camera
# 오디오(마이크/재생)
sudo apt install portaudio19-dev python3-pyaudio libportaudio2 ffmpeg
# HMI
sudo apt install python3-pyqt5
```

> Doosan 로봇 ROS2 패키지(`dsr_msgs2`, DSR bringup 등)는 두산 공식 저장소 안내에 따라 별도로 설치한다.

### 6-2. 워크스페이스 clone

이 저장소의 루트가 곧 워크스페이스의 `src` 디렉토리다. 아래처럼 `src` 안으로 clone한다.

```bash
mkdir -p ~/ws_cobot2_pjt/src
cd ~/ws_cobot2_pjt/src
git clone git@github.com:bh8648/ros2_prj2.git .
```

clone 후 `src` 안에는 두 패키지가 들어 있다.

- `a4_cobot2` — 메인 애플리케이션 패키지 (노드 전체)
- `od_msg` — 커스텀 메시지/서비스/액션 패키지 (`ScanWorkspace`, `JudgeWorkspace`, `GenerateReport`, `OrganizeObjects` 등)

### 6-3. Python 의존성

```bash
pip install -r ~/ws_cobot2_pjt/src/a4_cobot2/requirements_detailed.txt
```

주요 항목: `openai`, `langchain`, `langchain-openai`, `python-dotenv`, `openwakeword`, `edge-tts`, `sounddevice`, `PyAudio`, `ultralytics`, `opencv-python`, `open3d==0.19.0`, `torch`, `pymodbus<3`.
(ROS2 패키지 `rclpy`, `std_msgs` 등은 pip이 아니라 apt/rosdep으로 설치한다.)

### 6-4. SAM2.1 설치

앙상블의 mask 정밀화에 SAM2.1을 쓴다. 공식 저장소를 clone해 설치하면 `import sam2`가 가능해진다.

```bash
git clone https://github.com/facebookresearch/sam2.git
pip install -e ./sam2
```

### 6-5. 별도로 준비해야 하는 파일

아래 파일들은 저장소에 포함되지 않으므로(모델·비밀키) 직접 `a4_cobot2/resource/`에 넣어야 한다.

| 파일 | 설명 |
| --- | --- |
| `.env` | `OPENAI_API_KEY=...` 한 줄. STT/명령분류/VLM에 사용 (**없으면 빌드가 실패**한다, 6-6 참고) |
| `yolo_seg_best_v4.pt` | YOLO segmentation 모델 |
| `rtdetr_best.pt` | RT-DETR 모델 |
| `sam2_1_finetuned.pt` | SAM2.1 파인튜닝 가중치 |

`.env` 예시:

```bash
echo "OPENAI_API_KEY=your_api_key_here" > ~/ws_cobot2_pjt/src/a4_cobot2/resource/.env
```

> 저장소에 이미 포함되어 있는 리소스: `T_gripper2camera.npy`(카메라 캘리브레이션), `hello_rokey_8332_32.tflite`(웨이크워드), `sam2.1_hiera_b+.yaml`, `class_name_tool.json`(클래스 이름표).

### 6-6. 빌드

`a4_cobot2/setup.py`는 `resource/.env`를 설치 대상으로 지정하므로, **`.env`를 먼저 만든 뒤** 빌드해야 한다(6-5).

```bash
cd ~/ws_cobot2_pjt
colcon build --symlink-install
source install/setup.bash
```

`od_msg`가 먼저 빌드되어야 `a4_cobot2`가 커스텀 인터페이스를 찾는다. `colcon`이 의존성 순서를 자동으로 처리한다.

---

## 7. 실행

### 7-1. 전제 (먼저 실행)

```bash
# RealSense 카메라 (aligned depth 필수)
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# Doosan 로봇 bringup (dsr01 / m0609) — 두산 공식 안내에 따라 실행
```

`align_depth.enable:=true` 옵션이 있어야 RGB와 depth가 정렬된 aligned depth topic을 쓸 수 있다.

### 7-2. 전체 실행 (권장)

```bash
cd ~/ws_cobot2_pjt
source install/setup.bash
ros2 launch a4_cobot2 a4_cobot2.launch.py
```

launch 파일이 11개 노드를 한 번에 띄운다(`vlm_report_node`, `directed_move_vlm_node`, `db_node`는 파라미터와 함께 실행).

### 7-3. 개별 노드 실행 (디버깅용)

```bash
ros2 run a4_cobot2 object_detection_node
ros2 run a4_cobot2 workspace_judge_node
ros2 run a4_cobot2 robot_arm_node
ros2 run a4_cobot2 task_manager_node
ros2 run a4_cobot2 safety_node
ros2 run a4_cobot2 status_notifier_node
ros2 run a4_cobot2 command_input_node
ros2 run a4_cobot2 hmi_interface_node
ros2 run a4_cobot2 db_node
ros2 run a4_cobot2 vlm_report_node
ros2 run a4_cobot2 directed_move_vlm_node
```

### 7-4. 인식 화면·상태 확인

```bash
# YOLO 인식 결과 프리뷰
ros2 run rqt_image_view rqt_image_view /yolo_detection_image
# 사용자 안내 / 작업 상태
ros2 topic echo /user_notice
ros2 topic echo /task_status
```

---

## 8. 사용법

### 8-1. 음성 명령

1. 웨이크워드 **"hello rokey"** 로 로봇을 깨운다. (HMI의 **WAKE UP** 버튼으로도 대체 가능)
2. 이어서 명령을 말한다.

| 사용자 음성(예시) | 내부 명령 | 동작 |
| --- | --- | --- |
| "작업공간 확인해줘", "상태 확인해줘" | `check_workspace` | 3자세 스캔 → 판정 → 안내 |
| "정리 시작해줘", "잘못된 물건 치워줘" | `start_organize` | 오배치 물체 정리 → 재검증 → 보고 |
| "멈춰", "정지", "그만" | `stop` | 현재 동작 정지 |

### 8-2. 음성 없이 수동 테스트 (topic publish)

```bash
ros2 topic pub --once /task_command std_msgs/msg/String "{data: 'check_workspace'}"
ros2 topic pub --once /task_command std_msgs/msg/String "{data: 'start_organize'}"
ros2 topic pub --once /task_command std_msgs/msg/String "{data: 'stop'}"
```

---

## 9. 구역(zone) / 클래스 매핑

작업공간을 base 좌표계 4개 구역으로 나눈다. 물체가 매핑된 구역 안에 있으면 정상, 아니면 오배치로 판정하고 정리 시 해당 구역으로 옮긴다.

| 구역 | 클래스 |
| --- | --- |
| red | hammer, screwdriver |
| blue | bolt, tape |
| green | green_apple, pineapple |
| yellow | pocari, gatorade |

구역 좌표/클래스 매핑은 `a4_cobot2/workspace/workspace_judge_utils.py`(`DEFAULT_ZONES`, `CLASS_TO_ZONE`)에서 조정한다.

---

## 10. 디렉토리 구조

```
src/                                  # = 이 저장소 (colcon 워크스페이스의 src)
├── a4_cobot2/                        # 메인 패키지
│   ├── voice/                        # 웨이크워드 / STT / TTS / 명령 분류
│   ├── task_manager/                 # 중앙 상태머신 + safety
│   ├── object_detection/            # YOLO/RT-DETR/SAM2 앙상블, point cloud grasp
│   ├── workspace/                    # 구역 판정 + 그리드 배치
│   ├── robot_arm/                    # 로봇 모션, 그리퍼, 스캔 이동
│   ├── notification/                 # TTS 안내, VLM 보고, 지정 이동
│   ├── hmi/                          # PyQt HMI
│   ├── database/                     # SQLite 로깅 + rosbag 녹화
│   ├── launch/a4_cobot2.launch.py    # 전체 실행 launch
│   ├── resource/                     # 모델·캘리브레이션·.env (일부는 직접 준비)
│   ├── requirements_detailed.txt     # Python 의존성
│   └── setup.py
└── od_msg/                           # 커스텀 msg / srv / action
```

주요 튜닝 지점: `robot_arm/robot_motion.py`(`SCAN_POSES_DEG`, `GRASP_ORIENTATION`, 접근/하강 오프셋), `workspace/workspace_judge_utils.py`(구역 좌표/매핑), `voice/tts.py`(TTS 속도·피치).
