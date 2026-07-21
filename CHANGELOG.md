# Changelog

## 2026-07-22 — Stage 7 QGC global mission and Stage 8 rosbag/ULog ground truth

- 다른 컴퓨터에서 같은 환경을 재현하도록 `Dockerfile.0722`, `.dockerignore`, `PORTABLE_SETUP.md`를 추가했다. Docker Hub `reventon13713/project_f:0722`는 `pyulog==0.9.0`, `pymavlink==2.4.49`, SHA-256으로 고정한 Baylands assets를 포함하며 published digest는 `sha256:9fb94dd93bdd1c1cdd2bec78c1178593c85c14a84c62d480d09912d8abf9e45a`다.
- `vio_qgc_mission_test.py`를 추가했다. QGC-compatible `.plan`과 동일한 MAVLink mission을 업로드하고, 고정 yaw와 `DO_CHANGE_SPEED=0.55 m/s`로 기존 VIO 운동 계약을 유지하며 sequence 5에서 실제 Gazebo GPS를 끈다.
- `vio_mission_ground_truth_metrics.py`는 mission event 시간창에서 PX4/VIO WGS84 trajectory를 독립 Gazebo GT에 no-scale yaw/translation 정렬하고 path, jump, gap, RMSE/max, waypoint 오차를 판정한다. 같은 ULog의 mission finished, nav state, GPS/EV flags, EV aid fuse/rejection, global reset/step을 교차 확인한다.
- repeat runner에 stage-7 lifecycle, 저부하 evidence bag, ULog 보존/평가, 결과 merge, QGC/PX4 mission 의존성 검사와 infra-failure nonzero exit를 연결했다. `run_stage7_qgc_mission_ground_truth_test.sh`와 pinned `pymavlink==2.4.49` requirements를 추가했다.
- 최종 Baylands evidence `20260722T005543KST_run01`은 GPS dropout 뒤 3개 waypoint와 GT `9.28 m`를 완료했다. PX4/VIO GT RMSE `0.461/0.418 m`, 지도 max step `0.214 m`, ULog EV fused `177/178`, mission finished로 7·8단계를 통과했다.

## 2026-07-21 — Stage 6 actual Gazebo GPS dropout

- `vio_gazebo_plugins`를 추가했다. `libvio_gps_control.so`는 explicit sentinel 변화에 따라 Gazebo GPS sensor를 직접 on/off하며, `vio_gps_control_command`가 명령을 원자적으로 생성한다. transport CLI 전달 실패를 성공으로 오인하지 않도록 GPS stream stale이 최종 권위다.
- `vio_gps_dropout_test.py`와 `run_stage6_gps_dropout_test.sh`를 추가하고 repeat runner에 stage-6 lifecycle, metrics merge, cleanup, GPS dropout event, EV/no-aid 설정을 연결했다.
- `vio_bridge.py`에 opt-in `px4_timestamp_sample_from_receipt`를 추가했다. 기본 false이며 stage-6 SITL에서만 aligned estimator의 latest propagated pose publication time을 PX4 sample time으로 사용한다.
- Gazebo plugin smoke, Python/bash/diff 검사, Docker build와 Baylands clean `20260721T232119KST/run01` acceptance를 통과했다. 롤백 시 sensor plugin과 evaluator를 함께 제거해야 하며 `EKF2_GPS_CTRL=0`을 실제 GPS dropout 근거로 대체하면 안 된다.

## 2026-07-21 — stage-5 GPS+EV global continuity rosbag/ULog acceptance

- `vio_global_fusion_metrics.py`를 추가했다. alignment 뒤 GPS fix/handover, PX4↔VIO WGS84 RMSE/max, consecutive error, 양쪽 global step, VIO gap을 rosbag에서 판정하고 동일 실행 ULog의 GPS/EV 동시 control flag, 실제 GNSS/EV aid `fused`, rejection, global reset/step을 교차 판정한다.
- `run_clean_preflight_repeats.sh`에 opt-in `GPS_EV_GLOBAL_FUSION_EVALUATION=1`을 추가했다. stage-4+bag mode를 강제하고 launch 전 marker 뒤 생성된 ULog만 `px4.ulg`로 보존하며 offline 결과를 primary metrics/outcome에 병합한다. pyulog가 없으면 비행 전에 fail한다.
- `run_stage5_gps_vio_fusion_test.sh`는 GPS를 유지하고 stage-4 fusion, bag/ULog 보존, 20초 ready hold와 15초 post-metrics record를 재현한다. `requirements-ulog.txt`는 ROS 2 Humble system NumPy와 호환되는 `pyulog==0.9.0`을 고정한다.
- 정적 검사, Docker build, 기존 stage-4 artifact 회귀와 Baylands `20260721T202758KST/run01` live acceptance를 통과했다. GPS dropout/fault injection 동작은 변경하지 않았다.

## 2026-07-21 — aligned NED PX4 external-vision integration

- `vio_bridge.py`에 `odom_frame_contract`(`enu_flu`/`ned_frd`)와 `readiness_topic`을 추가했다. stage-4 `/vio/aligned_odometry`는 PX4 local NED/body-FRD contract를 만족하므로 raw VINS ENU/FLU 변환 없이 `VehicleOdometry`로 전달한다.
- `vio_gps_ev_fusion_test.py`를 추가했다. global alignment와 handover, EV input을 확인한 뒤 native PX4 estimator flags의 GPS/EV position/EV velocity가 동시에 유지될 때만 성공 JSON을 남긴다. alignment 이전과 이후 timeout을 분리했다.
- `run_clean_preflight_repeats.sh`에 `PX4_GPS_EV_FUSION_TEST=1` opt-in을 추가했다. `EKF2_EV_CTRL=7`으로 EV pos/height/velocity aid를 켜고 GPS는 유지하며, relevant ROS topics와 fusion JSON을 rosbag/artifact에 기록한다. fault mode와 동시 실행은 금지했다.
- `test_vio_aligned_bridge_contract.py`는 aligned NED position/velocity와 FRD attitude가 PX4 input에서 축 교환 없이 보존되는 ROS 회귀다. legacy handover regression, Docker build와 Baylands GPS+EV fusion run을 통과했다.
- 롤백 시 `ned_frd` mode를 raw VINS input에 적용하면 안 된다. GPS dropout/mission continuation은 이 변경 범위에 포함되지 않는다.

## 2026-07-21 — `vio_global_position` fail-closed global alignment

- `vio_global_position.py`를 추가해 신선하고 유효한 VIO/GPS/PX4 local sample에서 scale·yaw·translation과 WGS84 origin을 고정하고 `NavSatFix /vio/global_position` 및 NED `/vio/aligned_odometry`를 발행한다. ellipsoid/MSL 차이는 `SensorGps`의 두 altitude로 보존한다.
- alignment excitation/scale/error gate와 정렬 뒤 output gap·timestamp 역행·pose jump·새 readiness generation latch를 구현했다. latch 뒤 자동 재정렬은 없고 node restart가 필요하다.
- `vio_vins.launch.py`에 노드를 통합하되 PX4 external vision 연결은 하지 않았다. 반복 recorder에 GPS/local/global/global-output topic을 추가했다.
- `vio_global_position_metrics.py`, 합성 math/ROS policy test, `run_vio_global_position_test.sh`를 추가했다. Docker build와 Baylands `20260721T084626KST`에서 local fit RMSE/max `0.056/0.111 m`, PX4 global 대비 WGS84 RMSE/max `0.0387/0.0699 m`로 통과했다.
- 롤백 시 `vio_vins.launch.py`의 node와 새 recorder topic/entry point를 함께 제거해야 한다. `/vio/aligned_odometry`를 raw VINS ENU와 혼동하면 안 되며 PX4 EV 연결은 4단계 전까지 비활성이다.

## 2026-07-21 — VIO reset 뒤 stationary initialization window 명시화

- `vio_preflight_test.py`의 reset 뒤 hover warm-up을 하드코딩 `2 s`에서 파라미터 `post_reset_warmup_seconds=3.0`으로 바꿨다.
- 초기 heading 유지와 부드러운 XYZ 병진 cross는 기존대로 유지한다. 변경 목적은 새 VINS epoch가 excitation 전 독립 stationary IMU window를 갖게 하는 것이며, GPS/EV/global conversion에는 영향이 없다.

## 2026-07-21 — Baylands relative-waypoint independent ground-truth acceptance

- `vio_preflight_test.py`에 opt-in scored assessment state와 12개 waypoint start/reached event를 추가했다. 초기화용 cross와 평가용 cross를 분리하며 assessment 중 VINS output loss는 fail-closed한다.
- `vio_ground_truth_metrics.py`를 추가해 raw VINS, Gazebo P3D GT, PX4 local pose, waypoint event를 no-scale yaw/translation 정렬로 평가한다. RMSE `0.30 m`, trajectory/waypoint max `0.50 m` acceptance를 자동 판정한다.
- `run_clean_preflight_repeats.sh`가 독립 GT publisher/message를 확인하고 토픽을 bag에 기록하며, opt-in 평가 JSON·summary를 만들고 assertion failure를 비정상 종료로 전달하도록 확장했다. 전용 `run_baylands_relative_waypoint_test.sh`를 추가했다.
- `vio_bag_metrics.read_topic()`에 호환 기본값을 유지하는 topic-tag 반환 옵션을 추가했고, `setup.py`/`package.xml`에 새 executable과 `rosbag2_py` runtime dependency를 연결했다.
- 정적 검사, 합성 yaw/translation test, ROS 2 build와 Baylands 최종 GT `2/2`를 통과했다. external vision/global conversion은 변경하지 않았다.

## 2026-07-21 — public Baylands outdoor world integration

- Open Robotics `baylands` archive의 URL, 크기, SHA-256을 고정한 설치 스크립트와 PX4 Gazebo Classic용 `vio_baylands.world`를 추가했다.
- upstream 3개 visual mesh와 자연 texture를 모두 유지하고, 무거운 mesh collision은 500 m 평면으로 분리했다. 첫 smoke에서 발견한 `collide_without_contact` 낙하 결함을 제거했다.
- ROS 2 camera/IMU/range와 독립 P3D ground truth를 포함하는 `iris_fpv_cam` assets를 프로젝트에서 설치하도록 했고, demo 기본 world 및 반복 runner의 선택 가능한 `PX4_SITL_WORLD`를 연결했다.
- PX4 v1.14.4에서 sensor/GPS/geodetic origin/ground contact/camera feature smoke를 통과했다. external-vision fusion과 global conversion은 변경하지 않았다.

## 2026-07-17 — visualization load reduction and rejected reset-epoch experiments

- SITL config의 `show_track`을 `0`으로 바꿔 비행 중 diagnostic tracking image 렌더링/publication을 끊었다. 유효 live 9/9 terminal·recovery 0을 확인했다.
- per-frame image/feature fingerprint와 queue trace를 opt-in으로 바꾸는 실험은 recovery 0이 유효 `6/9`로 악화돼 전부 원복했다. 최종 소스는 기존 trace를 유지한다.
- reset 전후 feature에 epoch를 태그하고 stale frame을 버리는 atomic/nonblocking 두 구현을 시험했다. stale drop은 관측됐지만 terminal/recovery/strict 결과가 기존 최고보다 개선되지 않아 관련 struct, atomic flag, enqueue/drop 경로를 전부 제거했다.
- initialization cost-per-block 로그와 solver summary 진단은 유지했다. Ceres 1.14, serial marginalization, trust-radius 제한은 A/B 실패 후 원복했으며 최종 소스는 system Ceres 2와 `NUM_THREADS=4`다.
- 최종 검증 artifact: show-track-off `20260717T164507KST`, trace-off 폐기 `20260717T171548KST`, atomic epoch 폐기 `20260717T175537KST`, nonblocking epoch 폐기 `20260717T182304KST`.

## 2026-07-17 — solver/prior diagnostics and rejected operational mitigations

- position-jump 로그에 Ceres cost/time/step/termination을 유지했다. 고비용 residual group 분해는 marginalization prior 지배를 확인한 뒤 production 오버헤드를 남기지 않도록 제거했다.
- 반복 runner에 동작 기본값을 바꾸지 않는 `EXCITATION_SPEED` 실험 인자와 batch 기록을 추가했다.
- iteration 25, excitation 0.40 m/s, validation dither 0.10/0.04 m, threshold 3.5 px, outlier-before-prior 순서 변경은 모두 replay/live 반증 후 원복했다.
- 최종 동작은 SITL threshold 4 px, 12 iterations, 0.55 m/s excitation, validation/READY dither 없음이다.

## 2026-07-17 — configurable outlier threshold, reset-epoch alignment, solver fault evidence

### `parameters.h`, `estimator.cpp`, SITL VINS config

- 평균 reprojection outlier 삭제 기준을 config `outlier_rejection_threshold_px`로 분리했다. 미설정 기본값은 3 px, 검증된 SITL config는 4 px다.
- position-jump 오류에 Ceres initial/final cost, solve time, successful/unsuccessful step, termination을 추가했다. estimator decision은 바꾸지 않는 진단이다.
- replay-only leading-image skip과 nonlinear feature/marginalization trace를 추가해 recorded reset 없는 동일 입력 failure를 결정적으로 재현했다.
- 검증: 4 px deterministic bag 통과; live 10회 recovery 0 `10/10`, 후속 합산 `17/18`. 후속 failure 때문에 완전 해결을 주장하지 않는다.

### `vio_preflight_test.py`

- reset 성공 시 scale뿐 아니라 alignment pair, yaw/translation fit, vector-error accumulator를 모두 초기화한다. 서로 다른 VINS local epoch 혼합으로 건강한 recovery가 timeout되던 오류를 수정했다.

### PX4 fault regression

- GPS fallback과 GNSS-denied Land/Descend를 현재 소스에서 다시 통과했다. 각각 `20260717T135258KST`, `20260717T135529KST`다.

## 2026-07-17 — nonlinear geometry health, long validation, PX4 handover gate

### `vins_fusion_ros2/vins/src/estimator/estimator.cpp`, `estimator.h`

- jump 진단에 timestamp/position/bias/active feature를 추가하고 nonlinear state trace를 기록한다.
- position output guard를 `0.5 m`에서 `0.25 m`로 낮췄다.
- active features `<100`과 optimizer correction `>0.10 m`가 동시에 발생하면 publish 전에 reset한다.
- 검증: historical 건강 9회 correction 최대 `0.026 m`, 실패 1회 `0.285 m`; final live 10회에서 geometry rejection 3회가 두 recovery run을 격리했고 최종 `HOVER_READY 10/10`, position jump 0이었다.

### `src/VIO_pkg/VIO_pkg/vio_bridge.py`

- readiness generation, `0.20 s` freshness, `0.25 m` jump, 10 healthy samples, recovery re-approval, conservative variance/quality 및 `/vio/handover_ready`를 추가했다.
- integration smoke: gate open, jump close, new ready reopen, stale close, `publish_visual_odometry=false`에서 PX4 message 0건.

### `vio_preflight_test.py`, `vio_vins.launch.py`, `run_clean_preflight_repeats.sh`, 분석 도구

- excitation repeats와 ready hold duration을 파라미터화했다.
- relay→recorder→estimator 분리 시작, post-metrics record grace, live DDS graph 검사, geometry event 및 solver 계측을 추가했다.
- `analyze_guard_ground_truth.py`, `test_vio_handover_gate.py`를 추가했다.
- `setup.py` data-file glob이 디렉터리를 복사하지 않도록 파일만 필터링했다.

## 2026-07-17 — unhealthy initialization rollback, SFM cheirality, publication probation

### `vins_fusion_ros2/vins/src/estimator/estimator.cpp`, `estimator.h`

- 수정 이유: initial Ceres solve가 이미 기존 bias failure limit을 넘긴 상태도 잠시 NON_LINEAR output으로 전환한 뒤 다음 frame에서 reset했음
- 핵심 변경: accel/gyro bias limit을 initial solve 직후 검사; pre-solve state/depth/extrinsic/time-delay snapshot을 복원하고 marginalization prior를 폐기한 뒤 next window로 slide; first nonlinear update 2회 동안 odometry publish를 보류하고 3회 통과 뒤 활성화
- 검증 결과: 첫 full-reset 구현은 replay에서 IMU epoch jump와 SIGSEGV를 만들어 폐기. rollback replay는 bias `2.74229 m/s²` 후보를 거부하고 다음 bias 약 `0.387 m/s²` 후보를 수락했으며 crash가 재발하지 않음. 정상 full bag은 probation 통과 후 failure 없음.
- 제한: post-BA/probation 최종 조합의 clean live 10회는 사용자 중단으로 완료하지 못함. 완료를 주장하지 않음.
- 롤백 주의사항: bias limit 자체는 nonlinear failure detector와 공유한다. rollback 제거 시 bad candidate가 output으로 노출되거나 전체 reset race가 다시 생길 수 있다.

### `vins_fusion_ros2/vins/src/initial/initial_sfm.cpp`

- 수정 이유: BA convergence만으로 camera 뒤쪽 또는 비유한 3-D geometry를 명시적으로 거르지 않았음
- 핵심 변경: BA 뒤 모든 triangulated observation의 camera depth를 재계산; nonfinite가 있거나 nonpositive depth ratio가 `5%`를 넘으면 `post_ba_cheirality` 단계에서 거부
- 검증 결과: 정상 full bag은 `0/1601`; recovery bag의 accepted/rejected bias 후보는 각각 `0~2.03%`로 cheirality false rejection 없이 통과. 이 gate가 live output-loss 성공률을 개선하는지는 반복 미완료.
- 롤백 주의사항: pre-BA diagnostics와 post-BA gate를 혼동하지 않는다. threshold 효과는 최종 live 반복으로 다시 확인해야 한다.

### `tools/analyze_initialization_quality.py`

- 수정 이유: 여러 batch의 accepted initialization을 수작업으로 비교하면 label과 지표 선택이 일관되지 않았음
- 핵심 변경: SFM/BA/alignment/초기 bias/early update/run outcome을 JSON 또는 TSV로 추출; 후속 estimator reset 유무를 분석 label로 부여
- 검증 결과: 기존 clean artifact에서 accepted initialization 72개 추출, Python compile 통과
- 제한: `reset/survived`는 ground-truth quality label이 아니며 외부 recorded/preflight reset도 포함할 수 있다.

### `tools/run_clean_preflight_repeats.sh`

- 수정 이유: initialization rejection/probation/SFM gate와 후속 jump/solver failure를 batch summary에서 자동 비교할 필요가 있었음
- 핵심 변경: candidate rejection, SFM cheirality rejection, probation pass, initialization completion, estimator reset, position jump, solver failure count를 metrics/TSV에 추가
- 검증 결과: `bash -n`, `git diff --check` 통과; batch `20260717T021346KST`와 `20260717T024957KST`에서 새 열 생성 확인
- 롤백 주의사항: count는 log 문자열 기반이며 알고리즘 상태를 바꾸지 않는다.

## 2026-07-17 — deterministic 10 Hz downsample과 queue backlog 계측

### `vins_fusion_ros2/vins/src/estimator/estimator.cpp`

- 수정 이유: queue-empty 기반 frame 선택을 제거한 all-frame enqueue가 replay는 결정적으로 만들었지만 live 처리량을 감당하는지 측정할 수 없었음
- 핵심 변경: enqueue/dequeue queue depth와 최신 대기 frame까지의 simulation-time span을 기록; all-frame 회귀 확인 후 callback count의 짝수 frame만 enqueue하는 고정 2:1 정책으로 변경
- 검증 방법: 동일 full bag 2회 replay, LiDAR ON clean live 10회, LiDAR OFF clean live 10회
- 검증 결과: replay 공통 선택 frame/feature fingerprint와 RANSAC 11개, initialization 3개, reset 2개 일치; 고정 downsample live 20회 모두 queue span p95 `0 s`; all-frame smoke run의 최대 depth/span `17/0.816 s` 회귀 제거
- 롤백 주의사항: queue 상태 조건을 복원하면 입력 sequence 비결정성이 다시 생기고, all-frame으로 복원하면 live backlog가 재현될 수 있다.

### `tools/run_clean_preflight_repeats.sh`

- 수정 이유: deterministic 입력 변경의 실시간 backlog와 실험별 실제 config를 run 결과와 함께 보존
- 핵심 변경: `feature_queue_max_depth`, `feature_queue_span_max_s`, `feature_queue_span_p95_s`를 `metrics.json`/`summary.tsv`에 병합; root 소유 metrics는 임시 파일 원자 교체로 갱신; `EXPERIMENT_LABEL`, active config snapshot, SHA-256 기록 추가
- 검증 결과: `bash -n`, `git diff --check`, Docker colcon build 통과; ON/OFF 두 batch의 모든 terminal run에서 queue metrics와 summary 생성
- 롤백 주의사항: 계측 제거는 estimator 계산에 영향이 없지만 live backlog 회귀를 자동 검출할 수 없고 실험 config 추적성이 사라진다.

### `vins_fusion_ros2/config/euroc/euroc_mono_imu_config.yaml`

- 수정 이유: range factor가 failure-free 성공률을 낮추는지 단일 변수로 검증
- 핵심 변경: 실험 중 `use_range: 1 → 0`; 10회 종료 뒤 근본 개선이 없어 `use_range: 1` 복원
- 검증 결과: ON 최종/무복구 `6/10, 6/10`; OFF 최종/무복구 `9/10, 6/10`, recovery 3회. range factor 단독 원인 가설 기각
- 롤백 주의사항: 최종 소스는 LiDAR ON이며 OFF 결과는 batch의 `active_vins_config.yaml`에 보존된다.

## 2026-07-16 — camera/feature trace와 deterministic replay 입력

### `vins_fusion_ros2/src/vins_estimator.cpp`, `vins/common/sensor_data_type.h`

- 수정 이유: 같은 bag에서도 reset 및 processing thread timing에 따라 estimator가 소비하는 frame/window가 달라짐
- 핵심 변경: callback sequence, image/feature fingerprint, enqueue 사유 기록; recorded reset을 image callback group에서 적용하는 opt-in `replay_reset_events` 추가; queue-empty 기반 frame 선택을 제거하고 relay frame 전부 enqueue
- 검증 결과: live bag overlap 4,161/4,161 timestamp 일치; in-process reset replay가 live `57.116 s` initialization fingerprint 재현; 변경 뒤 replay 2회의 공통 805 frame과 핵심 event 14개 일치
- 롤백 주의사항: `replay_reset_events` 기본값은 `false`다. all-frame enqueue는 live 성능/성공률 검증 전이며 원복 시 replay 비결정성이 다시 생긴다.

### `vins_fusion_ros2/vins/src/initial/solve_5pts.cpp`

- 수정 이유: live/replay relative-pose 차이가 OpenCV RNG인지 correspondence 입력인지 구분
- 핵심 변경: correspondence fingerprint, RNG state 전후, inlier 수 진단 추가
- 검증 결과: RNG state는 관측 후보 모두 `0xffffffff`로 같았고 correspondence fingerprint 차이/일치를 직접 구분함

### `src/VIO_pkg/VIO_pkg/vio_frame_trace_compare.py`, `setup.py`

- 수정 이유: 압축 bag relay frame과 VINS callback trace의 timestamp 완전성을 반복 계산
- 핵심 변경: overlap count, timestamp 누락/추가, monotonicity, sequence gap JSON 분석기 추가
- 검증 결과: 새 live full bag overlap에서 trace/bag 4,161개가 전부 일치하고 양쪽 전용 timestamp 0개 확인

## 2026-07-16 — SFM 실패 단계 및 BA geometry 진단

### `vins_fusion_ros2/vins/src/initial/initial_sfm.cpp`

- 수정 이유: `GlobalSFM::construct()`의 단일 false 결과만으로는 live run04의 반복 실패가 PnP, triangulation, BA 중 어디에서 발생했는지 구분할 수 없었음
- 핵심 변경: forward/backward PnP 실패 frame과 이유, triangulated/finite feature 수, observation 수, 비유한·비양수 depth 수와 범위, Ceres BA summary/cost/step 수를 기록
- 검증 방법: Docker ROS 2 Humble에서 `vins_fusion_ros2` 빌드 후 run04 저장 relay 토픽만 2배속 단일-source replay
- 검증 결과: 빌드 성공. replay 첫 후보는 음수 depth observation 86개와 BA final cost `0.08226`으로 실패했고, 두 번째 후보는 음수/비유한 observation 0개와 final cost `0.003167`로 수렴함을 확인
- 롤백 주의사항: 이번 변경은 로그만 추가했으며 SFM 계산, solver option, acceptance gate, PX4 publish 설정을 변경하지 않는다.

## 2026-07-16 — rosbag 독립 preflight 시작 지연

### `tools/run_clean_preflight_repeats.sh`

- 수정 이유: recorder 준비의 2초 대기와 bag 기록 부하를 분리해 시작 callback/frame 조건 가설을 검증
- 핵심 변경: `PREFLIGHT_START_DELAY_SECONDS` 비음수 설정 추가, preflight 시작 직전 독립 대기, `batch_config.tsv`에 실행 조건 보존
- 검증 결과: `bash -n`, 잘못된 값 거부, `git diff --check` 통과; 2초·무기록 10회에서 최종 통과 8/10, 무복구 통과 6/10
- 롤백 주의사항: 기본값은 `0`이어서 기존 실행 timing은 바뀌지 않는다. estimator 및 PX4 handover 설정에는 영향이 없다.

## 2026-07-16 — 압축 full-bag 반복 및 초기화 window 분석

### `tools/run_clean_preflight_repeats.sh`

- 수정 이유: 제한된 디스크에서 인접 성공/실패 ground-truth bag을 자동 확보
- 핵심 변경: `RECORD_BAGS=1` file-zstd 기록, `STOP_ON_OUTCOME_CHANGE=1` outcome 전환 조기 종료, recorder 정상 종료/압축 대기
- 검증 결과: 6회 full bag의 metadata와 압축 파일 생성, 각 `196~309 MB`, run 후 잔여 프로세스 0개
- 롤백 주의사항: 기본값은 기존처럼 bag 미기록이므로 estimator 실행에는 영향이 없다.

### `src/VIO_pkg/VIO_pkg/vio_bag_metrics.py`, `package.xml`

- 수정 이유: rosbag2 Python reader가 file-zstd bag을 sqlite로 직접 열지 못함
- 핵심 변경: `.db3.zstd`를 임시 디렉터리에 해제하고 metadata를 읽기용으로 변환한 뒤 자동 정리; `zstd` 실행 의존성 명시
- 검증 결과: 압축 run05 마지막 연속 segment 평가 성공, 임시 디렉터리 잔존 0개

### `src/VIO_pkg/VIO_pkg/vio_init_window_metrics.py`, `setup.py`

- 수정 이유: VINS sim-time 초기화 window의 실제 PX4 운동량을 반복 계산할 도구가 없었음
- 핵심 변경: `/clock`으로 기록 시각을 sim-time에 보간하고 window별 displacement/path/endpoint rotation JSON 출력
- 검증 결과: run04 정상 및 run05 비정상/recovery window JSON 생성; `py_compile`, colcon build, `git diff --check` 통과

## 2026-07-16 — clean 반복 실행기 및 ENU/NED 평가 수정

### `tools/run_clean_preflight_repeats.sh`

- 수정 이유: 잔여 relay 등 환경 오염을 자동 차단하고 clean 반복 성공률을 재현 가능하게 측정
- 핵심 변경: VINS/Gazebo/PX4/XRCE cleanup, 센서/clock/relay/PX4-input 검사, per-run JSON·로그·TSV summary, run 후 잔여 프로세스 검사
- 검증 결과: 10회 모두 infrastructure check를 통과하고 run 후 잔여 프로세스 0개; 결과 6회 성공/4회 실패 기록
- 롤백 주의사항: 실행기 제거는 estimator 동작에 영향을 주지 않지만 반복 실험 조건 자동 검사가 사라진다.

### `src/VIO_pkg/VIO_pkg/vio_preflight_test.py`, `vio_bag_metrics.py`

- 수정 이유: VINS ENU와 PX4 NED 위치를 raw 상태에서 yaw만 fit해 전체 trajectory vector error가 과대 계산됐음
- 핵심 변경: no-scale yaw/translation alignment 전에 고정 ENU→NED 변환 `(x,y,z)→(y,x,-z)` 적용
- 검증 결과: 기준 성공 bag의 3D RMSE `1.036 m → 0.076 m`, 최대 오차 `3.368 m → 0.188 m`; 빌드 및 `py_compile`, `git diff --check` 통과
- 롤백 주의사항: estimator, range factor, READY gate는 바뀌지 않았으며 평가 좌표 변환만 원복된다.

## 2026-07-16 — 초기 heading 보존 및 건강한 VINS 상태 유지

### `src/VIO_pkg/VIO_pkg/vio_preflight_test.py`

- 수정 이유: simulator 초기 yaw가 약 `+90°`인데 병진 waypoint가 절대 yaw `0°`를 동시에 명령해 첫 초기화 창에 `45.47°/0.504 s` 회전을 섞었음
- 핵심 변경: 첫 PX4 odometry yaw를 기준 heading으로 저장하고 병진 구간에는 이를 유지하며 마지막 yaw excitation만 기준 heading 대비 상대각으로 수행
- 추가 변경: 이륙 중 VINS odometry가 신선하면 hover에서 건강한 estimator를 강제 reset하지 않고 유지하며, 출력이 없을 때만 기존 reset fallback 사용
- 검증 결과: clean live run에서 scale `0.500626`, 초기 accel bias norm 약 `0.055 m/s²`, path scale `1.04`, 60초 hover drift 최대 약 `0.10 m`로 `HOVER_READY` 통과
- 제한: clean 성공은 1회이므로 반복 성공률 검증이 필요하며 PX4 handover는 비활성 유지

## 2026-07-16 — alignment 및 projection 비유한 값 진단 보강

### `vins_fusion_ros2/vins/src/initial/initial_aligment.cpp`

- gravity refinement 4회별 tangent update, gravity, scale을 기록
- physical design matrix condition과 scale observability 진단을 추가; 알고리즘 gate로 사용하지 않음

### `vins_fusion_ros2/vins/src/factor/projectionTwoFrameOneCamFactor.cpp`

- projection 입력, 보정 관측점, camera depth, 최종 residual이 비유한 경우 값을 기록
- 단일-source replay에서 이중 relay 오염의 직접 실패값 `velocity_j=(-nan,-nan)`을 확인
- 계산이나 factor acceptance는 변경하지 않음

## 2026-07-16 — VINS 초기화 진단 추가

### `vins_fusion_ros2/vins/include/vins/estimator/estimator.h`

- 수정 이유: 초기화 window 진단 함수를 estimator interface에 추가
- 핵심 변경: `logInitializationDiagnostics()` private method 선언
- 검증 방법: Docker에서 `vins_fusion_ros2` 빌드 및 rosbag 재생
- 검증 결과: 빌드 성공, 진단 출력 확인
- 롤백 주의사항: 선언만 제거하면 안 되며 `estimator.cpp` 구현과 호출도 함께 제거해야 한다.

### `vins_fusion_ros2/vins/src/estimator/estimator.cpp`

- 수정 이유: 성공/실패 초기화의 입력 window와 초기 optimizer 변화를 비교할 증거가 부족했음
- 핵심 변경:
  - 초기화 window timestamp와 camera/IMU duration 기록
  - IMU sample 수, dt 범위, acceleration/gyro 평균과 RMS 기록
  - feature 수, track 지속성, parallax 기록
  - 초기 3회 nonlinear update의 position correction과 accel/gyro bias 전후 기록
- 검증 방법: `gazebo_direct_vio` rosbag 재생
- 검증 결과: 실패 초기화에서 초기 최적화 직후 accel bias가 이미 커지는 사례와 후속 update에서 커지는 사례를 모두 구분할 수 있었음
- 롤백 주의사항: 진단만 제거해야 하며 기존 failure reset, range factor, state validation 변경과 혼동하면 안 된다.

### `vins_fusion_ros2/vins/src/initial/initial_aligment.cpp`

- 수정 이유: visual rotation과 IMU preintegration rotation의 불일치를 초기화마다 비교하기 위함
- 핵심 변경: gyro bias solve 전 rotation residual pair 수, 평균/최대 각도, solved gyro bias delta 기록
- 검증 방법: 진단 rosbag 재생
- 검증 결과: 실패 window에서도 평균 residual 약 `0.236°`로 작음을 확인
- 롤백 주의사항: 계산은 진단용이며 gyro bias solve 결과에는 영향을 주지 않는다.

## 2026-07-16 — 원복된 실험

- 최다 RANSAC inlier relative-pose 후보 선택: 성능 악화로 원복
- essential-matrix RANSAC threshold `0.5 px`: 성능 악화로 `1.0 px` 복원
- 최종 소스에는 위 두 실험의 알고리즘 변경이 남아 있지 않다.

## 2026-07-17 — 무복구 최종 후보와 PX4 fault integration

- `vins_fusion_ros2/vins/include/vins/estimator/feature_manager.h`, `feature_manager.cpp`, `estimator.cpp`: factor 최소 track 길이를 공통 3 frame 상수로 통일했다. 2 frame은 unsafe step 회귀로 폐기했다.
- `vins_fusion_ros2/vins/src/estimator/estimator.cpp`: nonlinear solver를 Dogleg에서 LM으로 바꾸고 low-geometry correction 및 output position guard를 `0.15 m`로 정렬했다. 최종 10회에서 post-ready reset/solver failure 0으로 검증했다.
- `src/VIO_pkg/VIO_pkg/vio_bridge.py`: output age, monotonic sample time, `0.15 m` step, `1.5 m/s` implied speed gate와 readiness generation 기반 PX4 `reset_counter`를 추가했다. 기본 PX4 publish는 계속 false다.
- `src/VIO_pkg/VIO_pkg/vio_preflight_test.py`: 안전한 GPS hover 상태의 validation timeout을 30→45초로 늘려 늦은 건강한 init이 8초 검증을 끝낼 수 있게 했다.
- `src/VIO_pkg/VIO_pkg/vio_px4_fault_test.py`, `src/VIO_pkg/setup.py`: 실제 EKF2 EV fusion 확인, reset 주입, GPS fallback 또는 GNSS-denied Land/Descend를 판정하는 integration executable을 추가했다.
- `tools/test_vio_handover_gate.py`: PX4와 호환되는 QoS로 실제 output, jump/stale close, re-ready, reset counter 증가를 검증하도록 수정했다.
- `tools/run_clean_preflight_repeats.sh`: PX4 fault test 실행/cleanup/metrics 대기, bool parameter, 새 환경변수 검증 및 batch config 기록을 추가했다.
- 검증: 최종 clean 10/10 terminal·post-ready 무복구, GPS fallback pass, GNSS-denied blind-land pass, ROS build와 handover smoke pass.
- 롤백 주의: LM/3-frame/0.15 m/45 s는 최종 통계가 결합된 한 후보 구성이다. 하나만 되돌리면 기존 batch와 직접 비교할 수 없으므로 새 10회 이상을 수행한다.

## 2026-07-17 — in-flight re-handover fail-closed 및 fault runner 판정

- `src/VIO_pkg/VIO_pkg/vio_bridge.py`: `allow_rehandover_after_fault=false` 기본 latch를 추가했다. 이미 열린 gate의 fault 뒤 readiness를 무시하고 bridge restart를 요구한다.
- `tools/test_vio_handover_gate.py`: opt-in 재연결/reset counter와 production 기본 latch-closed를 함께 검증한다.
- `tools/run_clean_preflight_repeats.sh`: PX4 fault JSON을 primary metrics에 병합하고 누락/실패 시 outcome/exit code를 실패로 만든다.
- `estimator.cpp` quarantine 및 feature `max_cnt=220` 실험은 unsafe/성능 회귀로 완전히 원복했다. 최종 estimator 동작은 이전 LM + full-reset guard와 동일하다.
- 검증: handover smoke, GPS fallback, GNSS-denied blind land, 원복 safety smoke, build/py_compile/bash syntax/diff check 통과.
