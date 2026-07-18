# Run History

## 2026-07-17 16:00~19:00 KST — 시각화/trace scheduling 및 reset-epoch 후보 반증

- initialization final cost-per-block은 failure/healthy/recovery 분포가 겹쳐 acceptance gate 후보에서 제외했다. 동작을 바꾸지 않는 초기화/solver 진단 로그만 남겼다.
- LM trust radius 제한, marginalization `NUM_THREADS=1`, isolated Ceres 1.14 static-link A/B는 결정적 failure를 없애지 못해 모두 원복했다. 최종 build는 system Ceres 2, `NUM_THREADS=4`다.
- `show_track=0` 배치 `20260717T164507KST`: 총 10회 중 PX4 auto preflight disarm 인프라 1회, 유효 terminal/recovery 0 `9/9`, jump/solver-failure-free `8/9`. 비행 중 불필요한 시각화 제거는 유지했다.
- frame fingerprint/queue trace를 기본 off로 한 `20260717T171548KST`: 유효 terminal `9/9`, recovery 0 `6/9`, jump-free `6/9`; trace 부하 제거가 오히려 scheduling 민감성을 노출해 변경을 원복했다.
- reset과 feature queue 사이 epoch race 가설로 stale frame tag/drop을 구현했다. atomic enqueue `20260717T175537KST`는 terminal `10/10`, recovery 0 `8/10`, strict `5/10`; nonblocking `20260717T182304KST`는 terminal `9/10`, 통과 run recovery 0 `9/9`, strict terminal `7/9`, 실패 run은 recovery 1/jump 2/solver failure 4였다.
- stale frame drop 자체는 live에서 관측됐지만 전체 성공률을 개선하지 못했다. 두 reset-epoch 구현은 전량 원복했으며 artifact만 근거로 보존한다.
- 최종 판정: 무복구 100% 근본 해결은 미완료. 최종 코드에는 `show_track=0`만 남고 frame trace와 기존 reset/queue 동작은 유지된다. 사용자 요청에 따라 추가 30회/PX4 회귀는 수행하지 않았다.

## 2026-07-17 14:26~16:00 KST — solver/prior 분해와 운용 후보 반증

- 30회 solver soak `20260717T142655KST`는 7회째 validation timeout과 position jump 5회를 재현해 진단 후 중단했다. fault solve는 모두 iteration 12의 `NO_CONVERGENCE`; 25 iteration은 첫 live run recovery 2회로 악화돼 원복했다.
- validation dither ±0.10 m `20260717T144944KST`는 6회째 PX4 hold displacement gate를 넘었고, ±0.04 m `20260717T151641KST`는 terminal `10/10`, recovery 0 `9/10`, jump-free `8/10`이었다. 둘 다 제거했다.
- excitation speed 0.40 m/s `20260717T150809KST`는 2/2 내부 jump가 발생해 중단했다. runner에는 비교 재현용 `EXCITATION_SPEED` 인자만 남겼고 기본은 0.55다.
- deterministic fault의 residual breakdown은 final cost `4551` 중 marginalization prior `4159`, visual `346`, IMU `44`, range `3`이었다. 기존 outlier 삭제를 prior 생성 앞으로 옮겨도 89.628초 jump가 재현되어 변경/진단을 원복했다.
- threshold 3.5 px는 deterministic replay에서 90.99초까지 통과했으나 live `20260717T155406KST` 첫 run이 features 24, jump `0.1538 m`, scale `1.0836`, validation timeout으로 실패해 즉시 4 px로 복원했다.
- 판정: 안전막은 계속 유효하지만 무복구 100% 근본 개선은 미완료. marginalization prior 불일치가 현재 가장 강한 원인 단서다.

## 2026-07-17 12:07~14:25 KST — deterministic outlier 원인 분리, 4 px 후보, PX4 재검증

- failure bag `20260717T040404KST_run09`에서 recorded reset을 무시하고 첫 camera frame 한 장만 replay 진단에서 건너뛰어 live 입력 시작을 일치시켰다. 89.628초 position jump `0.1572 m`를 반복 재현했다.
- Cauchy loss, Huber delta 2, 4-frame factor, low-geometry hold, earlier old marginalization은 실패를 앞당기거나 악화해 전부 원복했다.
- 3 px outlier 삭제 replay는 hover 구간에서 매 update 1~5 long feature를 제거해 active `149→63`, long `121→35` 뒤 jump했다. 5 px/4 px는 동일 bag의 해당 구간을 통과했다.
- 5 px live 10회 `20260717T130451KST`: terminal `9/10`, recovery 0 `8/10`; validation timeout 1. 기각.
- 4 px live 10회 `20260717T133105KST`: terminal/recovery 0 `10/10`, strict fault-free `6/10`; scale 평균 `1.0261`, RMSE 평균 `0.0874 m`, 공개 max step 최악 `0.1411 m`.
- logging-clean 후속 3회 `20260717T140142KST`: terminal/recovery 0 `2/3`; late jump 뒤 recovery했으나 hover drift acceptance에서 fail-closed 1. 따라서 안정적 10/10 주장은 철회.
- solver 진단 5회 `20260717T141251KST`: terminal/recovery 0 `5/5`, early jump 1. fault solve는 features 19, cost `16277.7→15216.6`, 13 successful step, `NO_CONVERGENCE`, 6.3 ms.
- GPS fallback `20260717T135258KST`: EV stop/gate close, GPS fusion active, failsafe false, max local step `0.00148 m`, pass.
- GNSS denied `20260717T135529KST`: EV/GPS false, gate close, failsafe true, nav state 12 Land/Descend, max step `0.00304 m`, pass.
- 판정: operational 합산 `17/18`로 개선됐고 PX4 보호막은 통과. strict 초기 retry와 rare late mutation 때문에 목표는 아직 완전 종료하지 않는다.

## 2026-07-17 03:38~06:26 KST — 4단계 완료 검증

- final initialization full-bag batch `20260717T033805KST`: `HOVER_READY 9/10`, `VALIDATION_TIMEOUT 1`, 무복구 8/10. 정상 bias candidate false rejection 0.
- output-loss/recovery run09: accepted initialization/probation 뒤 약 59초 후 `0.7435 m` internal jump. 같은 `0.1 s` PX4 step `0.0029 m`, solver/bias-limit 선행 없음. guard true positive.
- nonlinear trace failure run: active geometry가 `<100`으로 장시간 축소되고 correction `0.285 m`, bias 최대 `1.86 m/s²`; 건강 9회 correction 최대 `0.026 m`, low-feature 연속 최대 32 대 실패 112.
- long LiDAR ON `20260717T050050KST`: excitation cycle 3, hover 180초, recovery 0, RMSE 3D/z `0.117/0.100 m`, queue p95 `0.1 s`.
- long LiDAR OFF `20260717T050929KST`: recovery 0, RMSE 3D/z `0.123/0.100 m`, queue p95 `0 s`, container CPU 평균/p95/max `117/138/237%`.
- 0.25 m guard 회귀 `20260717T051945KST`: `HOVER_READY 9/10`, guard false reset 0; 한 run의 느린 geometry corruption을 확인.
- final geometry gate 회귀 `20260717T055314KST`: `HOVER_READY 10/10`, 무복구 8/10, recovery 2, geometry rejection 3, position jump 0. 두 recovery run은 최종 정상 scale로 복구.
- handover smoke: ready→10 healthy samples open, jump close, new ready reopen, stale close, PX4 output 0.
- 최종 설정: `use_range=1`, `publish_visual_odometry=false`.

## 2026-07-16 02:28 KST — 현재 소스 빌드 확인

- 목적: 기존 수정 상태가 Docker ROS 2 환경에서 빌드되는지 확인
- 명령: `colcon build --packages-select vins_fusion_ros2 VIO_pkg --symlink-install`
- 조건: Docker `project_f`, ROS 2 Humble, 기존 dirty worktree 보존
- 수정 파일: 없음
- 결과: 두 패키지 빌드 성공
- 판정: 성공
- 로그: `/home/reventon/ros2_ws/log`의 해당 colcon build 디렉터리
- rosbag: 없음
- 판단 영향: 현재 소스를 기준 실험에 사용할 수 있음을 확인

## 2026-07-16 02:30 KST — 기준 rosbag 재생

- 목적: 현재 VINS로 초기화 실패를 재현
- 명령: `ros2 bag play .../gazebo_direct_vio --rate 2.0 --remap /odometry:=/recorded/odometry /path:=/recorded/path`
- 조건: `vio_vins.launch.py`, recorded VINS output 격리, range topic은 bag에 없음
- 관련 설정: camera relay 최대 25 Hz, VINS mono+IMU, fixed extrinsic, `td=0`
- 수정 파일: 없음
- 결과: 첫 초기화 scale `0.0715`, 초기 accel bias norm은 작음. 이후 position jump `0.5196 m`로 reset. 후속 초기화 뒤 accel bias norm `3.1011`, `2.5200 m/s²`로 증가해 reset.
- 판정: 실패 재현 성공 / VIO 안정성 실패
- 실패 원인: 이 단계에서는 미확정
- 로그: 컨테이너 `/tmp/vio_replay_baseline.log`
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: bias가 항상 초기 정렬부터 큰 것은 아니며 후속 최적화에서도 폭발함을 확인

## 2026-07-16 02:31 KST — 동일 rosbag 반복 재생

- 목적: 고정 입력에서 결과 반복성 확인
- 명령: 기준 재생과 동일, 약 100초 simulation 구간 재생
- 조건: 첫 재생과 동일
- 수정 파일: 없음
- 결과: 첫 scale `0.1024`, position jump `0.5661 m`, 다음 scale `0.5663`. 첫 재생과 수치가 달랐고 relay 초기 수신 frame 수도 한 장 차이.
- 판정: 반복성 실패
- 실패 원인: 시작 frame/callback scheduling/RANSAC 중 무엇인지 미확정
- 로그: 컨테이너 `/tmp/vio_replay_repeat.log`
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: simulation timestamp와 window 통계가 포함된 진단 로그 필요성 확인

## 2026-07-16 02:36 KST — 초기화 진단 코드 검증 재생

- 목적: 성공/실패 초기화 window를 정량 비교할 진단값 수집
- 명령: 진단 코드 빌드 후 기준 bag 2배속 재생
- 조건: 알고리즘 설정은 기준과 동일
- 수정 파일: `estimator.h`, `estimator.cpp`, `initial_aligment.cpp`
- 결과: window timestamp/duration, IMU sample/dt/mean/RMS, feature track 통계, visual/IMU rotation residual, 초기 3회 nonlinear bias 변화가 출력됨.
- 주요 성공 window: `28.184~29.936 s`, scale `0.1024`, 초기 bias 작음, rotation residual 평균 `0.216°`.
- 주요 실패 window: `86.108~87.508 s`, scale `0.0474`, 초기화 직후 accel bias 약 `(-0.15, -3.15, 0.35)`, rotation residual 평균 `0.236°`.
- 판정: 진단 기능 성공 / VIO 안정성 실패
- 로그: 컨테이너 `/tmp/vio_replay_diagnostics.log`
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: rotation consistency보다 translation/scale inconsistency가 더 유력한 후보가 됨

## 2026-07-16 02:38 KST — PX4 ground truth window 비교

- 목적: 실패 초기화가 운동 부족인지 확인
- 명령: `rosbag2_py`로 camera header timestamp와 PX4 vehicle odometry를 결합한 read-only 분석
- 조건: 진단 로그에서 선택한 네 window 비교
- 수정 파일: 없음
- 결과:
  - 안정 window `28.184~29.936 s`: displacement `0.613 m`, path `0.639 m`, rotation `1.56°`
  - 실패 window `86.108~87.508 s`: displacement `0.677 m`, path `0.723 m`, rotation `7.93°`
  - visual translation 방향 오차: 안정 약 `24.7°`, 실패 약 `53.7°`
- 판정: 운동 부족 가설 기각
- 로그: 터미널 분석 출력, 별도 파일 없음
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: 실패 window에 충분한 병진 운동이 있었고 visual translation 방향이 더 크게 틀렸음을 확인

## 2026-07-16 02:40 KST — 최다 inlier relative-pose 후보 선택 실험

- 목적: 첫 통과 후보 대신 가장 강한 RANSAC 후보를 선택하면 안정화되는지 검증
- 명령: 변경 빌드 후 기준 bag 2배속 재생
- 조건: 모든 reference frame 후보를 평가하고 inlier 수가 가장 큰 후보 선택
- 수정 파일: 실험 중 `solve_5pts.h`, `solve_5pts.cpp`, `estimator.cpp`; 결과 후 전부 원복
- 결과: position jump가 약 `0.84 m`로 악화되고 후속 scale이 `0.0052`까지 감소
- 판정: 실패, 원복 완료
- 실패 원인: inlier 수만으로 병진 방향의 물리적 일관성을 보장하지 못함
- 로그: 컨테이너 `/tmp/vio_replay_best_candidate.log`
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: 최다 correspondence/inlier 휴리스틱을 해결책으로 사용하지 않기로 결정

## 2026-07-16 02:43 KST — essential-matrix RANSAC 0.5 px 실험

- 목적: 1.0 px threshold가 outlier를 과도하게 허용하는지 검증
- 명령: threshold 변경 빌드 후 기준 bag 2배속 재생
- 조건: `1.0/277.191356`에서 `0.5/277.191356`으로 변경
- 수정 파일: 실험 중 `solve_5pts.cpp`; 결과 후 원복
- 결과: 초기화가 늦어지고 scale `0.006~0.011`, accel bias limit reset 반복
- 판정: 실패, 원복 완료
- 실패 원인: tighter threshold가 안정적인 translation을 만들지 못하고 usable correspondence를 줄임
- 로그: 컨테이너 `/tmp/vio_replay_ransac_half.log`
- rosbag: `/home/reventon/ros2_ws/bags/gazebo_direct_vio`
- 판단 영향: RANSAC threshold 단순 강화 접근을 반복하지 않기로 결정

## 2026-07-16 02:45 KST — 최종 원복 및 빌드

- 목적: 실패 실험을 제거하고 진단 코드만 남은 상태 검증
- 명령: `colcon build --packages-select vins_fusion_ros2 --symlink-install`, `git diff --check`
- 조건: RANSAC `1.0 px`, 기존 첫 통과 relative-pose 선택 복원
- 수정 파일: 최종 유지 파일은 `estimator.h`, `estimator.cpp`, `initial_aligment.cpp`
- 결과: 빌드 성공, `git diff --check` 통과, 실행 중 ROS/Gazebo/PX4 프로세스 없음
- 판정: 성공
- 로그: `/home/reventon/ros2_ws/log`의 마지막 colcon build 디렉터리
- rosbag: 없음
- 판단 영향: 다음 세션은 진단 가능한 기준 상태에서 live run으로 시작 가능

## 2026-07-16 03:03 KST — 완전 기록 live Gazebo preflight run

- 목적: 현재 진단 빌드에서 live Gazebo/PX4 입력으로 첫 VINS initialization 성공 또는 첫 failure까지의 증거를 영구 보존
- 명령: `HEADLESS=1 PX4_SITL_WORLD=vio_feature GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-} make px4_sitl gazebo-classic_iris_fpv_cam`, `ros2 bag record ...`, `ros2 launch VIO_pkg vio_preflight_test.launch.py`
- 조건: Docker `project_f`, ROS 2 Humble, Gazebo Classic `vio_feature.world`, `iris_fpv_cam`, current diagnostic build; `publish_visual_odometry=false`; 알고리즘 파라미터와 소스 수정 없음
- 실행 전 검증: `/clock`, `/vio/imu`, `/vio/camera/image_raw`, `/vio/range`, `/fmu/out/vehicle_odometry` publisher 확인; recorder도 workspace overlay를 source해 PX4 message type 구독 확인
- 수정 파일: 없음 (문서 및 artifact 기록만 추가)
- 결과: 첫 window `33.972~34.572 s`, camera/IMU duration 모두 `0.600 s`, IMU `79` samples. relative-pose translation `(0.57915, -0.244893, 0.777569)`, alignment scale `0.112983`, initial accel bias `(-1.22881, -0.615287, 0.125753) m/s²`. initialization complete 뒤 accel bias norm `2.50685 m/s²`로 first reset 발생.
- 판정: VINS initialization 성공 관측 / 후속 안정성 실패. 이 1회는 재현 가능한 해결이 아님.
- 로그: `artifacts/live_preflight/20260716T0330KST/vio_preflight_stdout.log`, `px4_gazebo.log`, `rosbag_record.log`
- rosbag: `artifacts/live_preflight/20260716T0330KST/sensor_preflight_bag/` — 58.218 s, 19,434 messages; raw/relay camera, IMU, range, PX4 odometry(7,201), event, VINS odometry 포함
- 다음 판단 영향: live 입력에서도 작은/불안정 scale과 accel-bias reset의 연결이 관측됐다. 다음에는 이 첫 window의 visual translation 방향을 PX4 ground truth와 비교해야 한다.

## 2026-07-16 02:55~03:02 KST — 보존된 무효/불완전 환경 시도

- 첫 환경 시도: `GAZEBO_PLUGIN_PATH`에 ROS Gazebo library 경로가 없어 IMU/camera/range publisher가 0개. VINS 입력이 없었으므로 결과를 run으로 판정하지 않음.
- 두 번째 기록 시도: sensor flow는 있었으나 recorder가 workspace overlay를 source하지 않아 PX4 odometry가 bag에 저장되지 않음. 완전 기록 run으로 판정하지 않음.
- 보존 경로: `artifacts/live_preflight/20260716T0312KST/`, `artifacts/live_preflight/20260716T0316KST/`
- 판단 영향: live run 시작 전 sensor publisher와 recorder의 PX4 type support를 함께 검증해야 함.

## 2026-07-16 04:10~06:30 KST — 초기화 원인 분리 및 무효 replay 확인

- IMU/PX4 운동학 대조: 동적 벡터 상관 `0.99`, 최적 지연 약 `+8 ms`; 단위·중력·축·큰 시간 오프셋 가설은 주원인에서 제외
- 목표 실패 창 `49.608~51.372 s`: visual 방향 오차 `2.96°`, 물리 baseline 약 `0.878 m`, 추정 scale `0.02093`로 baseline의 `2.4%`
- 중력 normal-equation rebuild 실험: 목표 실패 수치가 변하지 않아 원복
- 잘못된 replay 절차: 저장 relay와 새 relay를 동시에 공급한 실험은 알고리즘 증거에서 제외
- 단일-source 재생 절차: `vins_fusion_ros2.launch.py` + 저장 relay bag; 목표 실패 수치 재현
- projection NaN 진단: `artifacts/replay_projection_nan/20260716T0630KST/`, 직접 입력 `velocity_j=(-nan,-nan)` 확인

## 2026-07-16 05:40 KST — 초기 heading 보존 첫 live 비교

- 변경: 초기 PX4 yaw를 기준 heading으로 유지하고 병진 뒤 상대 yaw excitation 수행
- 이륙 초기화: yaw 변화 `0.18°`, direction error `3.25°`, scale/baseline `1.094`, 초기 bias norm 약 `0.035 m/s²`
- hover 강제 reset 뒤 첫 실패: direction error `28.30°`, scale/baseline `0.137`, 초기 accel bias norm 약 `4.36 m/s²`
- 판정: heading 수정은 이륙 초기화를 개선했으나 강제 reset 뒤 재초기화는 실패
- 보존: `artifacts/live_preflight/20260716T0540KST/`

## 2026-07-16 06:15 KST — READY 뒤 이중 relay 오염 run

- 결과: path scale `1.04`, 579 paired increments, `VALIDATE -> READY_HOLD`까지 성공
- 실패: READY 직후 projection residual NaN과 VINS reset
- 원인: 이전 replay의 relay가 남아 simulation `92.964 s`부터 두 relay가 같은 토픽을 발행; bag relay timestamp 중복/역행 1,012개
- 판정: VINS 알고리즘 실패가 아닌 환경 오염, 최종 성공/실패 통계에서 제외
- 보존: `artifacts/live_preflight/20260716T0615KST/`

## 2026-07-16 06:45 KST — clean 단일 relay full preflight 성공

- 사전 조건: 잔여 ROS/Gazebo/PX4 프로세스 0개, `/clock` 진행 확인, raw camera publisher 1개, launch 후 relay publisher 1개
- 변경 조건: 초기 heading 보존, 건강한 이륙 VINS 상태의 hover reset 생략; VINS 알고리즘 파라미터 변경 없음
- 초기화: scale `0.500626`, accel bias `(-0.04425, 0.00529, 0.03173) m/s²`, 초기 3회 nonlinear update 안정
- trajectory 검증: 699 paired increments, path scale `1.04`, vector error `0.19 m`, max step `0.06 m`
- hover 검증: 60초 drift error 최대 약 `0.10 m`, `VIO READY: 60 s hover drift check passed`, `HOVER_READY`
- rosbag: 161.21초, 53,042 messages, relay 2,941 frames, relay timestamp 중복/역행 0개
- 오류 검색: Ceres failure, projection NaN, accel bias limit, FAILSAFE 없음
- 판정: 성공 1회. 재현 가능한 해결 판정 전 10회 이상 clean 반복 필요
- 보존: `artifacts/live_preflight/20260716T0645KST/`
- 안전: `publish_visual_odometry=false`; PX4 handover 미실시

## 2026-07-16 18:37~19:09 KST — 자동 clean preflight 10회 반복

- 목적: heading 보존/단일 relay 성공의 반복성과 초기화 scale/bias 분포를 clean live Gazebo에서 측정
- 명령: `./tools/run_clean_preflight_repeats.sh 1`, 이후 corrected metric 빌드에서 `./tools/run_clean_preflight_repeats.sh 9`
- 조건: 매 run 전후 잔여 VINS/relay/TF/recorder/Gazebo/PX4/XRCE 프로세스 0개; 센서 publisher와 message 확인; `/clock` 진행; relay publisher 1개; `/fmu/in/vehicle_visual_odometry` publisher 0개; full bag 미기록
- 수정 파일: `tools/run_clean_preflight_repeats.sh`, `vio_preflight_test.py`, `vio_bag_metrics.py`
- 결과: `HOVER_READY` 6/10(60%), `VALIDATION_TIMEOUT` 3/10, `VINS_OUTPUT_LOST` 1/10
- 성공 path scale: `1.0247~1.0417`, 평균 `1.0321`
- timeout path scale: `1.0529~1.0910`, 평균 `1.0721`
- output-lost run: recovery 2회, position jump `1.541 m`와 `1.714 m`; 세 번째 출력 손실 뒤 FAILSAFE
- 초기화 경로: takeoff 중 초기화 7회 중 4회 성공, hover reset fallback 3회 중 2회 성공. 표본이 작아 경로별 우열은 판정하지 않음.
- 초기 alignment scale/bias: 성공과 실패 양쪽에 넓게 분포했고 단일 threshold로 결과를 분류하지 못함. 초기 accel bias norm 약 `1.66 m/s²` 뒤에도 성공한 사례가 있음.
- 판정: 성공 재현은 확인했으나 60% 성공률이므로 VIO 안정성 실패. PX4 handover 금지 유지.
- 로그/metrics: `artifacts/clean_repeats/20260716T183732KST/`, `artifacts/clean_repeats/20260716T184302KST/`
- rosbag: 없음. 디스크 여유 9 GB에서 10개 full bag을 만들지 않기 위해 로그와 JSON만 보존.
- 판단 영향: 환경 오염 제거만으로 완성되지 않으며, 다음에는 full bag 실패/성공 쌍으로 scale 과대추정과 position jump를 분석해야 함.

## 2026-07-16 18:41 KST — ENU/NED no-scale 평가 수정

- 목적: 자동 run JSON의 전체 trajectory vector RMSE `2.11 m`가 마지막 validation error와 불일치한 원인 확인
- 명령: 기준 성공 bag에 `ros2 run VIO_pkg vio_bag_metrics ...` 적용 전/후 비교
- 조건: VINS ROS ENU 위치와 PX4 NED 위치, scale fit 없음, 고정 ENU→NED 축 변환 뒤 yaw/translation만 fit
- 수정 파일: `src/VIO_pkg/VIO_pkg/vio_preflight_test.py`, `src/VIO_pkg/VIO_pkg/vio_bag_metrics.py`
- 결과: 기준 성공 bag 3D RMSE `1.036 m → 0.076 m`, 최대 오차 `3.368 m → 0.188 m`, final error `0.094 m → 0.062 m`
- 판정: 평가 좌표계 버그 수정 성공. estimator와 READY gate에는 영향 없음.
- 로그/metrics: `artifacts/live_preflight/20260716T0645KST/offline_metrics_current.json`, `offline_metrics_enu_to_ned.json`
- rosbag: `artifacts/live_preflight/20260716T0645KST/sensor_preflight_bag/`
- 판단 영향: corrected vector RMSE를 이후 반복 통계에 사용할 수 있게 됨. 첫 자동 run의 vector RMSE는 수정 전 값이므로 비교에서 제외.

## 2026-07-16 20:09~20:30 KST — zstd full-ground-truth clean 반복 6회

- 목적: 인접 성공/실패 full bag을 확보하고 초기화 window를 ground truth와 비교
- 명령: `RECORD_BAGS=1 STOP_ON_OUTCOME_CHANGE=1 ./tools/run_clean_preflight_repeats.sh 6`
- 조건: 기존 clean 검사 유지, `publish_visual_odometry=false`, 8개 센서/ground-truth 토픽 기록, run 종료 후 file-zstd 압축; recorder 준비 때문에 preflight 시작이 기존 반복보다 2초 늦음
- 수정 파일: `tools/run_clean_preflight_repeats.sh`
- 결과: 최종 `HOVER_READY` 6/6, path scale `1.0251~1.0369`(평균 `1.0295`), run 후 잔여 프로세스 0개. 각 bag `196~309 MB`.
- 제한: run05는 recovery 1회가 있었으므로 무복구 통과는 5/6. 시작 지연과 bag 기록 부하가 기존 10회와 달라 성공률을 직접 합산하지 않음.
- 판정: full bag 기록 자동화 성공 / 최종 실패 bag 미확보
- 로그/metrics/bag: `artifacts/clean_repeats/20260716T200914KST/`
- 판단 영향: 첫 frame/callback 시작 조건이 성공률에 영향을 줄 가능성이 생겼으며 독립 반복 검증이 필요

## 2026-07-16 20:31~20:36 KST — 인접 정상과 output-loss/recovery window 비교

- 목적: run04 정상 초기화와 run05 비정상 초기화의 실제 운동량 및 recovery 후 정확도 비교
- 명령: `ros2 run VIO_pkg vio_init_window_metrics ...`, `ros2 run VIO_pkg vio_bag_metrics ... --last-contiguous-segment`
- 조건: `/clock`으로 bag 기록 시각을 VINS sim-time에 매핑, PX4 `VehicleOdometry` 사용, no-scale ENU→NED 평가
- 수정 파일: `vio_bag_metrics.py`, `vio_init_window_metrics.py`, `setup.py`, `package.xml`
- 결과:
  - run04 정상 window `41.032~41.984 s`: displacement `0.446 m`, path `0.450 m`, endpoint rotation `2.83°`
  - run05 비정상 window `55.804~59.496 s`: displacement `1.440 m`, path `1.783 m`, endpoint rotation `2.52°`, alignment accel bias norm 약 `3.56 m/s²`
  - run05 첫 validation: path scale `1.11`, vector error `2.74 m`, 이후 output loss
  - run05 recovery 후 마지막 연속 segment: 3D RMSE `0.072 m`, 최대 `0.243 m`, final `0.044 m`
- 판정: 운동 부족 가설 기각 / recovery는 최종 metric trajectory를 복구함 / 최종 실패 원인은 여전히 미확정
- 데이터: run04/run05 `init_window_ground_truth.json`, run05 `offline_metrics_last_segment.json`
- 판단 영향: 최종 outcome만으로 내부 실패를 숨기지 말고 recovery count와 segment별 metrics를 함께 봐야 함

## 2026-07-16 20:55~21:27 KST — 명시적 2초 시작 지연 무기록 clean 반복 10회

- 목적: full-bag batch의 recorder 준비 과정에서 생긴 2초 preflight 시작 지연을 bag 기록 부하와 분리해 성공률에 미치는 영향 확인
- 명령: `PREFLIGHT_START_DELAY_SECONDS=2 RECORD_BAGS=0 STOP_ON_OUTCOME_CHANGE=0 ./tools/run_clean_preflight_repeats.sh 10`
- 조건: VINS/relay 준비 뒤 preflight 노드 시작 전 2초 대기, bag 미기록, `publish_visual_odometry=false`, 기존 clean infrastructure 검사 유지
- 수정 파일: `tools/run_clean_preflight_repeats.sh`
- 결과: 최종 `HOVER_READY` 8/10, `FAILSAFE_HOVER` 1/10, `VINS_OUTPUT_LOST` 1/10. 그러나 성공 중 run01·07은 output loss 뒤 recovery 1회를 거쳤으므로 무복구 통과는 6/10으로 0초 조건과 동일했다.
- run04: path scale `1.0464`, 전체 RMSE `0.106 m`로 validation 통과 후 hover drift가 `0.30 m` 기준을 넘어 실패
- run08: recovery 2회, 전체 RMSE `0.329 m`, max step `0.396 m`; VINS 로그에 position jump `1.120 m`, `0.646 m`, `0.502 m`와 반복 dense Cholesky failure 기록
- recovery 성공 run: run01 전체 RMSE `1.165 m`, run07 전체 RMSE `1.683 m`; 최종 outcome만 보면 중간 corruption이 숨겨짐
- 인프라: 10회 모두 실행 전후 잔여 프로세스 0개, full bag 0개
- 판정: 2초 지연이 실패를 제거하거나 failure-free 성공률을 개선한다는 가설 기각. outcome 분포 변화 여부는 표본 10회로 미확정.
- 로그/metrics: `artifacts/clean_repeats/20260716T205529KST/`
- rosbag: 없음
- 판단 영향: 다음 테스트는 시작 지연 추가 반복보다 최종 실패 full ground-truth bag 확보를 우선함

## 2026-07-16 21:35~21:49 KST — 인접 full-bag 성공/validation-timeout 확보

- 목적: 최종 clean 실패 full ground-truth bag과 인접 성공 bag을 확보하고 scale 과대추정 timeout을 초기화 window와 비교
- 명령: `RECORD_BAGS=1 STOP_ON_OUTCOME_CHANGE=1 PREFLIGHT_START_DELAY_SECONDS=0 ./tools/run_clean_preflight_repeats.sh 10`, `ros2 run VIO_pkg vio_init_window_metrics ...`, `ros2 run VIO_pkg vio_bag_metrics ...`
- 조건: recorder 준비 2초 대기, 별도 preflight 지연 0초, file-zstd, `publish_visual_odometry=false`, clean infrastructure 검사
- 수정 파일: 소스/설정 수정 없음. 분석 산출물 `init_window_ground_truth.json`, `offline_metrics.json` 추가
- 결과: run01~03 `HOVER_READY`, run04 `VALIDATION_TIMEOUT`; 모두 recovery 0회. outcome 변경에서 조기 종료
- 성공 run03: live path scale `1.033213`, RMSE `0.0928 m`; accepted window `38.576~39.632 s`, PX4 displacement/path/rotation `0.532/0.539 m/1.83°`; 첫 비선형 position correction `0.0348 m`
- timeout run04: live path scale `1.050663`(상한 `1.05`보다 `0.000663` 큼), RMSE `0.0782 m`; relative-pose 후보 18개 중 SFM construct 실패 15회와 alignment 기각 2회, dense Cholesky failure 반복, accepted window `64.608~66.620 s`
- run04 accepted ground truth: displacement/path/rotation `1.087/1.088 m/0.71°`; 첫 비선형 position correction `0.436 m`. 운동 부족 가설은 이 window에서 기각
- offline 전체 평가: run03 RMSE/max `0.094/0.257 m`, run04 `0.096/0.331 m`. timeout run을 전체 trajectory corruption으로 판정하지 않음
- 판정: 최종 scale-timeout full bag 확보 성공 / SFM 실패·늦은 초기화와 scale 경계 초과의 인과는 미확정 / output-loss 최종 실패 bag은 아직 없음
- 로그/metrics/bag: `artifacts/clean_repeats/20260716T213537KST/`
- 판단 영향: 다음은 인접 run03/run04의 초기 SFM 차이와 단일-source replay 재현성을 우선 확인

## 2026-07-16 21:49~22:02 KST — run03/run04 SFM 비교 및 run04 단일-source replay 2회

- 목적: live run04의 SFM 실패 단계와 늦은 초기화가 저장 relay 단일 source에서 재현되는지 확인
- 명령: `colcon build --packages-select vins_fusion_ros2 --symlink-install`, `ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py`, `ros2 bag play .../20260716T214448KST_run04/sensor_ground_truth_bag --clock --rate 2.0 --disable-keyboard-controls`
- 조건: 새 relay를 띄우지 않고 bag의 `/vio/camera/vins_image_raw`만 사용, `publish_visual_odometry=false`, 동일 run04 file-zstd bag을 독립 VINS 프로세스로 2회 재생
- 수정 파일: `vins_fusion_ros2/vins/src/initial/initial_sfm.cpp` 진단 로그만 추가. SFM 계산과 acceptance threshold는 변경하지 않음.
- live 비교: run03은 relative-pose 후보 1개로 `39.632 s`에 초기화. run04는 후보 18개 중 SFM construct 실패 15회, 음수 alignment scale 기각 2회 뒤 `66.620 s`에 초기화. dense Cholesky failure 메시지는 21개.
- replay 결과: 2회 모두 같은 결과. 첫 `40.688 s` 후보는 triangulated `257/409`, 비유한 observation 0, 음수 depth 86, BA final cost `0.08226`, unsuccessful step 8회로 실패. 두 번째 `41.240 s` 후보는 `265/398`, 비유한/음수 depth 0, BA final cost `0.003167`로 수렴하고 초기화 성공.
- live/replay 차이: 첫 후보부터 live feature/correspondence `407/61`, replay `409/62`였고 relative translation도 달랐다. live의 반복 SFM 실패, 늦은 초기화, path-scale `1.050663` 경계 초과는 replay에서 재현되지 않음.
- 판정: SFM 진단 및 단일-source 재생 절차 성공 / live failure sequence 재현 실패. 고정 bag과 estimator가 실제 소비한 frame/feature window는 동일하다고 간주할 수 없음.
- 검증: `vins_fusion_ros2` 빌드 성공(기존 VLA warning만 발생), `git diff --check` 통과, 재생 종료 후 VINS/bag play 잔여 프로세스 0개, replay가 만든 임시 해제 `.db3` 삭제 확인
- 로그: `artifacts/replay_sfm_run04/20260716T2158KST/vins_replay_stdout.log`, `bag_play.log`
- rosbag: `artifacts/clean_repeats/20260716T213537KST/20260716T214448KST_run04/sensor_ground_truth_bag/`
- 판단 영향: 다음에는 live/replay의 실제 camera callback timestamp/sequence와 OpenCV feature/RANSAC 상태를 고정·기록한 뒤 같은 bag을 비교해야 함

## 2026-07-16 22:19~22:40 KST — frame trace 원인 분리와 deterministic replay

- 목적: live/replay SFM window 차이를 bag 누락, reset timing, feature/RANSAC 상태, estimator frame 선택으로 분리
- 변경: camera sequence/image·feature fingerprint, correspondence/RNG trace, `vio_frame_trace_compare`, replay 전용 in-process reset, deterministic all-frame enqueue 추가
- live full-bag 결과: `artifacts/clean_repeats/20260716T221956KST/20260716T222001KST_run01/`; 최종 `VALIDATION_TIMEOUT`, recovery 2회, path scale `1.044909`, RMSE `1.159 m`, max step `1.288 m`; accel-bias reset 1회와 position jump `3.282/0.640/0.527 m`
- bag 완전성: overlap `30.036~239.112 s`에서 callback/bag relay 4,161/4,161 일치, sequence gap·timestamp 역행·bag 누락 0개
- 초기화 ground truth: accepted 네 window displacement `0.218/0.905/0.875/0.528 m`, path `1.487/0.906/0.877/0.798 m`; 운동 부족으로 단정하지 않음
- helper reset replay: recorded reset이 live보다 3 frame 늦어 reset 후 feature fingerprint 0/161 일치, live failure sequence 재현 실패
- in-process reset replay: live reset 뒤 160/160 feature fingerprint와 `56.572 s` BA 실패, `57.116 s` initialization/RANSAC fingerprint 재현
- 원인 확인: 이후 queue-empty 판정이 `57.216 s`부터 달라져 estimator 처리 sequence와 bias reset 시점이 분기
- deterministic enqueue 검증: 같은 bag 2회 replay의 공통 805 camera timestamp/feature fingerprint와 initialization/RANSAC/reset event 14개가 수치까지 일치
- 빌드/정적 검증: `vins_fusion_ros2`, `VIO_pkg` colcon build 성공, Python `py_compile`, `git diff --check` 통과
- 판정: replay 입력 비결정성의 구체 원인 확인 및 고정 성공. live 안정성 개선 여부는 미검증이며 `publish_visual_odometry=false` 유지
- 다음 영향: 변경된 all-frame enqueue를 clean live 10회 반복하고 backlog와 무복구 통과율을 기존 6/10과 비교

## 2026-07-16 23:47~23:50 KST — all-frame enqueue live backlog 회귀 확인

- 목적: deterministic all-frame enqueue의 clean live 성공률과 processing backlog를 최초 측정
- 명령: `RECORD_BAGS=0 STOP_ON_OUTCOME_CHANGE=0 PREFLIGHT_START_DELAY_SECONDS=0 ./tools/run_clean_preflight_repeats.sh 10`
- 조건: LiDAR ON, 약 20 Hz relay frame 전부 estimator enqueue, `publish_visual_odometry=false`
- 수정 파일: `estimator.cpp`, `run_clean_preflight_repeats.sh` queue 계측 추가
- 결과: run01 `VALIDATION_TIMEOUT`, path scale `5.21329`, max VIO step `38.390 m`; feature queue 최대 depth 17, 최대 span `0.816 s`
- 실행기 문제: Docker root 소유 `metrics.json` 직접 쓰기에서 `PermissionError`가 발생해 batch가 1회 후 중단됨. 임시 파일 작성 후 `os.replace()`하는 원자 교체로 수정
- 판정: all-frame 입력 정책 live 회귀 확인, 9회 추가 반복 중단. replay 결정성과 live 처리 가능성은 별도 문제
- 로그/metrics: `artifacts/clean_repeats/20260716T234716KST/`
- 판단 영향: queue 상태와 무관한 고정 2:1 callback sequence downsample로 변경

## 2026-07-16 23:50~23:53 KST — 고정 2:1 downsample replay 2회

- 목적: all-frame backlog 없이 동일 bag estimator 입력/event 반복성 유지 여부 확인
- 명령: `ros2 launch vins_fusion_ros2 vins_fusion_ros2.launch.py replay_reset_events:=true`, 동일 failure bag `ros2 bag play --clock --rate 2.0` 2회
- 조건: callback count 짝수 frame만 enqueue, 저장 relay 단일 source, image callback group recorded reset
- 수정 파일: `vins_fusion_ros2/vins/src/estimator/estimator.cpp`
- 결과: 공통 선택 camera timestamp/feature fingerprint 동일; RANSAC event 11개, initialization 3개, reset 2개 동일. queue p95 span 두 run `0 s`; 최대는 run1 depth 2/span `0.1 s`, run2 depth 1/span `0 s`
- 제한: wall-time timeout 경계 때문에 최종 frame count는 964/952로 달랐으나 공통 알고리즘 입력/event는 일치
- 판정: 성공. deterministic 입력 sequence와 sustainable rate 동시 확보
- 로그: `artifacts/replay_frame_trace/20260716T2355KST_fixed_downsample_1/`, `20260716T2355KST_fixed_downsample_2/`

## 2026-07-16 23:54~2026-07-17 00:21 KST — 고정 downsample, LiDAR ON clean live 10회

- 목적: 고정 10 Hz 입력 정책의 무복구 성공률, 실패 종류, queue backlog를 기존 6/10과 비교
- 명령: `RECORD_BAGS=0 STOP_ON_OUTCOME_CHANGE=0 PREFLIGHT_START_DELAY_SECONDS=0 ./tools/run_clean_preflight_repeats.sh 10`
- 조건: `use_range=1`, fixed extrinsic, fixed `td=0`, `publish_visual_odometry=false`, run 전후 clean infrastructure 검사
- 결과: `HOVER_READY` 6, `VALIDATION_TIMEOUT` 2, `FAILSAFE_HOVER` 2; 무복구 통과 6/10, recovery 총 1회
- 무복구 성공 6회: path scale `1.0096~1.0411`(평균 `1.0255`), RMSE `0.0818~0.1082 m`(평균 `0.0916 m`)
- 실패: run01 output loss recovery 뒤 hover drift, run02 validation 동적 step/speed timeout, run08 path scale `1.05132` 경계 초과, run10 READY 직후 hover drift
- queue: 모든 run p95 span `0 s`; 대부분 최대 `0~0.1 s`. run05 startup에서 depth 12/span `1.104 s` spike 뒤 즉시 drain됐고 최종 무복구 통과
- 판정: replay 결정성과 backlog는 개선됐으나 live 무복구 성공률은 기존 6/10과 동일. VIO 안정성 미해결
- 로그/metrics: `artifacts/clean_repeats/20260716T235400KST/`

## 2026-07-17 00:22~00:54 KST — LiDAR range factor OFF clean live 10회

- 목적: range factor가 scale/output-loss/hover drift 실패의 근본 원인인지 단일 변수로 비교
- 명령: `EXPERIMENT_LABEL=range_factor_off RECORD_BAGS=0 STOP_ON_OUTCOME_CHANGE=0 PREFLIGHT_START_DELAY_SECONDS=0 ./tools/run_clean_preflight_repeats.sh 10`
- 조건: 센서 publisher/stream 검사는 동일, estimator `use_range=0`만 변경, active config snapshot/hash 보존, `publish_visual_odometry=false`
- 결과: 최종 `HOVER_READY` 9, `VALIDATION_TIMEOUT` 1; recovery 3회, 무복구 통과 6/10
- 무복구 성공 6회: path scale `1.0245~1.0370`(평균 `1.0306`), RMSE `0.0855~0.1160 m`(평균 `0.0975 m`)
- 비정상 사례: run02 전체 RMSE `1.668 m`, run06 `1.114 m`, run09 max step `0.324 m` 뒤 recovery; run05 path scale `1.05844` timeout
- queue: 모든 run p95 span `0 s`; startup 일시 spike 외 지속 backlog 없음
- 판정: 최종 통과율만 9/10으로 늘었지만 무복구 성공률은 ON과 같은 6/10. range factor 단독 원인 가설 기각, `use_range=1` 복원
- 로그/metrics/config: `artifacts/clean_repeats/20260717T002253KST/summary.tsv`, 각 run `metrics.json`, `active_vins_config.yaml`, `batch_config.tsv`

## 2026-07-17 00:54~00:55 KST — 20회 초기화 진단 1차 집계

- 목적: 건강한 무복구 성공과 실패/recovery를 구분할 초기 SFM/alignment/nonlinear 신호 탐색
- 명령: 두 batch의 `vio_vins.log`에서 accepted initialization 직전 diagnostics와 초기 3회 nonlinear update를 read-only 집계
- 확인됨: 실패/recovery 일부에서 작은 alignment scale, 낮은 physical condition, 큰 초기 accel bias 또는 초기 3회 bias 증가가 함께 나타남
- 반례: 건강한 run에도 초기 bias norm 약 `0.513 m/s²`, 초기 3회 최대 bias norm 약 `0.912 m/s²` 사례가 있어 단일 threshold로 완전 분리되지 않음
- 판정: 상관 후보는 있으나 원인/gate 확정 전. SFM depth/cost, physical condition, observability, bias 변화를 복합 비교해야 함
- 데이터: 위 ON/OFF 두 batch의 `vio_vins.log`; 별도 알고리즘 변경 없음

## 2026-07-17 01:10~01:35 KST — accepted initialization 72개 offline 정량화

- 목적: 정상 초기화를 거짓 거부하지 않으면서 명확히 나쁜 초기 후보를 찾음
- 명령: `python3 tools/analyze_initialization_quality.py artifacts/clean_repeats/*`
- 조건: 기존 모든 persistent clean-repeat log; 후속 `reset state successfully` 유무는 분석용 label이며 ground truth가 아님
- 수정 파일: `tools/analyze_initialization_quality.py`
- 결과: 72 accepted initialization, reset label 23, survived 49. 초기 accel bias `>2.5 m/s²`는 reset 5, survived 0. scale/condition 복합 임계값은 survived 사례 7개 이상을 거짓 거부해 폐기.
- 판정: 기존 failure bias limit을 initialization 전에 앞당기는 것은 보수적 gate로 채택 / 임의 scale gate는 기각
- 데이터: `/tmp/all_init_quality.json`은 임시 분석 산출물이며 원본은 각 batch `vio_vins.log`

## 2026-07-17 01:35~01:50 KST — initialization bias gate 첫 replay와 reset 방식 폐기

- 목적: bad initial solve를 output 전에 거부하고 다음 window로 재초기화
- 조건: full bag `20260716T202148KST_run05`, fixed 2:1 input, recorded reset in image callback group, 2배속
- 첫 구현: gate 뒤 `resetStateLocked()` 전체 reset
- 결과: bias `2.74229 m/s²` 후보 거부는 성공했으나 다음 IMU 입력에서 `Abnormal IMU timestamp jump` 뒤 node exit `-11` 발생
- 판정: 전체 reset retry 방식 실패 및 폐기
- 로그: `artifacts/replay_initialization_gate/20260717T_gate_run05/`

## 2026-07-17 01:50~02:00 KST — pre-solve rollback+next-window retry replay

- 목적: input epoch를 깨지 않고 rejected candidate의 optimizer mutation만 되돌림
- 변경: state/depth/extrinsic/time-delay snapshot 복원, marginalization prior 폐기, INITIAL window slide
- 결과: bias `2.74229` 후보 거부 뒤 다음 window에서 bias 약 `0.387 m/s²` 후보 수락. SIGSEGV 없음. 이후 `99.032 s` reset은 recorded preflight event임을 확인.
- 추가 결과: recovery 뒤 건강한 초기화 이후 약 25초 뒤 `1.006~1.122 m` position jump가 재현되어 초기화 bias gate와 후속 corruption을 분리함.
- 로그: `artifacts/replay_initialization_gate/20260717T_gate_rollback_run05/`
- 판정: rollback retry 성공 / 뒤늦은 jump 원인 미확정

## 2026-07-17 02:13~02:43 KST — bias gate clean live 10회 요청, 유효 9회

- 목적: 정상 초기화 false reject와 무복구 outcome 측정
- 명령: `EXPERIMENT_LABEL=initialization_health_gate RECORD_BAGS=0 STOP_ON_OUTCOME_CHANGE=0 ./tools/run_clean_preflight_repeats.sh 10`
- 조건: LiDAR ON, fixed 10 Hz, `publish_visual_odometry=false`, bag 미기록
- 결과: 유효 9회 중 `HOVER_READY 6`, `VINS_OUTPUT_LOST 2`, `VALIDATION_TIMEOUT 1`; run09 `INFRA_SENSOR_CHECK_FAILED` 제외. 정상 6회 rejection 0. run01 bad bias 후보 1개 거부.
- run01: recovery 2, RMSE `1.389 m`, max step `1.863 m`; accepted init 뒤 jump `0.734/0.683/0.643/0.748 m`, solver failure/bias-limit 선행 없음.
- run08: 첫 initialization 다음 update에서 accel bias `2.59453 m/s²`, 후속 recovery에서 `0.968889 m` jump와 scale corruption.
- 판정: 정상 false rejection 0 / bias gate 단독으로 무복구 성공률 개선 확인 못함 / probation 및 SFM 후속 gate 필요
- 로그: `artifacts/clean_repeats/20260717T021346KST/`

## 2026-07-17 02:44~02:49 KST — post-BA cheirality와 3-update probation replay

- 목적: SFM 뒤쪽 geometry 거부와 초기 update 실패의 output 격리 검증
- 변경: post-BA nonfinite/nonpositive depth gate `5%`, first 2 nonlinear outputs 보류, 3번째 통과 뒤 publish
- recovery bag 결과: 정상 후보 post-BA `0/1613`, probation 통과. bad bias 후보 `21/1035=2.03%`는 cheirality가 아니라 bias gate로 거부, 다음 후보 `2/1036=0.19%` 수락. 뒤늦은 jump는 계속 재현.
- 정상 bag 결과: `20260716T201835KST_run04` post-BA `0/1601`, probation 통과, bias/jump failure 없음.
- 판정: replay smoke 성공 / live 반복 필요
- 로그: `artifacts/replay_initialization_gate/20260717T_sfm_probation_run05/`, `20260717T_sfm_probation_success_run04/`

## 2026-07-17 02:49~02:54 KST — post-BA/probation full-bag live batch 중단 체크포인트

- 목적: 최종 gate live 반복과 final output-loss full bag 동시 확보
- 명령: `EXPERIMENT_LABEL=sfm_cheirality_probation RECORD_BAGS=1 STOP_ON_OUTCOME_CHANGE=1 ./tools/run_clean_preflight_repeats.sh 10`
- 확정 run01: `HOVER_READY`, recovery 0, path scale `1.033832`, RMSE `0.081284 m`, max step `0.072506 m`; bias/SFM rejection 0, probation pass 1, queue max depth/span `6/0.504 s`, p95 `0.1 s`.
- bag: `artifacts/clean_repeats/20260717T024957KST/20260717T025003KST_run01/sensor_ground_truth_bag/`, file-zstd 약 `203 MB`
- run02: 사용자 중단 시점 미완료. runtime cleanup 후 결과에서 제외.
- 판정: 정상 live smoke 성공 1회 / 10회 반복과 final output-loss bag 확보 미완료

## 2026-07-17 07:14~10:36 KST — 무복구 빈도 개선 및 PX4 reset/failsafe 최종 검증

- 목적: post-ready 무복구 성공률을 100%에 가깝게 올리고, rare reset을 handover gate와 실제 PX4 EKF2/failsafe가 흡수하는지 검증
- 주요 변경: 3-frame feature factors, LM nonlinear solver, low-geometry correction `0.15 m`, estimator/bridge pose step `0.15 m`, implied speed/timestamp gate, validation timeout `45 s`, PX4 `reset_counter`, actual PX4 fault monitor
- baseline 및 feature 밀도 실험: `20260717T071402KST` 4/5, `20260717T073115KST` 4/5(dense Cholesky 회귀), `20260717T074557KST` terminal 5/5·무복구 4/5. feature `250/min_dist 12`와 `min_dist 12`는 폐기하고 `200/15` 복원.
- 3-frame 10회: `20260717T081916KST` terminal 10/10, post-ready 무복구 9/10. 2-frame smoke `20260717T085022KST`는 최대 step `0.232 m`를 노출해 중단·폐기.
- 0.10 m geometry 경계 최종 회귀 `20260717T093445KST`: 완결 2회 중 1 `HOVER_READY`, 1 `VALIDATION_TIMEOUT`; 부분 run 제외. 0.10 m 바로 위 correction reset cascade를 확인.
- geometry `0.15 m` smoke `20260717T094350KST`: 3/3 `HOVER_READY`, recovery 0, RMSE `0.083~0.098 m`. 후속 batch `20260717T095120KST` 첫 run에서 Dogleg position jump `0.169 m` 뒤 반복 reset/output loss 확인.
- LM smoke `20260717T095826KST`: 3/3 terminal, post-ready recovery 0. 한 run의 readiness 전 jump/retry는 최종 승인 전에 격리.
- 30 s validation 경계 실험 `20260717T100543KST`: 완결 3회 중 2 terminal pass; run03은 늦은 건강한 init 후 RMSE `0.061 m`, scale `1.039`였으나 8 s hold 전에 30 s timeout. 45 s로 확대.
- 최종 7회 `20260717T101452KST`: 7/7 `HOVER_READY`, post-ready recovery 0. 앞 LM smoke와 결합한 동일 후보 10회는 terminal 10/10, post-ready recovery 10/10.
- 최종 10회 정확도: scale min/mean/max `1.0126/1.0274/1.0392`, RMSE `0.0818/0.0913/0.1053 m`, max VIO step `0.0669/0.0711/0.0758 m`; post-ready geometry rejection/solver failure/position jump 0.
- PX4 GPS fallback: `PX4_VIO_FAULT_TEST=1`, batch `20260717T103025KST`; PX4 EV fusion 확인 뒤 reset, gate close, fault 이후 EV 0, GPS fusion active, XY valid, max local step `0.000815 m`, pass.
- PX4 GNSS denied: `PX4_DISABLE_GPS_BEFORE_FAULT=1`, batch `20260717T103319KST`; EV fusion 확인 뒤 GPS disable/reset, fault 이후 EV 0, GPS fusion false, failsafe true, nav state 12, `Failsafe: blind land`, pass.
- rosbag: 이번 최종 반복은 없음(`RECORD_BAGS=0`). 각 batch에 source/config snapshot, logs, metrics 보존.
- 판정 영향: SITL 무복구 10/10 및 두 PX4 rare-reset 경로 확인. 실기체/HIL은 별도 검증 전 미승인.

## 2026-07-17 10:41~11:42 KST — fail-closed 재-handover와 nonlinear quarantine 기각

- handover 변경: production 기본 `allow_rehandover_after_fault=false`. 열린 gate의 stale/jump/reset 이후 fault latch가 설정되고 후속 `/vio/ready`를 무시하며 bridge restart를 요구한다. opt-in test만 재연결 허용.
- runner 변경: PX4 fault JSON 누락/파손/`passed!=true`를 `PX4_FAULT_TEST_FAILED` primary outcome 및 exit 1로 승격. 정상 fault reason도 primary metrics에 병합.
- GPS fallback `20260717T104151KST`: fault assertion pass, fault 후 EV 0, GPS active, max local step `0.000855 m`. 다만 pre-handover VINS recovery 1회와 전체 RMSE `1.294 m`를 관측해 근본 빈도 미완료 증거로 기록.
- GNSS denied `20260717T104525KST`: recovery 0, RMSE `0.070 m`; fault 후 EV 0, GPS false, failsafe true, nav state 12, blind land pass.
- max feature 220 smoke `20260717T104931KST`: 첫 완결 run이 geometry reject와 scale `1.0562` timeout. 250과 마찬가지로 폐기하고 200 복원.
- bounded rollback smoke `20260717T105601KST`: 3/3 pass. 후속 `20260717T110306KST` run08에서 quarantine은 단발 mutation을 넘겼지만 persistent 3회 jump 두 구간은 full reset으로 전환되어 final timeout.
- outlier-cleansing rollback `20260717T113353KST` run01: 대량 feature 제거와 prediction drift로 max VIO output step `8.7108 m`, scale `1.4126`; unsafe regression으로 즉시 중단·전량 원복. partial 후속 run은 분모 제외.
- 원복 확인 `20260717T113946KST`: recovery 0 `HOVER_READY`, scale `1.0243`, RMSE `0.0913 m`, max step `0.0715 m`, solver/geometry/jump event 0.
- 판정: fail-closed handover/runner는 채택. nonlinear quarantine과 220-feature 실험은 기각. estimator는 검증된 full-reset guard 유지.
