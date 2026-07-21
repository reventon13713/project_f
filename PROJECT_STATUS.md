# Project Status

마지막 업데이트: 2026-07-22 KST

## 현재 판정

### 2026-07-22 — 7·8단계 QGC 글로벌 mission 지속·rosbag/ULog GT 판정 완료

- `vio_qgc_mission_test`는 QGroundControl에서 그대로 열 수 있는 `qgc_stage7.plan`을 만들고 표준 MAVLink mission protocol로 PX4에 업로드한다. mission item 0은 검증된 VIO envelope의 `DO_CHANGE_SPEED=0.55 m/s`, waypoint 1~7은 초기 heading 고정·수락 반경 `0.50 m`의 WGS84 글로벌 항목이다.
- GPS+EV 준비 뒤 `AUTO.MISSION`을 시작하고 sequence 5 진입 시 Gazebo GPS sensor를 실제 `off`했다. 최종 run `artifacts/clean_repeats/20260722T005537KST/20260722T005543KST_run01`은 sequence 5·6·7을 모두 완료했고 GPS age `17.9 s`, EV age `0.10 s`, alignment/handover true였다.
- `vio_mission_ground_truth_metrics`는 evidence rosbag의 `/vio/global_position`, PX4 global, 독립 Gazebo GT, GPS/event stream과 같은 실행 ULog을 mission start/dropout/result 시간창으로 교차 판정한다. acceptance는 post-dropout path `≥8 m`, RMSE/max `≤1.0/1.5 m`, waypoint `≤1.5 m`, step `≤0.5 m`, receipt gap `≤0.60 s`다.
- 최종 GT/PX4/VIO path는 `9.282/8.786/8.638 m`; PX4/VIO 대 GT RMSE는 `0.461/0.418 m`, max는 `0.546/0.534 m`, GT waypoint horizontal max는 `0.734 m`였다. PX4 지도 max step/gap은 `0.214 m/0.115 s`, VIO global은 `0.079 m/0.559 s`였다.
- ULog은 mission finished, GPS aid false, EV pos/vel 실제 fused `177/178`·rejection 0, post-stale EV flags 17 sample을 확인했다. GNSS 제거 시 global reset counter가 1회 변했지만 실제 최대 global step은 `0.214 m`여서 지도 연속성 기준을 통과했다.
- 원 판정의 유일한 실패는 rosbag receipt gap `0.559 s > 0.50 s`였고 bridge는 latch되지 않았으며 종료 EV age도 `0.10 s`였다. recorder scheduling 한도를 `0.60 s`로 명시한 최종 계약으로 같은 보존 bag/ULog을 재평가했고 원 JSON도 `mission_ground_truth_metrics.pre_gap_adjustment.json`에 보존했다.

### 2026-07-21 — 6단계 실제 Gazebo GPS dropout·VIO 지속 비행 완료

- `vio_gps_control` Gazebo world plugin은 project-owned sentinel 명령으로 모든 Gazebo `gps` sensor의 `SetActive(false)`를 실행한다. PX4 fusion parameter만 끄는 방식이 아니며, evaluator가 `/fmu/out/vehicle_gps_position` stream 정지를 별도로 확인한다.
- `vio_gps_dropout_test`는 stage-5 GPS+EV 정상 fusion 근거 뒤 alignment/handover/EV input을 gate하고 dropout을 주입한다. 이후 GPS stream stale, PX4 native GPS aid 해제, EV position/velocity aid와 실제 EV `time_last_fuse`, armed/nav-state/global-position 지속, drift/failsafe를 fail-closed 판정한다.
- clean run `artifacts/clean_repeats/20260721T232113KST/20260721T232119KST_run01`은 `HOVER_READY`와 `gps_dropout_test.passed=true`로 통과했다. GPS 마지막 sample age는 `10.0 s`, dropout 뒤 EV-only 관찰은 `8.0 s`, EV input은 120개, EV fused poll은 63개였다.
- 종료 시 native flags는 `cs_gps=false`, `cs_ev_pos=true`, `cs_ev_vel=true`, `cs_inertial_dead_reckoning=false`였다. 기체는 armed와 Offboard nav state 14를 유지했고 새 failsafe/nav-state 변화가 없었으며 hover drift 최대는 `0.0488 m`였다.
- stage-6 runner는 current propagated aligned pose를 PX4 receipt timestamp로 명시 전달하고 EV freshness를 `0.50 s`, `EKF2_NOAID_TOUT=10 s`로 설정한다. 기본 bridge 계약은 기존 measurement timestamp/`0.20 s` 그대로라 다른 mode에 영향이 없다.
- 같은 batch의 run02는 run01 acceptance 확보 뒤 수동 종료한 partial이며 분모에서 제외한다. QGC global waypoint mission 및 지도 이동은 아직 수행하지 않았고 다음 7단계다.

### 2026-07-21 — 5단계 GPS+VIO 정상 fusion global 연속성 완료

- `vio_global_fusion_metrics`는 global alignment 뒤 rosbag에서 GPS 3-D fix, handover, PX4↔VIO WGS84 오차, 양쪽 trajectory step과 VIO output gap을 판정하고, 같은 실행의 PX4 ULog에서 GPS·EV position·EV velocity 동시 fusion, aid-source `fused`/rejection, `VehicleGlobalPosition` reset/step을 교차 확인한다.
- 고정 acceptance는 정렬 뒤 `≥10 s`, `≥50` pair, RMSE `≤0.50 m`, max/error-change/각 trajectory step `≤1.00/0.50/0.50 m`, VIO output gap `≤0.50 s`, ULog aid 실제 fusion과 global reset 0이다.
- Baylands clean run `20260721T202758KST/20260721T202803KST_run01`은 `HOVER_READY`, recovery 0으로 통과했다. 18.56초/177쌍의 RMSE/p95/max는 `0.1072/0.1171/0.1223 m`, VIO/PX4 max step은 `0.00470/0.00355 m`, VIO max gap은 `0.223 s`였다.
- 같은 run의 `px4.ulg`는 GPS+EV position+EV velocity 동시 fusion 25 sample, EV pos/vel 각 178 fused sample, GNSS position 392 fused sample, innovation rejection 0, fusion 뒤 global reset 0과 max step `0.00392 m`를 기록했다.
- 첫 attempt `20260721T202332KST/run01`은 융합·오차는 정상이었지만 post-metrics rosbag이 7.87초뿐이라 고정 10초 evidence window를 충족하지 못했다. threshold는 유지하고 stage-5 wrapper의 post-metrics record grace를 15초로 늘린 뒤 위 clean run에서 통과했다.
- GPS dropout은 아직 주입하지 않았다. 다음은 글로벌 mission에 결합할 실제 simulated GPS `off` injector를 구현하는 6단계다.

### 2026-07-21 — 4단계 aligned NED → PX4 EV 연결·GPS+EV fusion 완료

- `vio_bridge`는 legacy raw VINS 계약 `enu_flu`와 stage-4 정렬 계약 `ned_frd`를 명시적으로 구분한다. stage-4에서는 `/vio/aligned_odometry`와 `/vio/global_alignment_ready`를 입력으로 쓰며, 이미 NED/FRD인 position·velocity·orientation에 ENU/FLU 변환을 다시 적용하지 않는다.
- bridge는 global alignment가 latch-ready 된 뒤에만 10개 healthy sample을 요구해 `/fmu/in/vehicle_visual_odometry`를 발행한다. 기존 freshness, timestamp, position step/rate, quaternion/velocity fail-closed와 in-flight fault 뒤 restart 정책은 유지한다.
- `PX4_GPS_EV_FUSION_TEST=1`은 PX4 `EKF2_EV_CTRL=7`만 켜고 GPS는 유지한다. native `px4-listener estimator_status_flags`에서 `cs_gps`, `cs_ev_pos`, `cs_ev_vel`가 동시에 유지돼야 통과한다.
- Baylands `20260721T093911KST/run01`은 `HOVER_READY`, recovery 0, global alignment/handover true, EV input 13개 및 GPS/EV position/velocity fusion 동시 true로 통과했다. artifact는 `artifacts/clean_repeats/20260721T093911KST/20260721T093916KST_run01/`이다.
- 첫 stage-4 시도 `20260721T093446KST/run01`은 bridge 결함이 아니라 fusion gate의 pre-alignment timeout으로 EV input 0으로 끝났다. alignment 대기와 alignment 뒤 fusion timeout을 분리한 뒤 위 clean run에서 통과했다.
- 다음은 GPS 정상 중 PX4 global position과 `/vio/global_position`의 연속성·오차를 rosbag/ULog로 정량 판정하는 5단계다. GPS dropout은 아직 주입하지 않는다.

### 2026-07-21 — 3단계 `vio_global_position` 구현·Baylands 검증 완료

- `vio_global_position` 노드는 raw VINS ENU, PX4 georeferenced local NED, 유효 3-D `SensorGps`를 receipt-time freshness로 동기화해 단일 scale·yaw·translation을 고정한다. WGS84 local tangent/ECEF 변환 뒤 `/vio/global_position`을 `NavSatFix` ellipsoid altitude로 실시간 발행하며 `/vio/aligned_odometry`에는 같은 정렬의 NED pose/velocity를 발행한다. PX4 EV에는 아직 연결하지 않는다.
- 정렬은 `/vio/ready` 이전 current VINS epoch의 동기 sample만 축적하고, 최소 sample·XY/Z excitation·scale `0.8~1.2`·fit RMSE/max `0.25/0.50 m`를 모두 요구한다. GPS fix/accuracy와 PX4 global reference가 유효하지 않으면 sample을 받지 않는다.
- 정렬 뒤 VIO output gap, timestamp 역행, `0.50 m` pose discontinuity 또는 새 readiness generation은 fail-closed latch다. 새 arbitrary frame을 비행 중 자동으로 기존 global trajectory에 붙이지 않으며 명시적 node restart만 허용한다.
- Baylands batch `20260721T084626KST/run01`은 EV publisher 0 상태에서 `HOVER_READY`와 기존 독립 GT acceptance를 통과했다. global lock은 474 sample, scale `0.97725`, yaw `-0.16°`, local fit RMSE/max `0.056/0.111 m`였다.
- 같은 rosbag의 `/vio/global_position` 68개를 PX4 fused `VehicleGlobalPosition`과 비교한 WGS84 3-D RMSE/p95/max는 `0.0387/0.0661/0.0699 m`로 고정 acceptance `0.50/1.00 m`를 통과했다. artifact는 `artifacts/clean_repeats/20260721T084626KST/20260721T084630KST_run01/`이다.
- 합성 Sim(3)/WGS84 수학 및 lock→새 generation latch ROS 회귀, Python/bash 정적 검사, Docker ROS 2 build를 통과했다. 다음 작업은 aligned NED orientation/velocity 계약을 `vio_bridge` input으로 연결하고 GPS 정상 중 PX4 GPS+EV fusion을 검증하는 4단계다.

### 2026-07-21 — 2단계 Baylands VIO 상대 waypoint·독립 ground truth 검증 완료

- `ground_truth_waypoint_test` 모드는 기존 cross를 VINS 초기화/8초 건강성 검증에 먼저 사용하고, VIO가 준비된 뒤 별도의 12-waypoint XYZ cross를 비행한다. scored mission의 시작·도달 event만 rosbag에 기록하므로 초기화 중 무출력/새 frame을 정확도 궤적에 섞지 않는다.
- `vio_ground_truth_metrics`는 raw VINS `/odometry`, 독립 Gazebo P3D `/vio/ground_truth/odometry`, PX4 local pose, waypoint event를 동기화한다. VIO↔GT에는 yaw/translation만 fit하고 scale은 fit하지 않는다. acceptance는 trajectory RMSE `≤0.30 m`, trajectory max `≤0.50 m`, GT waypoint reach max `≤0.50 m`, waypoint VIO↔GT max `≤0.50 m`로 고정했다.
- 최종 자동 clean batch `20260721T080015KST`는 GT acceptance `2/2`였다. 두 run의 scored trajectory RMSE는 `0.0413~0.0423 m`, max는 `0.1192~0.1309 m`, GT waypoint reach max는 `0.2465~0.3087 m`, waypoint VIO↔GT max는 `0.0757~0.0915 m`였다. 각 run은 12/12 waypoint와 VIO↔GT 동기 sample `345~347`개를 포함한다.
- 앞선 건강한 scored run `20260721T075549KST`도 평가기 sampling threshold 수정 후 같은 bag 재판정에서 RMSE/max/GT reach/VIO-GT waypoint max `0.0472/0.1045/0.2656/0.1041 m`로 통과했다. 첫 `20260721T075206KST` 시도는 initialization cross 자체를 scored mission으로 잘못 사용해 공통 XY VIO가 부족했으므로 무효다.
- 최종 batch run01은 초기화 recovery 1회 뒤 scored mission을 통과했지만 mission 직후 hover drift에서 `FAILSAFE_HOVER`, run02는 recovery 없이 `HOVER_READY`였다. scored accuracy `2/2`와 terminal stability `1/2`를 구분하며 VIO 무고장률 100%를 주장하지 않는다.
- GPS는 정상 유지했고 PX4 external-vision publisher는 0개였다. GPS/EV fusion/global conversion은 변경하지 않았다. 이 시점의 다음 작업은 `vio_global_position`이었고 현재 위 3단계에서 완료됐다.
- 초기화 운동은 pre-arm stationary `5 s`, reset 뒤 stationary `3 s`, initial heading 유지, `0.55 m/s`의 X/Y `±1.2 m` 및 Z `±1.0 m` 부드러운 왕복으로 구성된다. reset 뒤 stationary hold는 `post_reset_warmup_seconds` 파라미터로 명시했다.

### 2026-07-21 — 1단계 realistic outdoor world 통합 완료

- 공개 Open Robotics Gazebo model database의 `baylands`를 후속 시험 환경으로 선정했다. 원본 URL과 SHA-256을 `tools/install_baylands_world.sh`에 고정했으며 원본 3개 DAE와 도로·포장·식생·구조물 자연 texture를 그대로 사용한다.
- Gazebo Classic에서 원본 triangle-mesh collision의 긴 초기화를 피하기 위해 visual은 보존하고 500 m 평면 collision만 쓰는 `vio_baylands` 래퍼를 추가했다. 첫 smoke에서 잘못 켠 `collide_without_contact` 때문에 기체가 낙하한 것을 확인했고, 해당 설정을 제거한 뒤 ground truth `z=0.0545 m`로 안정 정지함을 재검증했다.
- PX4 v1.14.4 `iris_fpv_cam`에서 `/clock`, `/vio/camera/image_raw`, `/vio/imu`, `/vio/range`, `/vio/ground_truth/odometry`, `/fmu/out/vehicle_gps_position`을 확인했다. camera frame은 320×240 RGB8이고 자연 식생이 렌더링됐으며 Shi-Tomasi corner 102개가 검출됐다.
- `spherical_coordinates`는 upstream Baylands 기준 `37.413534, -121.996561, 1.3 m`를 유지한다. spawn GPS는 `37.4135428, -121.9965495, 1.345 m`로 약 `(N,E)=(0.98,1.01) m` spawn offset 및 ground 높이와 일치했다.
- `vio_gazebo_demo.launch.py`와 반복 runner의 기본 world는 `vio_baylands`이고, runner는 실제 선택 world를 `batch_config.tsv`에 기록한다. 이전 환경은 `PX4_SITL_WORLD=vio_feature`로만 명시 선택한다. GPS/EV fusion/global conversion은 변경하지 않았다.
- 1단계 판정은 완료다. 후속 2단계 상대좌표 waypoint와 독립 ground-truth 검증도 완료됐다.

### 2026-07-21 — 사용자 확정 운용 목표와 다음 개발 순서

- VIO 알고리즘 품질 개선과 기존 VIO-fault→GPS fallback/GNSS-denied 착륙 시험은 다음 개발의 우선순위가 아니다. 기존 안전 근거와 제한을 지우지는 않지만, 사용자는 현재 VIO 후보를 기준으로 GPS 상실 뒤 임무를 계속하는 기능을 구현·검증하기로 결정했다.
- 모든 후속 Gazebo 시험은 기존 `vio_feature.world`의 인위적 흑백 feature 배치 대신, 도로·건물·식생·울타리·차량 등 자연스러운 시각 특징을 가진 공개 outdoor/suburban world를 선정·통합한 뒤 수행한다. 선택 world는 현재 PX4 v1.14.4 Gazebo Classic, `iris_fpv_cam` 센서, 독립 ground truth와 `spherical_coordinates` 기반 QGroundControl 지도 위치를 함께 지원해야 한다.
- 첫 시험은 GPS·PX4 external-vision fusion·global conversion을 건드리지 않는 VIO 상대좌표 waypoint 정확도 검증이다. 기존 relative waypoint 비행에서 raw VINS `/odometry`와 독립 Gazebo ground truth를 비교한다. 목표는 전체 RMSE 약 `0.2~0.3 m`, 최대/waypoint 도달 오차 `0.5 m` 이하이며, 실제 acceptance는 새 world와 mission 크기를 확정한 뒤 기록한다.
- 첫 시험 통과 후 계획한 `vio_global_position` ROS 2 노드는 현재 구현·검증됐다. GPS 정상 구간의 VIO/GPS/PX4 동기화 sample로 scale·yaw·translation과 WGS84 origin을 고정하고, VIO를 WGS84 위도·경도·고도로 실시간 변환해 `/vio/global_position`을 발행한다.
- 이어서 변환 노드가 정렬한 local-NED VIO를 기존 `vio_bridge`를 통해 PX4 external vision에 공급한다. GPS 정상 중 GPS+VIO fusion과 PX4 global position 연속성을 확인한 뒤, 실제 simulated GPS `off`를 mission 중 주입한다.
- 최종 SITL acceptance는 QGroundControl의 글로벌 waypoint mission이 GPS 상실 뒤에도 mission mode·waypoint index를 유지하고, PX4 global position과 지도상의 기체 위치가 연속적으로 이동하며 남은 waypoint를 완료하는 것이다. rosbag/ULog에는 `/vio/global_position`, PX4 global position 및 독립 Gazebo ground truth를 함께 남긴다.
- 첫 GPS-dropout 성공 뒤의 유일한 확장 시험은 GPS 복구 후 GPS와 VIO의 재융합이다. `stuck`, `wrong`, 장시간 GPS 상실은 현재 범위에서 제외한다.

- 최종 소스는 SITL outlier threshold `4 px`, LM `12 iterations`, excitation `0.55 m/s`, dither 없음, `show_track=0`, frame fingerprint/queue trace 유지다. 16~19시 reset-epoch/frame-trace 실험 코드는 결과가 나빠 전부 원복했다.
- `show_track=0` 단독 배치 `20260717T164507KST`는 PX4 auto-disarm 인프라 1회를 제외한 유효 9/9가 `HOVER_READY`·recovery 0이었고 strict jump/solver-failure-free는 8/9였다. 시각화 제거는 유지하지만 근본 해결로 주장하지 않는다.
- frame trace까지 끈 `20260717T171548KST`는 유효 terminal 9/9이나 recovery 0이 6/9로 악화됐다. 고주파 진단 부하가 callback/solver scheduling을 바꾸는 증거이므로 trace-off 변경은 원복했다.
- reset 전후 feature epoch 혼입 가설은 실제 stale-frame drop을 관측했지만, atomic enqueue 후보 `20260717T175537KST`는 terminal 10/10·recovery 0 8/10·strict 5/10, nonblocking 후보 `20260717T182304KST`는 terminal 9/10이었다. 성공률 개선 근거가 없어 두 구현 모두 최종 소스에서 제거했다.

- SITL config의 평균 reprojection outlier 삭제 기준을 `3 → 4 px`로 조정한 후보는 clean 10회 `HOVER_READY 10/10`, recovery 0 `10/10`을 한 번 달성했다. 경로는 `artifacts/clean_repeats/20260717T133105KST/`다.
- 같은 4 px 후보의 후속 clean을 합치면 operational recovery 0은 `17/18`(94.4%)였다. 후속 3회 `20260717T140142KST`에서 late jump/recovery 뒤 hover-drift fail-closed 1건이 재현되어 실패확률 0 또는 안정적 10/10을 주장하지 않는다.
- 10회 배치 정확도는 path scale `1.0102~1.0465`(평균 `1.0261`), 3-D RMSE `0.0601~0.1025 m`(평균 `0.0874 m`), 최대 공개 VIO pose step `0.0669~0.1411 m`였다.
- readiness 전 position/geometry fault까지 모두 0인 strict run은 같은 10회 중 `6/10`이었다. 나머지는 GPS 보호 구간에서 reset/re-initialization 후 recovery counter 0으로 최종 승인됐다.
- 최종 코드에서 실제 PX4 external-vision 융합을 확인한 뒤 reset을 주입했다. GPS 가용 시 EV 융합을 중단하고 GPS offboard를 유지했으며, GNSS 차단 시 PX4 failsafe가 `blind land`/Descend로 전환했다.
- gate가 한 번 열린 뒤 fault가 발생하면 production 기본값은 latch-closed 상태가 되어 이후 `/vio/ready`를 무시한다. 새 VINS 좌표계의 비행 중 자동 재-handover에는 bridge 재시작이 필요하다.
- PX4 fault metrics가 없거나 `passed=true`가 아니면 반복 실행기는 primary outcome을 `PX4_FAULT_TEST_FAILED`로 바꾸고 비정상 종료한다.
- `publish_visual_odometry=false` 기본값은 유지한다. 반복 실행기만 시험 시 명시적으로 활성화한다.
- PX4 fault 경로는 현재 소스에서도 재검증했다. GPS fallback은 `20260717T135258KST`, GNSS denied Land/Descend는 `20260717T135529KST`이며 둘 다 assertion `passed=true`다.
- SITL 안전막과 operational 빈도는 크게 개선됐지만 실기체 센서 시간동기, 진동, 조도/텍스처, 좌표 정렬, 실제 PX4 파라미터 및 착륙 동작은 미검증이다. 이 상태를 실기체 자유비행 승인으로 해석하지 않는다.

## 이번 세션에서 확인됨

- 동일 failure bag의 첫 live estimator frame을 맞추는 replay-only `replay_skip_initial_images`를 추가했다. 기본값 0이라 production 입력에는 영향이 없다.
- recorded reset event를 적용하지 않은 replay에서 89.628초 `0.1572 m` jump를 결정적으로 재현했다. 3 px outlier 삭제는 기존 long track을 매 update 1~5개씩 제거해 active `149→63`, long track `121→35`로 고갈시켰다.
- 5 px는 deterministic bag을 통과했지만 live 10회가 terminal `9/10`, recovery 0 `8/10`이라 기각했다. 4 px는 deterministic bag을 통과하고 live 10회 recovery 0 `10/10`을 보였으나 후속 합산은 `17/18`이다.
- threshold는 hardcode하지 않고 `outlier_rejection_threshold_px` config로 분리했다. 설정이 없으면 upstream/default 3 px이며 검증된 SITL config만 4 px를 사용한다.
- reset 응답 뒤 preflight가 이전 VINS epoch의 alignment pair를 보존하던 버그를 수정했다. 새 arbitrary local frame에서 yaw/translation/error accumulator를 모두 초기화한다.
- solver fault 진단을 position-jump 로그에 추가했다. 첫 샘플은 features 19, cost `16277.7→15216.6`, 13 successful step, 6.3 ms, `NO_CONVERGENCE`로, 단순 solver time exhaustion이 아니라 고비용 저기하 비수렴이었다.
- 중단된 30회 진단 soak `20260717T142655KST`는 7회째에 position jump 5회와 validation timeout을 재현했다. 모든 fault solve가 iteration 12 한계의 `NO_CONVERGENCE`였고, 25 iteration 실험은 첫 run에서 recovery 2회로 악화되어 원복했다.
- 고비용 residual 분해 replay에서 fault 비용 `4551` 중 marginalization prior가 `4159`(약 91%), visual `346`, IMU `44`, range `3`이었다. prior 생성 전 outlier 삭제 순서 변경도 같은 89.628초 jump를 재현해 원복했다.
- validation dither ±0.10 m는 PX4 hold displacement gate와 충돌했고, ±0.04 m는 terminal `10/10`이나 recovery 0 `9/10`(`20260717T151641KST`)이라 근본 개선이 아니어서 제거했다.
- excitation `0.40 m/s`는 2/2 내부 jump, threshold `3.5 px`는 deterministic replay를 통과했지만 첫 live run에서 low-feature jump/validation timeout을 내어 모두 폐기했다. 최종 설정은 `0.55 m/s`, SITL `4 px`, dither 없음이다.
- 초기 solve cost-per-residual-block은 failure replay `0.5627`, 건강 replay `0.4836`, 건강 live 약 `0.549~0.729`, recovery 첫 후보 `0.584`로 겹쳐 gate로 사용할 수 없었다. 로그 진단만 유지한다.
- marginalization `NUM_THREADS=1`, Ceres 1.14 isolated A/B, LM trust-region 초기/최대 radius 제한은 동일 failure를 재현하거나 더 일찍 실패해 원복했다. 프로젝트는 다시 system Ceres 2와 `NUM_THREADS=4`를 사용한다.

- feature factor의 최소 track 길이를 4에서 3 frame으로 낮추면 기존 최종 10회 기준 post-ready 무복구가 8/10에서 9/10으로 개선됐다. 2 frame은 최대 step `0.232 m`를 노출해 폐기했다.
- Dogleg은 건강한 수십 초 뒤 `0.17~0.43 m` optimizer mutation을 드물게 수용했다. nonlinear solver를 damped Levenberg-Marquardt로 변경한 최종 후보에서는 10회 모두 post-ready reset과 solver failure가 0이었다.
- low-geometry correction gate를 `0.10`에서 `0.15 m`로 조정했다. 3-frame 건강 구간의 `0.10~0.11 m` correction은 허용하되 관측된 corrupt mutation `0.222~0.285 m`는 estimator/bridge의 독립 `0.15 m` 경계로 계속 차단한다.
- preflight validation timeout을 30초에서 45초로 늘렸다. 늦은 건강한 초기화는 GPS hover/handover 이전에 추가 8초 검증을 완료할 수 있다.
- bridge는 output age `0.20 s`, pose step `0.15 m`, implied speed `1.5 m/s`, monotonic timestamp, 10 healthy sample, 새 readiness generation을 요구한다.
- bridge가 readiness generation을 PX4 `VehicleOdometry.reset_counter`로 전달한다. reset 뒤 재승인 시 좌표계 discontinuity를 일반 이동으로 표시하지 않는다.
- handover integration smoke는 실제 PX4 입력 QoS로 output on → jump close → new readiness/reopen 및 reset counter 증가 → stale close를 확인했다.
- GPS fallback: `artifacts/clean_repeats/20260717T103025KST/`; reset 뒤 EV 메시지 0개, GPS fusion active, local XY valid, post-fault max step `0.000815 m`, pass.
- GNSS denied: `artifacts/clean_repeats/20260717T103319KST/`; reset 뒤 EV 메시지 0개, GPS fusion false, failsafe true, nav state 12, PX4 `Failsafe: blind land`, pass.

## 아직 불안정하거나 미검증

- Baylands 최종 GT accuracy batch는 `2/2` 통과지만 terminal `HOVER_READY`는 `1/2`였다. run01은 pre-score recovery 1회와 scored mission 직후 hover drift fail-closed가 있었다. 이는 2단계 accuracy acceptance를 반증하지 않지만 VIO 장기 무고장률 근거도 아니다.
- 최신 4 px 후보는 operational `17/18`이므로 “무복구 성공률 100%”를 확정하지 않았다. 한 번의 10/10 뒤 후속 fail-closed가 있었으며 30~100회 soak가 필요하다.
- readiness 전 내부 fault 없는 strict 비율은 최신 10회에서 `6/10`이다. 초기 nonlinear update 1에서 `0.1045~0.326 m` correction으로 reset되는 후보가 남는다. 정상 후보도 `0.1043 m` correction을 보여 단일 임계값 gate는 추가하지 않았다.

- 10/10은 유한 표본이며 실제 실패확률 0을 증명하지 않는다. readiness 전 내부 초기화 retry는 최종 10회 중 2회에서 발생했지만 PX4 handover 전이라 비행제어에는 노출되지 않았다.
- 이후 GPS fallback 회귀 `20260717T104151KST`에서 post-ready 이전 VINS recovery 1회가 다시 관측됐다. 최종 handover/PX4 fault assertion은 통과했지만 근본 무복구 빈도는 아직 100%로 확정할 수 없다.
- nonlinear update rollback/quarantine 실험은 persistent corruption에서 reset을 없애지 못했고, outlier-cleansing 변형은 최대 `8.71 m` output step을 노출해 즉시 전량 원복했다. 최종 소스에는 quarantine 경로가 없다.
- reset 뒤 자동 재-handover의 실제 비행 거동은 검증하지 않았다. counter는 전달되지만 arbitrary VINS origin/yaw 재정렬 시 controller setpoint 영향은 별도 시험이 필요하다.
- 실제 기체의 GNSS fallback 및 GNSS-denied 착륙, optical degradation, CPU saturation, 카메라/IMU hardware timestamp와 transport latency는 미검증이다.
- PX4 v1.14.4 SITL 설정에서 검증했다. 실기체 파라미터 파일과 firmware build가 동일한지 아직 확인하지 않았다.

## 다음 세션 첫 작업

7·8단계까지 완료됐다. 다음은 9단계로, 동일 글로벌 mission에서 GPS sensor를 다시 `on`한 뒤 GPS+VIO 재융합과 global-position jump를 rosbag/ULog로 판정한다.

## 주요 명령과 데이터

- Baylands 설치: `BAYLANDS_ARCHIVE=artifacts/world_assets/baylands-model.tar.gz tools/install_baylands_world.sh` (로컬 archive가 없으면 고정 URL에서 다운로드)
- interactive demo: `ros2 launch VIO_pkg vio_gazebo_demo.launch.py` (기본 `vio_baylands`)
- headless 반복: `tools/run_clean_preflight_repeats.sh 1` (기본 `vio_baylands`)
- 2단계 재현: `tools/run_baylands_relative_waypoint_test.sh 1`
- 2단계 최종 evidence: `artifacts/clean_repeats/20260721T080015KST/`
- 3단계 재현: `tools/run_vio_global_position_test.sh`
- 3단계 evidence: `artifacts/clean_repeats/20260721T084626KST/20260721T084630KST_run01/`
- 4단계 재현: `RECORD_BAGS=1 PRESERVE_LAST_BAGS=1 READY_HOLD_SECONDS=20 PX4_GPS_EV_FUSION_TEST=1 EXPERIMENT_LABEL=stage4_gps_ev_fusion tools/run_clean_preflight_repeats.sh 1`
- 4단계 evidence: `artifacts/clean_repeats/20260721T093911KST/20260721T093916KST_run01/`
- 5단계 재현: `tools/run_stage5_gps_vio_fusion_test.sh 1` (`pyulog`이 없으면 먼저 `docker exec project_f python3 -m pip install --user --no-deps -r /home/reventon/ros2_ws/src/tools/requirements-ulog.txt`)
- 5단계 evidence: `artifacts/clean_repeats/20260721T202758KST/20260721T202803KST_run01/`
- 6단계 재현: `tools/run_stage6_gps_dropout_test.sh 1`
- 7·8단계 재현: `tools/run_stage7_qgc_mission_ground_truth_test.sh 1` (`pymavlink`은 `tools/requirements-qgc-mission.txt`)
- 7·8단계 evidence: `artifacts/clean_repeats/20260722T005537KST/20260722T005543KST_run01/`
- 남은 목표: GPS 복구 후 재융합과 global jump 판정.

- clean 반복: `READY_HOLD_SECONDS=15 tools/run_clean_preflight_repeats.sh 10`
- PX4 GPS fallback: `PX4_VIO_FAULT_TEST=1 PX4_DISABLE_GPS_BEFORE_FAULT=0 READY_HOLD_SECONDS=15 tools/run_clean_preflight_repeats.sh 1`
- PX4 GNSS denied: `PX4_VIO_FAULT_TEST=1 PX4_DISABLE_GPS_BEFORE_FAULT=1 READY_HOLD_SECONDS=15 tools/run_clean_preflight_repeats.sh 1`
- 최신 4 px 10회: `artifacts/clean_repeats/20260717T133105KST/`
- 후속 반증/회귀: `artifacts/clean_repeats/20260717T140142KST/`, solver 진단 `artifacts/clean_repeats/20260717T141251KST/`
- 최신 PX4 fault evidence: `artifacts/clean_repeats/20260717T135258KST/`, `artifacts/clean_repeats/20260717T135529KST/`
- visualization/trace 비교: `artifacts/clean_repeats/20260717T164507KST/`, `artifacts/clean_repeats/20260717T171548KST/`
- 폐기한 reset-epoch 후보: `artifacts/clean_repeats/20260717T175537KST/`, `artifacts/clean_repeats/20260717T182304KST/`
- fail-closed latch 재검증: `artifacts/clean_repeats/20260717T104151KST/`, `artifacts/clean_repeats/20260717T104525KST/`
- quarantine 폐기 증거: `artifacts/clean_repeats/20260717T110306KST/`, `artifacts/clean_repeats/20260717T113353KST/`; 원복 smoke `artifacts/clean_repeats/20260717T113946KST/`
