# Lessons Learned

## 진단 부하 제거도 scheduling을 바꾸는 알고리즘 실험으로 취급한다

- 잘못된 접근 또는 가정: frame fingerprint와 queue trace는 진단뿐이므로 끄면 CPU 부하만 줄고 estimator 결과는 같거나 좋아진다고 가정.
- 왜 문제였는지: `show_track=0`·trace on은 유효 recovery 0 `9/9`였지만 trace off 배치는 terminal `9/9` 안에서 recovery 0이 `6/9`로 악화됐다.
- 확인된 사실: 전체 image/feature hashing과 로그 I/O가 camera callback과 nonlinear solver의 상대 scheduling을 바꾼다. 이것이 수치 failure의 근본 원인인지는 미확정이다.
- 올바른 접근: trace on/off를 동작 변경 A/B로 기록하고, executor/CPU/callback overlap을 계측한 뒤 명시적인 scheduling 정책을 검증한다.
- 다음에도 지켜야 할 규칙: 로그를 안정화용 pacing으로 의존하지 않으며, 진단 제거 후에도 동일 규모 live 반복을 수행한다.

## reset race가 그럴듯해도 stale-frame drop 관측만으로 채택하지 않는다

- 잘못된 접근 또는 가정: reset 전후 stale feature가 실제로 drop되면 rare recovery의 근본 원인을 해결했다고 판단.
- 왜 문제였는지: atomic enqueue 후보는 terminal 10/10이어도 recovery 0 `8/10`, strict `5/10`이었고, nonblocking 후보는 terminal `9/10`으로 기존 최고보다 낮았다.
- 확인된 사실: reset과 queue 사이 epoch 혼입 가능성과 live stale drop은 존재한다. 그러나 구현이 전체 failure 빈도를 낮춘다는 증거는 없고 정상 scheduling도 바꿀 수 있다.
- 올바른 접근: safety-correctness 가설도 deterministic reset-boundary test, live 성공률, queue latency를 모두 통과해야 채택한다. 실패한 후보는 전량 원복한다.
- 다음에도 지켜야 할 규칙: race를 발견한 것과 시스템 성공률 개선을 입증한 것을 구분한다.

## 한-frame jump guard만으로 느린 geometry corruption을 막을 수 없다

- 잘못된 접근 또는 가정: estimator pose step만 제한하면 모든 후속 corruption이 output 전에 잡힌다고 가정.
- 왜 문제였는지: 실패 run은 각 update가 0.25 m 아래인 채 active geometry가 장시간 축소되어 scale/vector error가 누적됐다.
- 실제 확인된 원인: 건강 run 대비 low-feature 지속시간과 optimizer correction이 동시에 크게 증가했다. bias와 solver failure는 선행하지 않았다.
- 올바른 접근: feature 수만 단독 threshold로 쓰지 않고 low active geometry와 큰 optimizer mutation을 결합해 publish 전에 거부한다. PX4 경계에서는 readiness/freshness/recovery generation을 별도로 요구한다.
- 다음에도 지켜야 할 규칙: instantaneous jump, slow drift, initialization health를 서로 다른 gate와 증거로 검증한다.

## recorder가 VINS보다 먼저 시작하는 것만으로 complete tracker history가 보장되지 않는다

- 잘못된 접근 또는 가정: 아직 advertise되지 않은 relay topic을 rosbag 명령에 넣으면 첫 relay frame부터 기록된다고 가정.
- 왜 문제였는지: DDS discovery 동안 초기 callback 233개가 누락되어 동일 bag replay의 feature tracker state가 live와 달랐다.
- 올바른 접근: relay를 먼저 advertise하고 recorder subscription을 준비한 뒤 estimator를 시작하며, terminal metrics 뒤 grace도 둔다.
- 다음에도 지켜야 할 규칙: bag overlap뿐 아니라 trace 시작 이전 bag frame 존재와 live DDS publisher count를 확인한다.

## 나쁜 initialization 후보를 processing epoch 전체 reset으로 거부하지 않는다

- 잘못된 접근 또는 가정: initial solve가 bias limit을 넘으면 processing thread 안에서 `resetStateLocked()`를 호출하는 것이 가장 안전하다고 가정
- 왜 문제였는지: 압축 full-bag replay에서 거부 직후 incoming IMU와 epoch가 어긋나 `Abnormal IMU timestamp jump` 뒤 SIGSEGV가 발생했다.
- 실제 확인된 원인: gate 판단 자체가 아니라 전체 input buffer/tracker/preintegration epoch를 동시에 초기화하는 retry 방식이 live callback과 충돌했다.
- 올바른 접근: initial solve 전 state/depth/calibration을 snapshot하고, reject 시 optimizer mutation과 marginalization prior만 되돌린 뒤 INITIAL window를 slide한다.
- 다음에도 지켜야 할 규칙: candidate-level rejection과 estimator-wide recovery를 구분하고, 다음 window를 보존할 수 있는 rollback을 우선한다.

## offline threshold는 모든 역사적 정상 사례의 false rejection을 먼저 확인한다

- 잘못된 접근 또는 가정: 최근 ON/OFF 20회에서 분리되는 alignment scale+bias 조합이면 gate로 충분하다고 판단
- 왜 문제였는지: 전체 clean artifact 72개로 넓히자 같은 threshold가 최종 정상 또는 생존 initialization 7개 이상을 거부했다.
- 실제 확인된 원인: SfM arbitrary scale과 초기 bias 분포는 trajectory/window에 따라 넓게 겹친다.
- 올바른 접근: 기존 failure safety limit처럼 역사적 survived 범위를 넘는 명확한 조건부터 적용하고, 분석 label과 ground truth를 구분한다.
- 다음에도 지켜야 할 규칙: 작은 최신 batch에 맞춘 복합 threshold를 live gate로 바로 옮기지 않는다.

## initialization complete 로그와 건강한 output 준비를 같은 사건으로 보지 않는다

- 잘못된 접근 또는 가정: initial optimize가 finite/bias check를 통과하면 즉시 odometry를 publish해도 된다고 가정
- 왜 문제였는지: live run08은 initialization 직후 첫 nonlinear update에서 accel bias `2.59453 m/s²`로 reset됐다. 다른 run은 세 번째 update에서 `0.771672 m` correction 후 jump guard가 발동했다.
- 실제 확인된 원인: 초기 optimizer 통과 뒤 첫 수 frame에서 bias와 position correction이 추가로 급변할 수 있다.
- 올바른 접근: estimator는 내부 NON_LINEAR update를 수행하되 첫 2개 output을 보류하고 3번째 update와 failure guard까지 통과한 뒤 publish한다.
- 다음에도 지켜야 할 규칙: solver mode, probation pass, externally consumable health를 별도 상태로 기록한다.

## pre-BA depth와 post-BA cheirality를 구분한다

- 잘못된 접근 또는 가정: triangulation 직후 음수 depth 수만으로 BA 결과까지 나쁘다고 판정
- 왜 문제였는지: recovery replay 후보는 pre/post 상태가 달랐고 post-BA 정상 후보는 음수 depth가 0 또는 매우 낮았다. BA가 geometry를 고칠 수 있다.
- 실제 확인된 원인: 기존 `SFM geometry diagnostics`는 BA 전 값이었다.
- 올바른 접근: BA가 끝난 camera pose/3-D point로 observation depth를 다시 계산하고 post-BA 값만 acceptance gate에 사용한다.
- 다음에도 지켜야 할 규칙: 진단값의 계산 시점을 로그 이름에 명시하고 서로 다른 단계의 수치를 같은 threshold에 섞지 않는다.

## replay 결정성만으로 live 처리 가능성을 판단하지 않는다

- 잘못된 접근 또는 가정: relay의 모든 frame을 enqueue해 동일 bag replay가 완전히 일치하면 live에서도 안전한 입력 정책이라고 가정
- 왜 문제였는지: 약 20 Hz relay 전체를 10 Hz 설정 estimator에 넣은 live smoke run에서 queue가 최대 17 frame, `0.816 s` 밀렸고 path scale `5.213`, max VIO step `38.390 m`가 발생했다.
- 실제 확인된 원인: queue-empty 조건 제거는 결정성을 개선했지만 입력률과 estimator 처리 예산의 불일치를 별도로 해결하지 않았다.
- 올바른 접근: queue 상태와 독립적인 명시적 callback sequence 정책으로 2:1 downsample하고 queue depth/span을 live에서 계측한다.
- 다음에도 지켜야 할 규칙: replay 반복성, live 처리량, estimator 정확도를 서로 다른 검증 축으로 다룬다.

## 센서 ON/OFF 효과는 최종 통과율과 무복구 통과율을 함께 본다

- 잘못된 접근 또는 가정: LiDAR OFF에서 최종 `HOVER_READY`가 9/10이면 range factor가 실패의 근본 원인이라고 판단
- 왜 문제였는지: OFF 성공 9회 중 3회는 output loss/recovery를 거쳤고 전체 RMSE가 최대 `1.668 m`였다. 무복구 통과는 ON/OFF 모두 6/10이었다.
- 실제 확인된 원인: OFF에서도 path scale `1.05844` timeout과 output-loss recovery가 반복됐다. range factor 단독 원인 가설은 기각됐다.
- 올바른 접근: final outcome, recovery count, 전체 RMSE/max step, 마지막 segment 품질을 함께 비교하고 장기 고도 효과는 별도 검증한다.
- 다음에도 지켜야 할 규칙: recovery가 가린 성공을 센서/알고리즘 개선의 근거로 합산하지 않는다.

## 실제 launch config와 유사 이름의 미사용 config를 구분한다

- 잘못된 접근 또는 가정: `src/VIO_pkg/config/vins_config.yaml`의 `estimate_extrinsic=2`, `estimate_td=1`을 현재 live estimator 설정으로 해석
- 왜 문제였는지: `vio_vins.launch.py`의 기본 경로는 `vins_fusion_ros2/config/euroc/euroc_mono_imu_config.yaml`이며 실제 값은 fixed extrinsic, fixed `td=0`이다.
- 실제 확인된 원인: 프로젝트에 역할이 겹쳐 보이는 config가 둘 있고 현재 launch는 하나만 사용한다.
- 올바른 접근: launch substitution과 install overlay가 실제로 전달하는 config 경로를 확인하고 batch마다 snapshot/hash를 남긴다.
- 다음에도 지켜야 할 규칙: 파일명이나 주석만으로 runtime parameter를 단정하지 않는다.

## recorded service event는 같은 estimator boundary를 보장하지 않는다

- 잘못된 접근 또는 가정: rosbag의 reset event를 별도 helper가 service로 다시 호출하면 live와 같은 camera frame 직후 reset될 것이라고 가정
- 왜 문제였는지: 2배속 replay에서 service reset은 live의 `49.080 s` boundary보다 3 frame 늦은 `49.228 s` 뒤 실행됐고, 이후 feature fingerprint가 0/161 일치했다.
- 실제 확인된 원인: bag frame loss가 아니라 event subscriber→service request→service callback의 비동기 지연이었다.
- 올바른 접근: replay 전용 recorded reset은 VINS image callback group 안에서 적용해 camera frame과 같은 serialization boundary를 사용한다.
- 다음에도 지켜야 할 규칙: event storage time과 실제 state mutation 완료 시점을 같은 것으로 간주하지 않는다.

## processing queue 상태로 입력 frame을 선택하지 않는다

- 잘못된 접근 또는 가정: 짝수 frame 외에도 feature queue가 비었을 때 홀수 frame을 넣으면 latency만 줄고 결과에는 영향이 없다고 가정
- 왜 문제였는지: 동일 bag replay가 `57.216 s`부터 queue-empty 판정이 달라져 서로 다른 frame을 처리했고, 이후 bias reset과 재초기화 시점이 갈라졌다.
- 실제 확인된 원인: `featureBuffer.empty()`는 processing thread scheduling 결과이므로 estimator 입력 sequence를 비결정적으로 만들었다.
- 올바른 접근: relay가 이미 rate-limit한 모든 frame을 enqueue해 입력 sequence를 timestamp로 고정한다.
- 다음에도 지켜야 할 규칙: drop/downsample이 필요하면 queue 상태가 아니라 명시적 sequence/timestamp 정책을 사용하고 선택 결과를 기록한다.

## threshold timeout을 전체 trajectory corruption으로 간주하지 않는다

- 잘못된 접근 또는 가정: `VALIDATION_TIMEOUT`이면 VINS trajectory 전체가 기하학적으로 깨졌다고 간주
- 왜 문제였는지: full-bag run04는 excitation path scale `1.050663`으로 상한 `1.05`를 극소량 넘었지만 offline 전체 3D RMSE는 `0.096 m`로 인접 성공 run03의 `0.094 m`와 거의 같았다.
- 실제 확인된 원인: 직접 timeout 조건은 scale 상한 경계 초과이다. 초기 SFM 실패와 늦은 초기화가 scale 차이를 만든 인과인지는 미확정이다.
- 올바른 접근: outcome, threshold margin, live excitation metric, offline 전체 metric, solver/reset 신호를 함께 비교한다.
- 다음에도 지켜야 할 규칙: 경계 threshold 실패와 output-loss/position-jump corruption을 같은 실패 분류로 합치지 않는다.

## 시작 지연의 효과도 최종 outcome만으로 판정하지 않는다

- 잘못된 접근 또는 가정: recorder 준비와 같은 2초 지연에서 최종 `HOVER_READY`가 8/10이면 시작 지연이 안정성을 개선했다고 간주
- 왜 문제였는지: 성공 8회 중 2회는 output loss와 recovery를 거쳤고 전체 RMSE가 `1.165 m`, `1.683 m`였다. 무복구 통과는 0초와 2초 조건 모두 6/10이었다.
- 실제 확인된 원인: 2초 조건에서도 hover drift 실패 1회와 recovery 2회 뒤 최종 output loss 1회가 발생했다. outcome 분포가 달라진 원인은 미확정이다.
- 올바른 접근: 시작 조건 실험도 최종 통과율, 무복구 통과율, recovery count, 전체/segment metric을 함께 비교한다.
- 다음에도 지켜야 할 규칙: 작은 반복 표본의 최종 통과율 상승만으로 estimator 안정성 개선을 주장하지 않는다.

## 최종 성공 outcome만으로 중간 estimator 실패를 숨기지 않는다

- 잘못된 접근 또는 가정: `HOVER_READY`이면 run 전체가 reset이나 corruption 없이 건강했다고 간주
- 왜 문제였는지: full-bag run05는 첫 validation에서 path scale `1.11`, vector error `2.74 m`와 output loss가 발생했지만 recovery 뒤 최종 `HOVER_READY`가 됐다.
- 실제 확인된 원인: 최초 corruption의 원인은 미확정이지만 recovery 전 전체 RMSE `1.227 m`와 recovery 후 마지막 segment RMSE `0.072 m`가 크게 달랐다.
- 올바른 접근: outcome과 함께 recovery count, output gap, segment별 metric error를 기록한다.
- 다음에도 지켜야 할 규칙: 성공률을 보고할 때 최종 통과와 무복구 통과를 구분한다.

## bag 기록 시각과 estimator sim-time을 같은 timestamp로 가정하지 않는다

- 잘못된 접근 또는 가정: PX4 메시지 timestamp 또는 rosbag storage timestamp를 VINS 진단 로그의 sim-time 초와 직접 비교
- 왜 문제였는지: 새 bag의 PX4 timestamp는 epoch 기반 기록 시각이어서 `55.804~59.496 s` window 선택 결과가 비었다.
- 실제 확인된 원인: timestamp 기준 차이. `/clock`의 storage timestamp와 simulation clock을 보간하면 정확한 PX4 window를 얻을 수 있었다.
- 올바른 접근: live bag의 분석 구간은 `/clock`으로 storage time→sim-time 매핑 후 선택한다.
- 다음에도 지켜야 할 규칙: 서로 다른 로그/메시지를 결합하기 전에 timestamp epoch와 clock source를 확인한다.

## 서로 다른 world convention을 yaw fit만으로 정렬하지 않는다

- 잘못된 접근 또는 가정: ROS ENU의 VINS 위치와 PX4 NED 위치를 축 변환 없이 yaw와 translation만 fit하면 된다고 가정
- 왜 문제였는지: XY 축 교환과 Z 반전은 yaw 회전으로 표현할 수 없어 성공 bag의 전체 trajectory RMSE가 `1.036 m`, 최대 오차가 `3.368 m`로 과대 계산됐다.
- 실제 확인된 원인: 고정 ENU→NED 변환 누락. 변환 후 같은 bag의 RMSE는 `0.076 m`, 최대 오차는 `0.188 m`였다.
- 올바른 접근: 알려진 좌표계 convention을 먼저 고정 변환하고, 그 뒤에만 VINS local world의 임의 yaw/translation을 no-scale fit한다.
- 다음에도 지켜야 할 규칙: 서로 다른 estimator 출력을 비교하기 전에 축 순서, handedness, vertical sign을 코드와 수치 양쪽에서 확인한다.

## clean 환경의 한 번 성공을 estimator 안정성으로 확대 해석하지 않는다

- 잘못된 접근 또는 가정: heading 보존과 단일 relay에서 한 번 통과했으므로 같은 조건이면 대부분 통과할 것이라고 기대
- 왜 문제였는지: 자동 clean 10회에서 성공률은 60%였고 scale timeout 3회와 position-jump/output-lost 1회가 발생했다.
- 실제 확인된 원인: 환경 오염은 제거됐지만 estimator 실패의 세부 원인은 아직 미확정. clean 조건만으로 충분하지 않다는 사실은 확인됨.
- 올바른 접근: 동일 조건을 batch로 반복하고 outcome, path scale, recovery, full-trajectory error를 run별로 보존한다.
- 다음에도 지켜야 할 규칙: handover 활성화 전 clean 반복 성공률과 실패 bag의 원인 분석을 모두 완료한다.

## accel bias 폭발을 곧바로 센서 bias 원인으로 해석하지 않는다

- 잘못된 접근 또는 가정: 큰 accel bias가 출력되면 IMU noise/bias 설정 자체가 원인이라고 단정
- 왜 문제였는지: 기준 재생에서는 초기 bias가 작다가 후속 nonlinear update에서 `2.5~3.1 m/s²`로 증가한 사례가 있었고, 다른 실패에서는 잘못된 scale과 함께 초기 최적화가 큰 bias를 만들었다.
- 실제 확인된 원인: 최종 원인은 아직 미확정. 다만 optimizer가 visual/IMU 불일치를 accel bias로 흡수하는 현상은 확인됨.
- 올바른 접근: 초기화 전 입력 window, visual translation/scale, rotation residual, 최적화 전후 bias를 함께 비교한다.
- 다음에도 지켜야 할 규칙: bias threshold/reset은 안전장치로 기록하고 원인 해결로 간주하지 않는다.

## 초기화 실패를 운동 부족으로 추정하기 전에 ground truth로 확인한다

- 잘못된 접근 또는 가정: 실패 window는 충분히 움직이지 않았을 것이라고 추정
- 왜 문제였는지: 실패 window에는 `0.677 m` displacement와 `7.93°` rotation이 있었다.
- 실제 확인된 원인: 운동 부족 가설은 해당 window에 대해 기각됨.
- 올바른 접근: PX4/Gazebo ground truth로 window displacement, path, rotation을 계산한 뒤 운동 부족 여부를 판정한다.
- 다음에도 지켜야 할 규칙: command trajectory가 아니라 실제 측정 trajectory를 증거로 사용한다.

## correspondence 또는 inlier 수만으로 relative pose를 선택하지 않는다

- 잘못된 접근 또는 가정: 가장 많은 inlier를 가진 후보가 가장 물리적으로 올바른 translation을 제공할 것이라고 가정
- 왜 문제였는지: 최다 inlier 선택 실험에서 position jump가 `0.84 m`로 악화되고 scale이 `0.0052`까지 낮아졌다.
- 실제 확인된 원인: inlier 수는 visual–inertial translation consistency를 보장하지 않는다.
- 올바른 접근: 후보의 visual reprojection 품질뿐 아니라 IMU 및 가능한 ground truth와의 translation 방향/scale 일관성을 별도로 평가한다.
- 다음에도 지켜야 할 규칙: 이 실험을 동일한 형태로 반복하지 않는다.

## RANSAC threshold를 임의로 강화하지 않는다

- 잘못된 접근 또는 가정: synthetic image이므로 threshold를 줄이면 outlier가 줄어 안정화될 것이라고 가정
- 왜 문제였는지: `1.0 px → 0.5 px` 실험은 usable correspondence를 줄이고 초기화 scale과 bias를 악화시켰다.
- 실제 확인된 원인: threshold 강화만으로 병진 방향 문제가 해결되지 않음.
- 올바른 접근: threshold를 바꾸기 전에 inlier residual 분포와 translation consistency를 측정한다.
- 다음에도 지켜야 할 규칙: `0.5 px` 설정을 근거 없이 다시 적용하지 않는다.

## rosbag 반복 비교는 입력 시작 조건까지 같아야 한다

- 잘못된 접근 또는 가정: 같은 bag 파일을 재생하면 estimator 입력도 완전히 동일하다고 간주
- 왜 문제였는지: 기존 반복 재생에서 relay 초기 frame 수가 한 장 달랐고 scale도 달랐다. 새 run04에서도 저장 relay bag의 두 replay는 서로 같았지만 live와 첫 후보 feature/correspondence부터 `407/61` 대 `409/62`로 달랐고, live의 15회 SFM 실패가 replay에서는 1회만 발생했다.
- 실제 확인된 원인: bag의 메시지 집합이 같아도 estimator가 실제 소비한 callback/frame/feature window가 같다는 보장은 없다. callback scheduling과 OpenCV feature/RANSAC 상태 중 어느 요소가 live/replay 차이를 만들었는지는 미확정이다.
- 올바른 접근: sensor publisher보다 VINS/relay를 먼저 준비하고 첫 전달 frame timestamp, 전체 소비 frame sequence, frame count, callback ordering, RNG 조건을 기록한다.
- 다음에도 지켜야 할 규칙: bag 파일 동일성, DDS 전달 동일성, estimator window 동일성, feature/RANSAC 동일성을 각각 구분한다.

## 임시 컨테이너 로그를 유일한 기록으로 남기지 않는다

- 잘못된 접근 또는 가정: `/tmp` 로그가 다음 세션까지 항상 남는다고 가정
- 왜 문제였는지: 컨테이너 삭제 시 모든 임시 로그가 사라질 수 있다.
- 실제 확인된 원인: 현재 핵심 재생 로그가 컨테이너 `/tmp`에만 있음.
- 올바른 접근: 다음 live run부터 프로젝트 또는 명시적인 영구 데이터 경로에 로그와 rosbag을 저장한다.
- 다음에도 지켜야 할 규칙: `RUN_HISTORY.md`에는 재현 명령과 영구 로그 경로를 함께 기록한다.

## ROS Gazebo sensor plugin 경로와 rosbag type support를 실행 전에 검증한다

- 잘못된 접근 또는 가정: PX4의 Gazebo Classic 실행이 ROS 2 Gazebo sensor plugin과 workspace message type을 자동으로 찾는다고 가정
- 왜 문제였는지: PX4가 설정한 `GAZEBO_PLUGIN_PATH`에는 `/opt/ros/humble/lib`가 없었고 IMU/camera/range publisher가 생성되지 않았다. recorder가 workspace overlay를 source하지 않으면 `px4_msgs/msg/VehicleOdometry`를 bag에 기록할 수 없었다.
- 실제 확인된 원인: 환경 path와 overlay 초기화 누락
- 올바른 접근: simulator 시작 시 ROS Gazebo library 경로를 유지하고, run 전 sensor publisher·PX4 odometry type·recorder subscription을 확인한다.
- 다음에도 지켜야 할 규칙: 토픽 이름이 보인다는 사실만으로 sensor data 흐름이나 rosbag 저장 성공을 판단하지 않는다.

## rosbag에 저장된 relay 토픽을 재생할 때 relay를 다시 띄우지 않는다

- 잘못된 접근 또는 가정: `vio_vins.launch.py`를 띄운 채 raw와 relay 토픽이 모두 든 bag 전체를 재생
- 왜 문제였는지: 저장된 relay 영상과 새 relay 출력이 같은 토픽으로 동시에 들어가 중복 timestamp, feature velocity NaN, 비결정적 초기화를 만들었다.
- 실제 확인된 원인: 단일-source 재생에서는 목표 실패 window가 동일 수치로 재현됐고 projection NaN은 이중 source에서만 발생했다.
- 올바른 접근: 저장 relay 토픽을 쓸 때는 `vins_fusion_ros2.launch.py`만 실행한다. raw를 재처리할 때는 저장 relay 토픽을 재생 목록에서 제외한다.
- 다음에도 지켜야 할 규칙: replay 전 대상 토픽 publisher count가 정확히 1인지 확인한다.

## ROS launch 부모 종료만으로 자식 프로세스가 모두 종료됐다고 가정하지 않는다

- 잘못된 접근 또는 가정: launch parent에 신호를 보낸 뒤 relay/static TF도 종료됐다고 간주
- 왜 문제였는지: 오래된 relay가 다음 simulation timestamp가 이전 값 `92.964 s`를 넘자 다시 발행해 live READY run을 오염시켰다.
- 실제 확인된 원인: bag에서 해당 시점부터 relay timestamp 중복/역행 1,012개가 생겼고 projection factor의 `velocity_j`가 NaN이었다.
- 올바른 접근: run 전후 `vio_camera_relay`, VINS, static TF, recorder, Gazebo, PX4 프로세스를 직접 확인하고 relay publisher count를 검사한다.
- 다음에도 지켜야 할 규칙: 깨끗한 프로세스 상태도 실험 조건의 일부로 기록한다.

## 초기 heading을 절대 0으로 덮어쓰지 않는다

- 잘못된 접근 또는 가정: world/model 초기 yaw와 무관하게 모든 병진 waypoint의 yaw를 0으로 지정
- 왜 문제였는지: 첫 초기화 창에서 0.504초 동안 45.47° 회전해 visual parallax는 만들었지만 scale/bias 초기화를 불안정하게 했다.
- 실제 확인된 원인: 초기 PX4 yaw를 유지한 clean run에서 이륙 초기 scale과 bias가 개선되고 full trajectory 및 60초 hover를 통과했다.
- 올바른 접근: 실제 초기 yaw를 캡처하고 병진 excitation 동안 heading을 유지하며 yaw excitation은 나중에 상대각으로 수행한다.
- 다음에도 지켜야 할 규칙: 주석의 trajectory 순서가 실제 setpoint 각 축에도 그대로 구현됐는지 bag 자세로 검증한다.

## 2026-07-17 — terminal 성공, recovery-free, PX4 안전을 분리해 측정한다

- 잘못된 가정: 최종 `HOVER_READY`만 높으면 VIO가 안정적이고, subscriber에서 메시지가 0이면 gate가 막았다고 볼 수 있다.
- 문제: recovery가 중간 corruption을 숨길 수 있고 QoS mismatch도 메시지 0을 만든다. 또한 bridge 차단만으로 PX4 EKF2가 어떤 aid를 쓰고 어떤 failsafe로 전환하는지 알 수 없다.
- 확인된 원인/사실: 기존 smoke subscriber QoS가 PX4 publisher와 맞지 않았다. 실제 QoS와 EKF2 native status를 사용하자 EV fusion, gate close, GPS fallback, blind-land를 각각 확인할 수 있었다.
- 올바른 접근: terminal outcome, post-ready recovery, estimator event, 실제 PX4 EV/GPS fusion flag, nav state/failsafe, post-fault 입력 수를 모두 독립 assertion으로 둔다.

## 2026-07-17 — 안전 경계를 완화하기 전에 optimizer step policy를 본다

- 잘못된 접근: rare jump를 feature 수나 guard threshold만 바꿔 없애려는 것.
- 문제: Dogleg이 충분한 feature가 있어도 수십 초 뒤 큰 mutation을 수용했으며, guard 완화는 이를 PX4에 노출할 수 있다.
- 확인된 사실: 2-frame은 `0.232 m` unsafe step을 만들었고 Dogleg failure는 `0.17~0.43 m` mutation으로 시작했다. LM 최종 10회 출력 step 최대는 `0.0758 m`였다.
- 올바른 접근: feature 조건은 3 frame보다 낮추지 않고, damped LM으로 mutation 빈도를 먼저 줄이며 estimator와 bridge 양쪽 `0.15 m` 경계를 유지한다.

## 2026-07-17 — handover 전 안전한 대기는 실패가 아니다

- 잘못된 가정: validation timeout을 늘리면 실패를 숨긴다.
- 문제: 늦게 초기화된 정확한 VIO가 8초 continuous hold를 끝내기 2초 전에 30초 timeout이 나는 사례가 있었다.
- 확인된 사실: 해당 구간은 GPS hover이고 handover 전이며 RMSE `0.061 m`, scale `1.039`였다.
- 올바른 접근: acceptance 기준과 hold 시간은 그대로 두고 pre-handover 대기창만 45초로 늘린다. handover 이후 stale/reset timeout은 완화하지 않는다.

## 2026-07-17 — 비정상 optimizer update를 rollback하는 것만으로 안전하지 않다

- 잘못된 접근: rejected nonlinear state만 되돌리고 window를 계속 진행하면 reset 없이 회복할 수 있다고 가정했다.
- 문제: persistent corruption은 다음 solve에서도 같은 jump를 요구했다. restored prediction 기준 outlier 제거는 feature를 대량 삭제해 관성 prediction drift를 정상 update처럼 만들었고 최대 `8.71 m` output step을 노출했다.
- 확인된 사실: bounded rollback은 단발 mutation 일부만 흡수했지만 최종 성공률을 보장하지 못했다. outlier-cleansing 변형은 명백한 안전 회귀였다.
- 올바른 접근: 현재는 full reset guard를 유지한다. 향후 rollback을 다시 시도하려면 마지막 published pose 대비 독립 output continuity, prior consistency, 최소 feature floor, fault-injection unit test를 먼저 갖춘다.

## 2026-07-17 — 비행 중 새 VIO frame의 자동 재연결은 기본 금지한다

- 잘못된 가정: `reset_counter`만 증가시키면 reset 뒤 자동 re-handover가 항상 안전하다.
- 문제: VINS origin/yaw가 바뀌면 PX4가 reset을 알아도 controller setpoint와 local frame 연속성은 별도 문제다.
- 올바른 접근: production bridge는 첫 in-flight fault 뒤 latch closed로 유지하고 명시적 restart/새 preflight 없이는 재연결하지 않는다.

## 2026-07-17 — 한 번의 10/10 뒤에도 바로 완료로 판정하지 않는다

- 잘못된 접근: deterministic replay 통과와 clean 10/10을 곧바로 실패확률 0으로 해석하는 것.
- 확인된 사실: 4 px 후보는 한 배치에서 recovery 0 `10/10`이었지만 후속 3회에서 late jump/recovery 뒤 fail-closed 1건이 재현됐다. 합산은 `17/18`이다.
- 올바른 접근: 동일 소스의 후속 반증 배치를 기존 성공 배치와 함께 집계하고, 30~100회 soak 전에는 “100% 확정”이나 실기체 승인 표현을 쓰지 않는다.

## 2026-07-17 — reset은 VINS alignment epoch도 함께 끊는다

- 문제: reset 뒤 scale accumulator만 초기화하고 yaw/translation alignment pair를 남기면 서로 다른 arbitrary local frame이 섞여 수 m vector error가 생긴다.
- 올바른 접근: reset 성공 시 VIO sample/scale뿐 아니라 alignment pair, fitted yaw/translation, vector-error accumulator를 원자적으로 새 epoch로 초기화한다.
