# Project Status

마지막 업데이트: 2026-07-17 19:00 KST

## 현재 판정

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

- 최신 4 px 후보는 operational `17/18`이므로 “무복구 성공률 100%”를 확정하지 않았다. 한 번의 10/10 뒤 후속 fail-closed가 있었으며 30~100회 soak가 필요하다.
- readiness 전 내부 fault 없는 strict 비율은 최신 10회에서 `6/10`이다. 초기 nonlinear update 1에서 `0.1045~0.326 m` correction으로 reset되는 후보가 남는다. 정상 후보도 `0.1043 m` correction을 보여 단일 임계값 gate는 추가하지 않았다.

- 10/10은 유한 표본이며 실제 실패확률 0을 증명하지 않는다. readiness 전 내부 초기화 retry는 최종 10회 중 2회에서 발생했지만 PX4 handover 전이라 비행제어에는 노출되지 않았다.
- 이후 GPS fallback 회귀 `20260717T104151KST`에서 post-ready 이전 VINS recovery 1회가 다시 관측됐다. 최종 handover/PX4 fault assertion은 통과했지만 근본 무복구 빈도는 아직 100%로 확정할 수 없다.
- nonlinear update rollback/quarantine 실험은 persistent corruption에서 reset을 없애지 못했고, outlier-cleansing 변형은 최대 `8.71 m` output step을 노출해 즉시 전량 원복했다. 최종 소스에는 quarantine 경로가 없다.
- reset 뒤 자동 재-handover의 실제 비행 거동은 검증하지 않았다. counter는 전달되지만 arbitrary VINS origin/yaw 재정렬 시 controller setpoint 영향은 별도 시험이 필요하다.
- 실제 기체의 GNSS fallback 및 GNSS-denied 착륙, optical degradation, CPU saturation, 카메라/IMU hardware timestamp와 transport latency는 미검증이다.
- PX4 v1.14.4 SITL 설정에서 검증했다. 실기체 파라미터 파일과 firmware build가 동일한지 아직 확인하지 않았다.

## 다음 세션 첫 작업

새 세션은 최종 소스가 epoch 실험 없이 빌드되는지 확인한 뒤, 30회 clean SITL을 한 번에 수행해 `HOVER_READY`, recovery 0, readiness 전 jump/solver-failure-free를 함께 집계한다. frame trace on/off처럼 scheduling을 바꾸는 변경은 알고리즘 개선으로 채택하지 않는다. 이후에만 최종 소스로 PX4 GPS fallback/GNSS-denied 회귀를 다시 수행한다. 실기체 자유비행 전에는 별도 HIL/계류 go/no-go가 필수다.

## 주요 명령과 데이터

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
