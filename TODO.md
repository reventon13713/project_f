# TODO

## P1

- 최종 원복 소스를 먼저 build/smoke 확인한 뒤 최신 4 px·`show_track=0` 후보를 최소 30회 clean SITL로 확대한다. acceptance는 terminal, recovery 0, readiness 전 position/geometry fault-free를 별도 보고한다. 현재 최고 근거는 4 px operational `17/18`과 show-track-off 유효 `9/9`이며, reset-epoch 후보는 더 나빠 폐기됐다.
- high-feature late mutation에서 marginalization prior가 final cost의 약 91%를 차지한 원인을 conditioning, linearization point 수명, second-new 연속 구간으로 분해한다. prior drop/robust loss는 replay와 output-step guard 검증 전 적용하지 않는다.
- callback/solver scheduling 민감성을 별도 재현한다. frame trace off가 recovery 0을 `9/9 → 6/9`로 악화한 결과를 기준으로, 진단 부하를 안정화 수단으로 사용하지 말고 executor thread 수·CPU saturation·camera callback/solver overlap을 계측만 한 뒤 한 변수씩 검증한다.

- 실기체 투입 전 HIL/계류 go/no-go 절차를 작성하고 hardware camera/IMU timestamp skew, end-to-end latency, CPU 여유를 계측한다.
- reset 이후 자동 재-handover 정책을 결정한다. 기본은 비행 중 자동 재연결 금지이며, 허용하려면 VINS frame continuity와 PX4 setpoint continuity를 별도 검증한다.
- 30회 최종 후보가 통과한 뒤에만 같은 바이너리로 GPS fallback과 GNSS-denied Land/Descend를 다시 1회씩 assertion한다. 현재 최신 통과 증거는 `20260717T135258KST`, `20260717T135529KST`다.
- 실제 기체 PX4 firmware/parameter dump를 SITL 검증값과 비교하고 GPS fallback 및 GNSS-denied Land/Descend를 프로펠러 제거 상태부터 단계적으로 시험한다.

## P2

- 최종 후보를 30~100회 clean SITL soak로 확대해 10/10 결과의 신뢰구간을 좁히고 readiness 전 retry 빈도도 별도 집계한다.
- 카메라 가림, 저조도, motion blur, texture loss, IMU dropout/time jump, CPU saturation을 주입해 gate close와 PX4 failsafe를 반복 검증한다.
- rare weak-geometry mutation의 근본 원인을 marginalization prior, feature lifetime, IMU/visual timing으로 분리한다. 검증 없이 nonlinear rollback/outlier 대량 제거를 다시 적용하지 않는다.

## P3

- VINS covariance를 PX4 `VehicleOdometry` variance에 연결하고 현재 고정 covariance/quality를 실제 innovation 통계로 보정한다.
- 장기적으로 arbitrary VINS frame을 PX4 local frame에 연속 정렬하는 명시적 transform 계층을 설계한다.
