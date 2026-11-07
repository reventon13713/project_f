# Jetson 도시 월드와 D435i VIO

대상은 Jetson Orin Nano Super 8GB, JetPack 6.2.3이다. ROS 2 Humble VIO는
기존 ARM64 이미지의 바이너리를 사용한다. 이 폴더는 이미지의
`/opt/project_f/simulation/city_vio`와 같은 위치에 해당한다.

## 전달한 원본

- `src/VIO_pkg/gazebo_worlds/vio_low_rise_city.world`는 노트북 실험에 사용한 원본이다.
  옆의 `gazebo_models`에 도시 지면, 건물 블록 16개, 도로 10개, 텍스처, sun과
  D435i 스테레오+IMU 모델이 있다.
- `config/d435i_oblique_stereo_imu_visualization.yaml`과 같은 디렉터리의 좌우 보정
  파일은 2026-08-24에 통과한 300 m 실행 폴더에서 가져왔다.
- `src/vins_fusion_ros2_d435i`는 ARM64 추정기의 C++ 소스·launch이며,
  `src/VIO_pkg/VIO_pkg`에는 카메라 릴레이, 특징점 표시, 좌표 변환 Python 노드가 있다.
  이 소스는 11월 7일 Jetson 배포판에 대응한다. 이후 추가된 실센서용 IMU QoS 옵션의
  기본값은 노트북 시뮬레이션 설정과 같은 reliable이다.
- `gazebo-linux-aarch64.lock`은 Linux ARM64용 Gazebo Classic 및 ROS 플러그인과
  의존성의 URL·SHA-256을 고정한다. 시뮬레이터 프로세스가 사용하는 Gazebo는
  11.15.1이고 카메라·IMU 플러그인은 gazebo_ros_pkgs 3.9.0이다.
  노트북의 Ubuntu 패키지는 Gazebo 11.10.2와 gazebo_ros_pkgs 3.9.0이었다.

## 실행

```bash
docker pull reventon13713/project_f:2026-11-07-sim-inputs
docker run --rm reventon13713/project_f:2026-11-07-sim-inputs sha256sum --quiet -c SHA256SUMS
docker run --rm -it --name city-vio reventon13713/project_f:2026-11-07-sim-inputs ./start-sensors.sh
```

첫 터미널은 열어 두고, Jetson의 두 번째 터미널에서 VIO를 실행한다.

```bash
docker exec -it city-vio /usr/local/bin/project-f-entrypoint bash -ec '
  mkdir -p /tmp/vins_output
  exec ros2 launch vins_fusion_ros2_d435i vins_fusion_ros2.launch.py \
    use_sim_time:=true config_file:="$PWD/config/d435i_oblique_stereo_imu_visualization.yaml"'
```

센서 점검용 `sensor_check.world`는 원본 도시의 고정 위치에 D435i를 놓는다.
모델 연결을 위해 실제 카메라를 연결할 필요가 없다. 300 m 실행 당시 월드는
별도로 보존했다. 여기서 확인할 항목은 새 가상 좌우 영상·IMU·시계 전달이다.
고정 센서 시험으로 300 m 이동, PX4 연동, 정확도 및 실시간 처리율을 판정할 수 없다.
컨테이너를 중지하려면 첫 터미널에서 Ctrl-C를 누른다.

## 이미지에서 소스 꺼내기

```bash
c=$(docker create reventon13713/project_f:2026-11-07-sim-inputs)
docker cp "$c:/opt/project_f/simulation/city_vio" ./city_vio_inputs
docker rm "$c"
```

기존 `2026-11-07` 태그는 계속 사용할 수 있다. 두 이미지의 공통 레이어는
Docker가 한 번만 보관한다.
