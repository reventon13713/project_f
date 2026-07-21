# Baylands world integration

## Selected public environment

The project uses the public Open Robotics `baylands` model as its realistic outdoor Gazebo Classic environment. The source archive is `http://models.gazebosim.org/baylands/model.tar.gz` (150,768,931 bytes) and is accepted only when its SHA-256 is `18bce28a1ae89a146b57218653d773ed432d4bd381252ffb19102af98f7e952c`.

All three upstream visual meshes, material scripts, and textures are preserved. `vio_baylands` replaces only the expensive triangle-mesh collision with a flat 500×500 m mission surface. The world keeps the upstream WGS84 origin at latitude `37.413534`, longitude `-121.996561`, elevation `1.3 m`, heading `0°`.

## Install and run

From the repository root:

```bash
tools/install_baylands_world.sh
```

When the already downloaded archive is available, installation can be repeated without a network transfer:

```bash
BAYLANDS_ARCHIVE=artifacts/world_assets/baylands-model.tar.gz \
  tools/install_baylands_world.sh
```

The interactive ROS launch uses `vio_baylands` by default:

```bash
docker start project_f
docker exec -it project_f bash
source /opt/ros/humble/setup.bash
source /home/reventon/ros2_ws/install/setup.bash
ros2 launch VIO_pkg vio_gazebo_demo.launch.py
```

The clean headless runner also defaults to this world:

```bash
tools/run_clean_preflight_repeats.sh 1
```

`VIO_GAZEBO_WORLD` can override the interactive launch world. `PX4_SITL_WORLD` overrides the repeat runner (for example, `PX4_SITL_WORLD=vio_feature`) and is saved in each `batch_config.tsv`.

## Integration contract

The `iris_fpv_cam` model must provide `/vio/camera/image_raw`, `/vio/imu`, `/vio/range`, and independent Gazebo-native `/vio/ground_truth/odometry`. PX4's simulated GPS remains enabled and the world origin must produce the expected WGS84 position. Stage 1 does not enable external-vision publication or alter GPS fusion/global conversion.

The 2026-07-21 smoke passed this contract on PX4 v1.14.4. See `RUN_HISTORY.md` for measured rates, ground contact, camera feature count, and geodetic checks. Stage 2 owns waypoint flight and VIO-versus-ground-truth accuracy acceptance.

## Stage 2 relative-waypoint acceptance

Run the scored test from the repository root:

```bash
tools/run_baylands_relative_waypoint_test.sh 1
```

The first XYZ cross initializes and validates VINS. A separate 12-waypoint XYZ cross is then scored from raw `/odometry` against independent `/vio/ground_truth/odometry`. Alignment fits yaw and translation only; it never fits scale. The fixed limits are `0.30 m` 3-D trajectory RMSE and `0.50 m` for trajectory maximum, ground-truth waypoint reach maximum, and waypoint VIO-to-ground-truth maximum.

The final automatic batch `artifacts/clean_repeats/20260721T080015KST/` passed ground-truth acceptance in 2/2 runs. See each run's `ground_truth_metrics.json` and the batch `summary.tsv`. External-vision publication and global conversion remained disabled.

## Stage 7/8 QGC global mission and GPS-denied ground truth

Run `tools/run_stage7_qgc_mission_ground_truth_test.sh 1` after installing `tools/requirements-qgc-mission.txt` and `tools/requirements-ulog.txt` in the container. The run writes `qgc_stage7.plan`, uploads the same eight-item standard MAVLink mission to PX4, and enters `AUTO.MISSION`. Item 0 fixes ground speed at `0.55 m/s`; seven WGS84 waypoints retain the initialized heading and use a `0.50 m` acceptance radius. Gazebo GPS is disabled at sequence 5, leaving three global waypoints and at least `8 m` of scored flight.

The Stage-8 evaluator uses the mission event window only. It requires the GPS stream to stop, remaining sequences and ULog mission completion, PX4/QGC map and VIO continuity, and independent Gazebo ground-truth accuracy. Fixed limits are post-dropout path `>=8 m`, PX4/VIO 3-D RMSE `<=1.0 m`, maximum `<=1.5 m`, ground-truth waypoint horizontal error `<=1.5 m`, trajectory step `<=0.5 m`, and evidence receipt gap `<=0.60 s`. The final artifact is `artifacts/clean_repeats/20260722T005537KST/20260722T005543KST_run01/`.
