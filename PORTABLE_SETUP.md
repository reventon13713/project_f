# Portable setup (0722)

The `reventon13713/project_f:0722` image contains the ROS 2 Humble/PX4/Gazebo
base environment, the pinned Baylands assets, `pyulog==0.9.0`, and
`pymavlink==2.4.49`. Project source is kept in GitHub and bind-mounted into the
container so that source and image updates remain independent.

## Start on another computer

```bash
git clone https://github.com/reventon13713/project_f.git
cd project_f
docker pull reventon13713/project_f:0722

xhost +local:docker
docker run -it --name project_f \
  --network host --ipc host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${PWD}:/home/reventon/ros2_ws/src" \
  reventon13713/project_f:0722
```

The test launchers run from the host after the container has been created. For
example, stages 7 and 8 are executed together with:

```bash
./tools/run_stage7_qgc_mission_ground_truth_test.sh
```

The scripts expect the container name to be `project_f`. Generated rosbag, ULog,
and report files are written below `artifacts/` and are intentionally excluded
from Git.

## Rebuild the image

```bash
docker build -f Dockerfile.0722 -t reventon13713/project_f:0722 .
```

The Baylands archive is downloaded from its canonical Gazebo model URL and
verified with the SHA-256 recorded in the Dockerfile.
