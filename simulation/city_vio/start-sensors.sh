#!/usr/bin/env bash
set -eu
script=$(readlink -f "$0")
cd "$(dirname "$script")"
if [[ -z ${DISPLAY:-} ]]; then
    exec xvfb-run -a -s "-screen 0 960x640x24 +extension GLX" "$script" "$@"
fi
# Keep the simulator and its Gazebo plugins in the same binary distribution.
prefix=/opt/city-gazebo
unset COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH
export PATH="$prefix/bin:/usr/bin:/bin"
export LD_LIBRARY_PATH="$prefix/lib:$prefix/lib/gazebo-11/plugins"
export AMENT_PREFIX_PATH="$prefix"
export GAZEBO_MODEL_PATH="$PWD/src/VIO_pkg/gazebo_models:$prefix/share/gazebo-11/models"
export GAZEBO_RESOURCE_PATH="$prefix/share/gazebo-11"
export GAZEBO_PLUGIN_PATH="$prefix/lib:$prefix/lib/gazebo-11/plugins"
export GAZEBO_MODEL_DATABASE_URI=""
exec gzserver --verbose -s libgazebo_ros_init.so "$PWD/sensor_check.world" "$@"
