#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-project_f}"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_GAZEBO_ROOT="/home/reventon/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
BAYLANDS_URL="http://models.gazebosim.org/baylands/model.tar.gz"
BAYLANDS_SHA256="18bce28a1ae89a146b57218653d773ed432d4bd381252ffb19102af98f7e952c"
BAYLANDS_ARCHIVE="${BAYLANDS_ARCHIVE:-}"

temp_dir="$(mktemp -d)"
trap 'rm -rf -- "${temp_dir}"' EXIT
archive="${temp_dir}/baylands-model.tar.gz"

docker start "${CONTAINER}" >/dev/null
if [[ -n "${BAYLANDS_ARCHIVE}" ]]; then
    cp -- "${BAYLANDS_ARCHIVE}" "${archive}"
else
    curl --fail --location --output "${archive}" "${BAYLANDS_URL}"
fi
printf '%s  %s\n' "${BAYLANDS_SHA256}" "${archive}" | sha256sum --check --status

docker exec "${CONTAINER}" mkdir -p \
    "${PX4_GAZEBO_ROOT}/models" \
    "${PX4_GAZEBO_ROOT}/models/fpv_cam" \
    "${PX4_GAZEBO_ROOT}/models/iris_fpv_cam" \
    "${PX4_GAZEBO_ROOT}/models/vio_imu" \
    "${PX4_GAZEBO_ROOT}/models/vio_baylands" \
    "${PX4_GAZEBO_ROOT}/worlds"
docker cp "${archive}" "${CONTAINER}:/tmp/project_f-baylands-model.tar.gz"
docker exec "${CONTAINER}" tar -xzf /tmp/project_f-baylands-model.tar.gz \
    -C "${PX4_GAZEBO_ROOT}/models" --wildcards \
    'baylands/model.config' \
    'baylands/media/baylands_01.DAE' \
    'baylands/media/baylands_02.DAE' \
    'baylands/media/baylands_03.DAE' \
    'baylands/media/scripts/*' \
    'baylands/media/Textures/*'
docker exec "${CONTAINER}" rm /tmp/project_f-baylands-model.tar.gz
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_models/vio_baylands/." \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/models/vio_baylands/"
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_models/fpv_cam/." \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/models/fpv_cam/"
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_models/iris_fpv_cam/." \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/models/iris_fpv_cam/"
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_models/vio_imu/." \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/models/vio_imu/"
docker cp "${PROJECT_ROOT}/src/VIO_pkg/gazebo_worlds/vio_baylands.world" \
    "${CONTAINER}:${PX4_GAZEBO_ROOT}/worlds/vio_baylands.world"

docker exec "${CONTAINER}" test \
    -f "${PX4_GAZEBO_ROOT}/models/baylands/media/baylands_01.DAE"
docker exec "${CONTAINER}" test \
    -f "${PX4_GAZEBO_ROOT}/models/baylands/media/baylands_02.DAE"
docker exec "${CONTAINER}" test \
    -f "${PX4_GAZEBO_ROOT}/models/baylands/media/baylands_03.DAE"
docker exec "${CONTAINER}" test \
    -f "${PX4_GAZEBO_ROOT}/models/vio_baylands/model.sdf"
docker exec "${CONTAINER}" test \
    -f "${PX4_GAZEBO_ROOT}/worlds/vio_baylands.world"

echo "Installed pinned Baylands assets and vio_baylands into ${CONTAINER}."
