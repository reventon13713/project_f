FROM --platform=$BUILDPLATFORM mambaorg/micromamba:2.3.3@sha256:800e7ade3ffe29c9a9ac2026163131495f8197c3852e572c5835beb4e8a33cd6 AS simulator
USER root
COPY gazebo-linux-aarch64.lock /tmp/gazebo.lock
RUN --mount=type=cache,target=/tmp/city-mamba \
    MAMBA_ROOT_PREFIX=/tmp/city-mamba micromamba create --yes --always-copy \
        --platform linux-aarch64 --prefix /opt/city-gazebo --file /tmp/gazebo.lock

FROM reventon13713/project_f:2026-11-07@sha256:571ad4c305440b8fc002da25d68044ea90b99d4222bf780e895d79b13437dfda
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        xvfb xauth libgl1-mesa-dri && rm -rf /var/lib/apt/lists/*
COPY --from=simulator /opt/city-gazebo /opt/city-gazebo
# Gazebo registers its own shader directory. OGRE 1.10 supplies the alpha-test
# shader in its separate media directory.
RUN ln -s /opt/city-gazebo/share/OGRE/Media/RTShaderLib/GLSL/FFPLib_AlphaTest.glsl \
          /opt/city-gazebo/share/gazebo-11/media/rtshaderlib/FFPLib_AlphaTest.glsl
WORKDIR /opt/project_f/simulation/city_vio
COPY . ./
ARG SOURCE_REVISION
LABEL org.opencontainers.image.revision=$SOURCE_REVISION
CMD ["bash"]
