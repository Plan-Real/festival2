# ROS distribution to use
ARG ROS_DISTRO=humble

###########################
# Base Image for festival #
###########################
FROM nvidia/cudagl:11.1.1-base-ubuntu20.04 as gpu
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ARG REPO
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Install basic apt packages
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
    build-essential \
    ccache \
    cmake \
    lld \
    python3-colcon-common-extensions \
    python3-colcon-coveragepy-result \
    python3-colcon-mixin \
    git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libqt5svg5-dev \
    python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python modules
RUN pip3 install  transforms3d \
    matplotlib \
    torch

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-realsense2-camera
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create Colcon workspace with external dependencies
RUN mkdir -p /upstream_ws/src
WORKDIR /upstream_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /upstream_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --mixin release lld

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /upstream_ws
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

##############################
# Overlay Image for festival #
##############################
FROM base AS overlay

ARG SIM_REPO
ARG SIM_REPO_ENV


RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libxcursor-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxi-dev \
    mesa-common-dev  \
    zip \
    unzip \
    make \
    gcc-8 \
    g++-8 \
    vulkan-utils   \
    mesa-vulkan-drivers  \
    pigz  \
    git \
    libegl1 \
    git-lfs

# Force gcc 8 to avoid CUDA 10 build issues on newer base OS
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

# WAR for eglReleaseThread shutdown crash in libEGL_mesa.so.0 (ensure it's never detected/loaded)
# Can't remove package libegl-mesa0 directly (because of libegl1 which we need)
RUN rm /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0 /usr/lib/x86_64-linux-gnu/libEGL_mesa.so.0.0.0 /usr/share/glvnd/egl_vendor.d/50_mesa.json

# Create an sim workspace
WORKDIR /opt/sim
COPY ./${SIM_REPO}/ ./${SIM_REPO}/
COPY ./${SIM_REPO_ENV}/ ./${SIM_REPO_ENV}/
ENV PATH="/opt/sim/.local/bin:$PATH"
RUN mv ./${SIM_REPO}/docker/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json \
    && mv ./${SIM_REPO}/docker/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    && cd /opt/sim/${SIM_REPO}/python && pip install -q -e . \
    && cd /opt/sim/${SIM_REPO_ENV} && pip install -e .

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./${REPO}/ ./src/${REPO}/
RUN source /upstream_ws/install/setup.bash
# && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y


# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay as dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=${UID}
ARG GID=${GID}

# Install extra tools for development
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    add-apt-repository ppa:sdurobotics/ur-rtde -y  \
    && apt-get update && apt-get install -y --no-install-recommends \
    gdb gdbserver nano \
    git \
    openssh-client \
    python3-pip \
    vim \
    wget \
    librtde librtde-dev \
    && rm -rf /var/lib/apt/lists/*

# install developer tools
RUN python3 -m pip install --no-cache-dir \
    pre-commit==3.0.4 \
    ur_rtde \
    flask \
    flask_socketio \
    ultralytics \
    pyrealsense2

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
    && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && mkdir -p /home/${USERNAME} \
    && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/ \
    && chown -R ${UID}:${GID} /opt/sim/

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
