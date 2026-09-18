FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/ros_ws

# ============================================================
# Ubuntu mirror
# ============================================================
RUN sed -i \
        -e 's|archive.ubuntu.com/ubuntu|it.archive.ubuntu.com/ubuntu|g' \
        -e 's|security.ubuntu.com/ubuntu|it.archive.ubuntu.com/ubuntu|g' \
        /etc/apt/sources.list.d/ubuntu.sources

# ============================================================
# System + ROS dependencies
# ============================================================
RUN apt-get update \
        -o Acquire::ForceIPv4=true \
        -o Acquire::http::Timeout=30 \
        -o Acquire::https::Timeout=30 \
        -o Acquire::Retries=3 \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        nano \
        vim \
        usbutils \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-opencv \
        python3-numpy \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-tf2 \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-rqt-image-view \
        ros-${ROS_DISTRO}-image-tools \
        qtwayland5 \
        libqt5waylandclient5 \
        libqt5waylandcompositor5 \
        ros-${ROS_DISTRO}-librealsense2 \
        ros-${ROS_DISTRO}-realsense2-camera \
        ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-diagnostic-updater \
        libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Python packages
# Install directly in the Python environment used by ROS 2
# ============================================================
RUN python3 -m pip install \
        --no-cache-dir \
        --break-system-packages \
        "numpy==1.26.4" \
        "mediapipe==0.10.21" \
        scipy

# ============================================================
# rosdep
# ============================================================
RUN rosdep init 2>/dev/null || true

# ============================================================
# Workspace
# ============================================================
WORKDIR ${WORKSPACE}

RUN mkdir -p ${WORKSPACE}/src

# ============================================================
# Bash environment
# ============================================================
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /root/.bashrc

CMD ["bash"]
