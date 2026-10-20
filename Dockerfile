FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
    ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions python3-pytest python3-numpy python3-yaml \
    python3-pil python3-rasterio python3-pyproj python3-requests \
    libegl1-mesa-dev libgl1-mesa-dri mesa-utils git \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspaces/ROS-2-URC
COPY . .
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install \
    && source install/setup.bash && python3 tools/ros_construct.py
ENV ROS_DOMAIN_ID=27
ENV LIBGL_ALWAYS_SOFTWARE=1
CMD ["bash"]
