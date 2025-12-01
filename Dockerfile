# Use ROS2 Jazzy base image
FROM ros:jazzy-ros-base

# Install pip + colcon
RUN apt-get update && \
    apt-get install -y python3-pip python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Workspace
WORKDIR /ros2_ws

# Copy entire workspace
COPY ros2_ws/src ./src

# Install Python requirements (if any)
RUN if [ -f src/multisensor_car/requirements.txt ]; then \
        pip3 install -r src/multisensor_car/requirements.txt; \
    fi

# Build ROS packages
RUN bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Source environment on startup
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

# Default command (run your node)
CMD bash -c "source /opt/ros/jazzy/setup.bash && \
             source /ros2_ws/install/setup.bash && \
             ros2 run multisensor_car multi_sensor_node"

