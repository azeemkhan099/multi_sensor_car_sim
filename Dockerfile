# Use ROS2 Jazzy base image
FROM ros:jazzy-ros-base

# Install Python pip and colcon
RUN apt-get update && \
    apt-get install -y python3-pip python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy workspace into container
COPY ./ros2_ws /ros2_ws

# Install Python requirements and build
RUN pip3 install -r /ros2_ws/src/multi_sensor_car_sim/requirements.txt || true
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"


SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc


# Default command to run node
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run multi_sensor_car_sim sensor_publisher"]
