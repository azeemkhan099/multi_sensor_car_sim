# Multi-Sensor Car Simulation (ROS2)

A ROS2-based multi-sensor car simulation project demonstrating integration of multiple sensors, publishing real-time data, and modular node-based architecture.

---

## Project Overview

This project simulates a car equipped with multiple sensors including:

- **Battery sensor**: Monitors battery voltage.
- **LIDAR sensor**: Provides environmental distance measurements.
- **Wheel speed sensor**: Reports vehicle speed.

The simulation is implemented as a ROS2 package with a single node (`multi_sensor_node`) that publishes sensor data on separate topics. The project is designed for modularity and easy extension for additional sensors or visualization tools.

---

## Features

- ROS2 (Jazzy) Python package for multi-sensor simulation.
- Dockerized environment for easy deployment and reproducibility.
- Sensor data publishing on dedicated topics:
  - `/battery_state`
  - `/lidar_front`
  - `/wheel_speed`
- Launch file to start simulation.
- CI/CD workflow for automated testing using GitHub Actions.

---

## Repository Structure

```

multisensor_car_sim/
├── Dockerfile
├── run.sh
├── ros2_ws/
│   └── src/
│       └── multi_sensor_car_sim/
│           ├── launch/
│           │   └── sim.launch.py
│           ├── multi_sensor_car_sim/
│           │   ├── **init**.py
│           │   └── sensor_publisher.py
│           ├── package.xml
│           ├── resource/
│           │   └── multi_sensor_car_sim
│           ├── setup.cfg
│           ├── setup.py
│           └── test/
│               ├── test_copyright.py
│               ├── test_flake8.py
│               └── test_pep257.py

````

---

## Getting Started

### Requirements

- Docker
- ROS2 Jazzy base image
- Python3

---

### Step 1: Clone the Repository

```bash
git clone <your-repo-url>
cd multisensor_car_sim
````

---

### Step 2: Build the Docker Image

```bash
docker build -t multi_sensor_car_sim:latest .
```

This will:

* Use ROS2 Jazzy as the base image
* Install Python pip and colcon
* Copy your workspace into `/ros2_ws`
* Install Python dependencies from `requirements.txt`
* Build the ROS2 package

---

### Step 3: Run the Container

```bash
docker run -it --name multi_sensor_car_sim multi_sensor_car_sim:latest bash
```

Inside the container, source the ROS2 and workspace setups:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

---

### Step 4: Run the Node

```bash
ros2 run multi_sensor_car_sim sensor_publisher
```

---

### Step 5: Verify Running Nodes and Topics

Check active nodes:

```bash
ros2 node list
```

Expected output:

```
/multi_sensor_node
```

Check active topics:

```bash
ros2 topic list
```

Expected output:

```
/battery_state
/lidar_front
/parameter_events
/rosout
/wheel_speed
```

---

### Step 6: Observe Published Sensor Data

Once the node runs, you will see output like:

```
[INFO] [multi_sensor_node]: Published battery: 12.31V, lidar ranges: 314, wheel speed: 2.71 m/s
[INFO] [multi_sensor_node]: Published battery: 12.45V, lidar ranges: 314, wheel speed: 1.68 m/s
[INFO] [multi_sensor_node]: Published battery: 12.33V, lidar ranges: 314, wheel speed: 2.45 m/s
[INFO] [multi_sensor_node]: Published battery: 11.58V, lidar ranges: 314, wheel speed: 4.89 m/s
```

**Explanation:**

* `battery` → simulated battery voltage in Volts
* `lidar ranges` → number of points measured by the LIDAR (unitless count)
* `wheel speed` → simulated wheel speed in meters/second

This is the **final outcome of the project**.

---

## ROS2 Nodes and Topics

| Node Name           | Topics Published | Description                         |
| ------------------- | ---------------- | ----------------------------------- |
| `multi_sensor_node` | `/battery_state` | Battery voltage readings            |
|                     | `/lidar_front`   | LIDAR scan data (distance readings) |
|                     | `/wheel_speed`   | Vehicle speed                       |

---

## Docker Setup

* **Base Image:** `ros:jazzy-ros-base`
* Dependencies installed: `python3-pip`, `colcon-common-extensions`
* ROS2 workspace copied into container (`ros2_ws`)
* Python requirements installed from `requirements.txt`
* ROS2 package built using `colcon build`
* Node automatically launched on container start (`sensor_publisher`)

---

## CI/CD Pipeline

GitHub Actions workflow (`ci.yml`):

* Runs tests on each push to the `main` branch.
* Builds Docker image.
* Validates that ROS2 package is buildable and node runs.

---

## Notes

* LIDAR data publishes raw distance points (number of measurements per scan: 314). Units are meters.
* Battery readings are in volts.
* Wheel speed readings are in meters per second.

---

## Future Improvements

* Add visualization using RViz2 or Gazebo.
* Add more sensors (GPS, IMU).
* Implement autonomous navigation simulation.
* Automate using Jenkins
