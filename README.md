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

### Prerequisites

- Docker installed on your system.
- ROS2 Jazzy environment (provided via Docker image).

### Running the Simulation

1. Build the Docker image:

```bash
docker build -t multi_sensor_car_sim .
````

2. Run the container:

```bash
docker run -it --name multi_sensor_car_sim multi_sensor_car_sim:latest
```

3. Inside the container, check the active ROS2 nodes:

```bash
ros2 node list
```

4. Check available topics:

```bash
ros2 topic list
```

5. Subscribe to a topic to view sensor data:

```bash
ros2 topic echo /battery_state
```

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

GitHub Actions workflow `ci.yml`:

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
