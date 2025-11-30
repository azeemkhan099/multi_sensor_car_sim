#!/usr/bin/env bash
set -e

docker build -t multi_sensor_car_sim:latest .

docker run --rm -it \
  --network host \
  --name multi_sensor_car_sim \
  multi_sensor_car_sim:latest
