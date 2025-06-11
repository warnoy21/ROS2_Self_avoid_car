# ROS2_Self_Avoid_Car

This project demonstrates a **self-avoiding robotic car** using **ROS 2**, simulated in **Gazebo**, with real-time motor control handled by an **Arduino Portenta X8**. It bridges simulation and physical systems using ROS 2 messaging and real hardware.

## 🧠 Project Overview

- **Simulation**: Gazebo running on Steam Deck
- **Sensors**: Simulated 2D LiDAR
- **Robot Base**: Differential drive car
- **Real-World Execution**: Motor control via Portenta X8
- **Communication**: ROS 2 Humble over LAN

## ⚙️ Architecture

1. **Gazebo (Steam Deck)**:
   - Publishes `/scan`
   - Subscribes to `/cmd_vel`
2. **Portenta X8**:
   - Receives `/cmd_vel`
   - Converts to real motor PWM signals using STM32 side
3. **ROS 2 Bridge**:
   - DDS-based communication (ROS_DOMAIN_ID and ROS_IP used)

## 🐳 Deployment

### On **Steam Deck** (simulation + ROS 2 nodes):

```bash
docker run -it --rm \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_IP=192.168.0.21 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  self_avoid_car
### On **Portenta X8 ** () ROS 2 nodes):
```bash
docker run -it --rm \
  --privileged \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_IP=192.168.0.45 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  self_avoidance

