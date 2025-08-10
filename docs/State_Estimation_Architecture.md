# F1TENTH Roboracer – State Estimation Architecture

## Overview

This document describes the design and implementation of a state estimation architecture for the F1TENTH autonomous vehicle. The system fuses data from multiple onboard sensors to produce a reliable and consistent estimate of the vehicle's pose and velocity in real time. The estimated state is a critical input to control, planning, and decision-making modules.

---

## State Variables to be Estimated
*_[Content from previous section — e.g., position, orientation, linear and angular velocity, acceleration, biases]_*

---

## Available Sensor Inputs
*_[Content from previous section — list of available sensors and their capabilities]_*

---

## Selected Estimation Algorithm
*_[Content from previous section — algorithm choice and justification]_*

---

## Modular State Estimation Pipeline

The state estimation system is built as a modular ROS-based pipeline using the **Extended Kalman Filter (EKF)** implementation from the [`robot_localization`](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package. The architecture supports asynchronous sensor updates, incorporates robust sensor fusion techniques, and outputs the estimated state at a fixed rate.

### Key Features
- **Asynchronous Sensor Handling** – Each sensor operates at its own rate; the EKF handles delayed and out-of-order messages.
- **Robust Sensor Fusion** – High-rate wheel odometry and IMU data are fused for smooth, drift-resistant local tracking, with the option to integrate low-rate global corrections from LiDAR-based SLAM.
- **Fixed-Rate Output** – The EKF publishes a fused state estimate at a constant frequency (50–100 Hz) to ensure predictable performance for downstream modules.
- **Planar Motion Mode** – Configured for two-dimensional operation, estimating only x, y, and yaw to match the vehicle’s kinematics.

### Pipeline Components
1. **Sensor Drivers**
   - **VESC 6 MkVI** → `vesc_driver` + `vesc_to_odom` → `/odom`
   - **IMU (SparkFun OpenLog Artemis)** → custom/rosserial node → `/imu/data_raw`
   - **LiDAR (Hokuyo UST-10LX)** → `urg_node` → `/scan` → (optional SLAM node) `/slam_out_pose`
   - **Camera (Intel RealSense D435)** → `realsense2_camera` (reserved for future visual odometry integration)

2. **Fusion Node**
   - `ekf_localization_node` (robot_localization package)
   - Inputs:
     - `/odom` (wheel odometry) – provides forward velocity and relative position changes
     - `/imu/data_raw` (IMU) – provides angular velocity and linear accelerations
     - `/slam_out_pose` (LiDAR pose, optional) – provides absolute position for global correction
   - Outputs:
     - `/odometry/filtered` (`nav_msgs/Odometry`)
     - TF transform `odom → base_link`

3. **Output Consumers**
   - **Control module** – reads velocity and pose for trajectory following
   - **Local planner** – uses fused pose and velocity for path tracking
   - **Finite State Machine (FSM)** – bases state transitions on accurate motion data

---

## Input and Output Interfaces

### Inputs to EKF
| Source | ROS Topic | Message Type | Variables Used |
|--------|-----------|--------------|----------------|
| Wheel Odometry (VESC) | `/odom` | `nav_msgs/Odometry` | x, y, yaw (optional), vx, vy (optional), yaw_rate |
| IMU (Artemis) | `/imu/data_raw` | `sensor_msgs/Imu` | yaw_rate, linear_accel_x, linear_accel_y |
| LiDAR Pose (optional) | `/slam_out_pose` | `geometry_msgs/PoseStamped` | x, y, yaw |

### Outputs from EKF
| ROS Topic | Message Type | Description |
|-----------|--------------|-------------|
| `/odometry/filtered` | `nav_msgs/Odometry` | Fused pose and velocity in the odom frame |
| `/tf` | TF transform | Transformation from `odom` to `base_link` |

---

## Integration with Other Modules
- **Control** – subscribes to `/odometry/filtered` for precise velocity tracking.
- **Planning** – consumes fused pose and velocity for local trajectory generation.
- **FSM** – uses the estimated state for decision-making and safety logic.
- **Visualization & Debugging** – monitored in RViz and `rqt_plot` for validation.

---

## Architecture Diagram

```mermaid
flowchart TD
  subgraph Sensors
    VESC[VESC 6 MkVI (vesc_driver + vesc_to_odom)]
    IMU[IMU - SparkFun Artemis (custom/rosserial)]
    LIDAR[Hokuyo UST-10LX (urg_node)]
    CAM[RealSense D435 (realsense2_camera)]
  end

  subgraph Preprocessing
    ODOM[/odom (nav_msgs/Odometry)/]
    IMUDATA[/imu/data_raw (sensor_msgs/Imu)/]
    SCAN[/scan (sensor_msgs/LaserScan)/]
    VO[/vo_odom (nav_msgs/Odometry)/]
  end

  subgraph Optional_SLAM
    SLAMNODE[SLAM Node (Hector/Cartographer)]
    SLAMPOSE[/slam_out_pose (geometry_msgs/PoseStamped)/]
  end

  subgraph Fusion
    EKF[ekf_localization_node (robot_localization)]
  end

  subgraph Outputs
    FILTERED[/odometry/filtered (nav_msgs/Odometry)/]
    TF[TF: odom->base_link]
  end

  VESC --> ODOM
  IMU --> IMUDATA
  LIDAR --> SCAN --> SLAMNODE --> SLAMPOSE
  CAM --> VO

  ODOM --> EKF
  IMUDATA --> EKF
  SLAMPOSE --> EKF
  VO --> EKF

  EKF --> FILTERED
  EKF --> TF
```

---

## Future Extensions

Planned evolution paths (deferred until the baseline EKF fusion is fully validated):

- Unscented Kalman Filter (UKF): We can later trial `ukf_localization_node` to better handle stronger nonlinear vehicle dynamics (slip, aggressive steering). This will require additional tuning effort (process / measurement covariances, sigma point spread parameters) and possibly an expanded state.
- GPS Integration: Add a GNSS (ideally RTK) receiver (`nmea_navsat_driver` + `navsat_transform_node`) to provide low‑rate absolute pose / velocity for drift correction when outdoors.
- Visual Odometry (VO): Leverage the existing RealSense camera (or a dedicated tracking camera) with VO/SLAM (e.g., ORB-SLAM2, RTAB-Map, VINS-Fusion) to publish a `/vo_odom` topic for indoor / GPS-denied operation.

Each added source increases configuration and covariance tuning complexity (time sync, frame alignment, outlier rejection). We will introduce them incrementally once quantitative EKF performance metrics (pose RMSE, innovation consistency, latency) are established.
