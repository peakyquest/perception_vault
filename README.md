# **Perception Vault – Unified Sensor Intelligence for Autonomous Systems**

**Perception Vault** is a comprehensive and extensible perception framework designed to process the full spectrum of sensor data used in modern robotics and autonomous systems. As autonomous platforms continue to evolve—mobile robots, drones, industrial manipulators, autonomous vehicles—the accuracy and robustness of their perception layer remain the foundation for all navigation, mapping, and decision-making.

This repository serves as a unified hub that ingests, processes, fuses, and interprets raw sensor information from diverse modalities, transforming noisy real-world signals into structured, meaningful insights.

At its core, *Perception Vault* embraces the principle that **no single sensor is enough**. Each type offers unique advantages:

- LiDAR → geometric precision  
- Cameras → rich visual texture  
- Radar → robustness in harsh conditions  
- IMU → high-frequency motion awareness  

By combining these streams, the system delivers a resilient and complete understanding of the environment—capable of handling noise, occlusions, lighting variation, high-speed motion, and more.

The architecture includes modular processing pipelines for each sensor type, multi-sensor fusion blocks, and interfaces for higher-level perception tasks such as SLAM, object detection, 3D clustering, tracking, and environment modeling. Designed for extensibility, it supports research, development, and production deployment.

---

# **Perception Vault Packages**

- **Camera RTSP Streamer**  
  Streams camera feeds over RTSP into ROS2 for monitoring, visualization, and integration into perception pipelines.

- **LiDAR PointCloud Filters**  
  A set of PCL-based filters for point cloud preprocessing (downsampling, noise filtering, segmentation), improving performance and accuracy of downstream algorithms.

- **Navsat Pose Localizer**  
  Converts raw GPS latitude/longitude into ROS2 `nav_msgs/Odometry`, enabling integration into mapping and navigation frameworks.

- **Lidar Euclidean Cluster**  
  Uses PCL’s Euclidean clustering to segment raw LiDAR point clouds into distinct object-level clusters.

- **Camera Image Processing**  
  Provides essential image preprocessing operations such as undistortion, resizing, ROI extraction, and enhancement. Prepares images for tasks like SLAM, detection, and pixel-to-pointcloud fusion.

- **ORB-SLAM2 Wrapper**  
  ROS2 wrapper for ORB-SLAM2, enabling real-time visual SLAM (pose estimation, mapping, relocalization) from monocular, stereo, or RGB-D cameras.

- **Pixel Cloud Fusion**  
  Performs 2D–3D sensor fusion by projecting camera detections (e.g., YOLO bounding boxes) onto LiDAR point clouds. Outputs 3D clusters corresponding to detected objects—ideal for tracking and semantic understanding.

---

# **Third-Party Integrated Packages**

- **YOLO ROS2 Package**  
  The [YOLO ROS2](https://github.com/mgonzs13/yolo_ros) package by *mgonzs13* integrates YOLOv8–YOLOv12 into ROS2 for real-time object detection suitable for mobile robots and autonomous systems.

- **3D LiDAR SLAM ROS2 Package**  
  The [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) package by *rsasaki0109* provides efficient scan-matching and graph-based SLAM to build accurate 3D maps and estimate robot pose in real time.

---

# **Bag Files (Dataset & Playback Support)**

This section provides curated ROS2 bag files to test and benchmark Perception Vault modules without requiring live hardware.

- **Autoware ROS2 Bag Files**  
  https://autowarefoundation.github.io/autoware-documentation/main/datasets/

- **KITTI Publisher ROS2**  
  https://github.com/umtclskn/ros2_kitti_publishers

---
