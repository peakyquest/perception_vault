# Pixel Cloud Fusion

`pixel_cloud_fusion` is a ROS2 package designed to fuse camera image data with LiDAR pointclouds. The package projects images onto the pointcloud, keeping only the points that are visible in the camera's field of view. This fusion allows 2D detections (e.g., from YOLO) to be accurately visualized in 3D, providing enhanced spatial context for perception tasks.

---

## Overview

The main goal of this package is to create a unified perception output by combining 2D image information with 3D LiDAR data.

### Key Features

- Projects camera images onto the pointcloud using calibration and TF transforms.
- Filters pointcloud points to retain only those visible in the camera.
- Maps YOLO object detections from the image to the 3D pointcloud.
- Publishes **visualization markers** for detected objects in RViz.
- Compatible with ROS2 standard message types.

---

## How It Works

The fusion pipeline operates as follows:

- The node receives synchronized **camera image**, **camera calibration**, and **LiDAR pointcloud** topics.
- Using TF transforms, the LiDAR points are projected into the camera frame.
- Points that lie within the camera’s valid pixel range are retained, creating an **image-aligned pointcloud**.
- YOLO detections from the image are projected into 3D space.
- Instead of creating a new pointcloud for detections, the node publishes **MarkerArray** objects for visualization.
- The final output includes:
- A pointcloud filtered to the camera's field of view.
- Markers representing detected objects, making them visible in 3D.

---

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Raw camera image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic parameters for projection |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Input LiDAR pointcloud |
| `/camera/yolo_detections` | `vision_msgs/msg/Detection2DArray` | 2D object detections from YOLO |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fusion/points_projected` | `sensor_msgs/msg/PointCloud2` | Pointcloud filtered to camera FOV |
| `/fusion/detections_markers` | `visualization_msgs/msg/MarkerArray` | 3D visualization markers representing YOLO-detected objects |

---

## Dependencies

Make sure the following dependencies are included in your `package.xml` and `CMakeLists.txt`:

- `rclcpp`
- `sensor_msgs`
- `vision_msgs`
- `visualization_msgs`
- `cv_bridge`
- `image_geometry`
- `tf2` / `tf2_ros`
- `OpenCV`

---

## Future Enhancements

- Multi-camera to multi-LiDAR fusion
- Depth estimation using hybrid camera + LiDAR data
- Colorizing pointcloud using camera pixels
- 3D bounding box generation for detected objects