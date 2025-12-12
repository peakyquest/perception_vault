# Camera Image Processing

`camera_image_processing` is a ROS2 package that contains multiple nodes for various image processing tasks. Each node is designed to perform a specific operation on incoming camera data.

Currently, the package includes an **Image Rectifier** node, with more image-processing nodes planned for future development.

---

## Image Rectifier

The **Image Rectifier** node corrects lens distortion and produces a geometrically accurate, rectified image.
It takes a **raw image** and corresponding **camera info** as input and outputs a **rectified image** using the intrinsic and distortion parameters provided by the camera.

### How It Works

The rectification process uses the camera's calibration parameters to remove lens distortion:

- The node receives `camera_info`, which contains the camera matrix (intrinsics), distortion coefficients, and image dimensions.
- Using these parameters, the node computes an **undistortion map** and **rectification transform**.
- Each incoming `image_raw` frame is processed through this undistortion map using OpenCV functions such as `cv::initUndistortRectifyMap` and `cv::remap`.
- The final result is a distortion-free, geometrically corrected **rectified image**, which is published for downstream processing.

---

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Raw input image stream |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration parameters (intrinsics & distortion) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_rectified` | `sensor_msgs/msg/Image` | Output rectified/undistorted image |

---

## Dependencies

Ensure the following dependencies are included in your `package.xml` and `CMakeLists.txt`:

- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `OpenCV`

---

## Future Enhancements

- Additional image-processing nodes
- GPU-accelerated rectification and processing
- Dynamic reconfiguration