## ORB SLAM2 Wrapper 

`orb_slam2_wrapper` is a ROS 2 package that wraps the original ORB-SLAM2 system and the appliedAI ROS integration for modern ROS 2 distributions.  
It provides **monocular**, **stereo**, and **RGB-D** SLAM nodes that estimate camera pose and publish a sparse 3D map.

- **Upstream SLAM implementation**: [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)  
- **Upstream ROS integration**: [appliedAI-Initiative/orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)  

This wrapper keeps the same high‑level interfaces as the appliedAI ROS 1 node but is adapted and extended for ROS 2 (Humble, Jazzy).

## What is modified

- **ROS 2 compatibility**: The original `orb_slam_2_ros` targets ROS 1; this wrapper ports the nodes to ROS 2 (Humble, Jazzy) using `rclcpp`, ROS 2 parameters, and ROS 2 topics/services.
- **Dual map saving**: When the `save_map` service is called, the map is saved both as a binary ORB-SLAM2 map (`map.bin`, in the current working directory) and as an ASCII PCD file (`map.pcd`, in `~/.ros`) that can be consumed by other tools.
- **Consistent point cloud convention**: The exported `map.pcd` uses the same coordinate convention as the published `PointCloud2` map topic for easy downstream use.

### Inputs / Outputs

| Type | Topic Name | Description |
|------|------------|-------------|
| Input | `/camera/image_raw` | Monocular camera image stream (`sensor_msgs/msg/Image`) |
| Input | `/camera/camera_info` | Camera intrinsics and distortion parameters (`sensor_msgs/msg/CameraInfo`) |
| Output | `/orb_slam2_mono_node/map_points` | Sparse 3D map points published by ORB-SLAM2 (`sensor_msgs/msg/PointCloud2`) |
| Output | `/orb_slam2_mono_node/pose` | Current camera pose in the map frame (`geometry_msgs/msg/PoseStamped`) |
| Output | `/orb_slam2_mono_node/debug_image` | Debug image with tracked ORB features and status text (`sensor_msgs/msg/Image`) |
| Output | TF (`map_frame_id` → `camera_frame_id`) | Transform from map frame to camera frame published via TF2 |

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `publish_pointcloud` | `true` | Publish 3D point cloud for the reconstructed map |
| `publish_pose` | `true` | Publish camera pose as `PoseStamped` |
| `localize_only` | `false` | If `true`, only localize in an existing map (no new map points) |
| `reset_map` | `false` | If `true` at startup, clear any existing map |
| `load_map` | `false` | Load a previously saved map from `map_file` on startup |
| `map_file` | `map.bin` | File name used to load/save the serialized ORB-SLAM2 map |
| `pointcloud_frame_id` | `map` | Frame ID used for the published map point cloud |
| `camera_frame_id` | `camera_link` | Frame ID associated with the input camera |
| `min_num_kf_in_map` | `5` | Minimum number of keyframes before treating the map as valid |
| `ORBextractor/nFeatures` | `5000` | Number of ORB features per frame |
| `ORBextractor/scaleFactor` | `1.2` | Scale factor between pyramid levels |
| `ORBextractor/nLevels` | `8` | Number of pyramid levels |
| `ORBextractor/iniThFAST` | `8` | Initial FAST threshold |
| `ORBextractor/minThFAST` | `3` | Minimum FAST threshold |
| `camera_fps` | `20` | Expected camera frame rate in Hz |
| `camera_rgb_encoding` | `true` | Whether incoming images use RGB encoding (`true`) or BGR (`false`) |


### How It Works

- The node starts and loads ROS 2 parameters (camera intrinsics, ORB parameters, map file, etc.) from the configured YAML file.  
- It subscribes to `/camera/image_raw` for the monocular image stream and `/camera/camera_info` for camera calibration.  
- Upon receiving the first `CameraInfo` message, it initializes the internal ORB-SLAM2 `System` in monocular mode with the given intrinsics and configuration.  
- For each incoming image frame:
  - The `sensor_msgs/msg/Image` message is converted to OpenCV's `cv::Mat` format via `cv_bridge`.
  - ORB-SLAM2's `TrackMonocular()` method is called with the image and timestamp to process the frame.
  - ORB features are extracted, matched, and used to estimate camera pose and update the sparse 3D map.
- After each tracking update, the wrapper:
  - Publishes the current camera pose as a TF transform (from map frame to camera frame) and optionally as a `PoseStamped` message.
  - Publishes the current set of map points as a `PointCloud2` message (filtering points based on minimum observation thresholds).
  - Publishes a debug image showing tracked ORB keypoints, inliers, and tracking status text.
- When the `save_map` service is called:
  - ORB-SLAM2 serializes the map to a binary file (e.g. `map.bin`) in the current working directory.
  - The wrapper also exports the same map points to an ASCII PCD file (e.g. `map.pcd`) stored under `~/.ros`, using the same coordinate convention as the published `PointCloud2` message.

## Build and Install

- Tested on **Ubuntu 22.04** with **ROS 2 Humble**; intended to work with **ROS 2 Jazzy** as well.

From the root of your ROS 2 workspace:

```bash
cd <your_ros2_ws>
colcon build
source install/setup.bash
```

Ensure dependencies such as OpenCV, Eigen3, and standard ROS 2 message packages are installed (see `Dependencies.md`).

### Launching the Node

The node can be launched using the provided launch file with customizable parameters:

```bash
ros2 launch orb_slam2_wrapper orb_slam2_mono_launch.py
```

You can override default parameters via launch arguments:

```bash
ros2 launch orb_slam2_wrapper orb_slam2_mono_launch.py \
    params_file:=/path/to/params.yaml \
    voc_file:=/path/to/vocabulary.txt
```

After launching, you should see:
- Map point cloud on `/orb_slam2_mono_node/map_points`.
- Pose on `/orb_slam2_mono_node/pose`.
- Debug image on `/orb_slam2_mono_node/debug_image`.


The image below illustrates the operation of ORB-SLAM integrated with the AirSim simulation environment in car mode. ORB-SLAM processes the front-camera feed from the simulated vehicle to extract ORB features, track them across frames, and estimate the camera pose in real time. As the vehicle moves through the virtual world, the system constructs a consistent sparse 3D map while simultaneously localizing itself within it. This setup allows rapid testing and validation of SLAM algorithms in a controlled, photorealistic simulation without the need for real-world sensors or hardware

<img width="1846" height="1014" alt="image" src="https://github.com/user-attachments/assets/9c34c5ed-32ff-4b88-a4eb-715651151331" />


## Saving Maps (`.bin` and `.pcd`)

Each node exposes a `save_map` service (e.g. `/orb_slam2_mono_node/save_map`).  
To save the current map as both **binary** and **PCD**:

```bash
ros2 service call /orb_slam2_mono_node/save_map orb_slam2_wrapper/srv/SaveMap "{name: 'map.bin'}"
```

This will create:
- `map.bin` – ORB-SLAM2 binary map file in the current working directory.
- `map.pcd` – ASCII PCD file with the same map points, written to `~/.ros`.

Both files can be reused for localization, analysis, or visualization in third‑party tools.

---
## NOTE

If you use ORB-SLAM2 in academic work, please cite the original authors as described in the upstream ORB-SLAM2 repository.
