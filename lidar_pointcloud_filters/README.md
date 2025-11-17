# LiDAR Point Cloud Filters

This package provides ROS 2 nodes for filtering and processing LiDAR point cloud data using PCL (Point Cloud Library) filters. The package includes three main filtering nodes that can be used individually or together in a processing pipeline.

## Voxel Grid Filter Node

The **VoxelGridNode** is a ROS 2 node that subscribes to a raw `sensor_msgs/PointCloud2` topic and downsamples it using the **PCL VoxelGrid filter**.  
This process reduces the number of points in the cloud while maintaining the overall structure, improving computational efficiency for perception and mapping tasks.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|-------------|-------------|
| **Input** | `/points/raw` | Incoming unfiltered point cloud data |
| **Output** | `/points/voxel_filter` | Downsampled point cloud after applying voxel grid filter |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `leaf_size_x` | `0.1` | Size of each voxel leaf along the X-axis (in meters) |
| `leaf_size_y` | `0.1` | Size of each voxel leaf along the Y-axis (in meters) |
| `leaf_size_z` | `0.1` | Size of each voxel leaf along the Z-axis (in meters) |
| `debug` | `false` | When true, prints the number of raw and downsampled points for debugging |

---

### How It Works

- The node subscribes to a raw LiDAR or point cloud topic.  
- The input point cloud is divided into **3D voxels (cubic regions)** defined by the `leaf_size_x`, `leaf_size_y`, and `leaf_size_z` parameters.  
- All points within each voxel are replaced by their centroid, effectively **reducing the total number of points** while maintaining the geometric shape.  
- The filtered point cloud is then published on `/points/voxel_filter`.  
- If `debug=true`, the node prints the total number of input and output points to help tune the voxel size.

---

## Ground Segmentation Node

The **GroundSegmentation** node is a ROS 2 node that subscribes to a raw `sensor_msgs/PointCloud2` topic and segments the ground points using a **radial grid-based height filter**.  
Points above the local ground surface are extracted and published as a separate point cloud.  
This process is useful for filtering out road or floor points in LiDAR data, improving perception and obstacle detection tasks.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|-------------|-------------|
| **Input** | `/sensing/lidar/top/rectified/pointcloud` | Incoming unfiltered LiDAR point cloud |
| **Output** | `/points/no_ground` | Point cloud containing points above the ground |
| **Debug Ground Points** | `/ground_points` | *(Optional, debug=true)* Point cloud of points classified as ground |
| **Debug Non-Ground Points** | `/no_ground_points` | *(Optional, debug=true)* Point cloud of points above the ground |
| **Grid Markers** | `/grid_markers` | *(Optional, debug=true)* Visualization markers showing radial/concentric grid |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `ground_threshold` | `0.2` | Height threshold above local ground to classify a point as non-ground (meters) |
| `radial_div_num` | `60` | Number of angular divisions in the radial grid |
| `concentric_div_num` | `30` | Number of concentric rings in the radial grid |
| `max_range` | `50.0` | Maximum distance to consider points for processing (meters) |
| `min_range` | `2.0` | Minimum distance to ignore points (e.g., LiDAR mounting points, vehicle body) |
| `debug` | `true` | When true, publishes debug point clouds and visualization markers |

---

### How It Works

- The node subscribes to a LiDAR point cloud (`sensor_msgs/PointCloud2`).  
- It constructs a **radial-concentric grid** around the sensor:  
  - Radial divisions split the 360° space around the sensor.  
  - Concentric divisions split the distance range from `min_range` to `max_range`.  
- For each cell in the grid, the **minimum Z value** is recorded as the local ground height.  
- Points higher than `ground_threshold` above the local ground height are classified as **non-ground**.  
- Optionally, if `debug=true`, the node publishes:  
  - Ground and non-ground point clouds separately.  
  - Visualization markers for the radial grid in RViz.

---

### Visualization in RViz

- Add a **PointCloud2** display and subscribe to `/points/no_ground` to see the filtered points.  
- If `debug=true`, add **PointCloud2** displays for `/ground_points` and `/no_ground_points`.  
- Add a **MarkerArray** display and subscribe to `/grid_markers` to visualize the radial grid used for ground estimation.

---

### Notes

- The `ground_threshold` parameter may need tuning based on sensor height, vehicle height, and terrain.  
- The radial and concentric divisions affect the resolution of the ground grid — higher values increase accuracy but also computational cost.  
- This node is designed for real-time segmentation of LiDAR point clouds in automotive or robotic applications.

---

## Outliers Filter Node

The **OutliersFilters** node is a ROS 2 node that removes **statistical outliers** from point clouds using the **PCL Statistical Outlier Removal (SOR) filter**.  
This is useful for cleaning noisy point clouds after voxel filtering or ground segmentation.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|------------|-------------|
| **Input** | `/points/voxel_filter` | Incoming downsampled point cloud (from voxel filter) |
| **Output** | `/points/inliers` | Filtered point cloud containing inlier points |
| **Output (Debug)** | `/points/outliers` | Optional: cloud containing outlier points (published only if `debug=true`) |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `mean_k` | `50` | Number of nearest neighbors analyzed for each point. Lower values improve performance. |
| `stddev_mul_thresh` | `1.0` | Standard deviation multiplier threshold. Higher values keep more points, lower values remove more. |
| `debug` | `false` | If true, publishes outlier cloud for debugging |

---

### How It Works

- The node subscribes to a point cloud topic (usually the output of the voxel filter).  
- For each point, it computes the mean distance to its `mean_k` nearest neighbors.  
- Points farther than `stddev_mul_thresh` standard deviations from the mean are classified as **outliers**.  
- The filtered inliers are published to `/points/inliers`.  
- If `debug=true`, outliers are also published to `/points/outliers`.

---

### Recommended Settings for Smooth Visualization

| Parameter | Suggested Value | Notes |
|-----------|----------------|-------|
| `mean_k` | 30 | Reduces computation for smoother real-time visualization |
| `stddev_mul_thresh` | 1.5–2.0 | Keeps the inlier cloud dense and visually smooth |
| `debug` | false | Disable outlier publishing to improve performance |

---

### Visualization in RViz

- Add a **PointCloud2** display and subscribe to `/points/inliers` for cleaned point clouds.  
- If `debug=true`, add another **PointCloud2** display for `/points/outliers` to visualize removed points.  
- Avoid publishing both inliers and outliers for large clouds at high frequencies to prevent lag.

---

### Notes

- This node works best **after a voxel grid filter** to reduce the number of points.  
- Proper tuning of `mean_k` and `stddev_mul_thresh` ensures smooth visualization without losing important points.  
- Designed for real-time LiDAR or depth sensor data in robotic and automotive applications.

---

## Launch File: `pcl_filters.launch.py`

The launch file provides a convenient way to launch all three filter nodes simultaneously with configurable parameters. It sets up a complete point cloud processing pipeline that chains the filters together: **Voxel Grid Filter → Ground Segmentation → Outliers Filter**.

### Launch File Overview

The launch file (`launch/pcl_filters.launch.py`) launches three nodes in sequence:

1. **Voxel Grid Node** - Downsamples the input point cloud
2. **Ground Segmentation Node** - Removes ground points (subscribes to voxel filter output)
3. **Outliers Filter Node** - Removes statistical outliers (subscribes to voxel filter output)

### Launch Arguments

The launch file accepts the following arguments, organized by node:

#### Global Arguments

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `use_sim_time` | `true` | Enable/disable simulation time for all nodes |

#### Voxel Grid Filter Arguments

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `voxel_input_topic` | `/cx/lslidar_point_cloud` | Input topic for voxel grid filter |
| `voxel_output_topic` | `/points/voxel_filter` | Output topic for voxel grid filter |
| `leaf_size_x` | `0.1` | Voxel leaf size along X-axis (meters) |
| `leaf_size_y` | `0.1` | Voxel leaf size along Y-axis (meters) |
| `leaf_size_z` | `0.1` | Voxel leaf size along Z-axis (meters) |
| `voxel_debug` | `false` | Enable debug output for voxel grid node |

#### Ground Segmentation Arguments

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `ground_input_topic` | `/points/voxel_filter` | Input topic (typically voxel filter output) |
| `ground_output_topic` | `/points/no_ground` | Output topic for non-ground points |
| `ground_threshold` | `0.2` | Height threshold above ground (meters) |
| `radial_div_num` | `60.0` | Number of angular divisions in radial grid |
| `concentric_div_num` | `30.0` | Number of concentric rings |
| `max_range` | `50.0` | Maximum processing range (meters) |
| `min_range` | `2.0` | Minimum processing range (meters) |
| `ground_debug` | `false` | Enable debug outputs (ground points, markers) |

#### Outliers Filter Arguments

| Argument | Default Value | Description |
|----------|---------------|-------------|
| `outliers_mean_k` | `20` | Number of nearest neighbors for outlier detection |
| `outliers_stddev` | `1.5` | Standard deviation multiplier threshold |
| `outliers_debug` | `false` | Enable outlier point cloud publishing |

### How to Launch

#### Basic Launch (Default Parameters)

```bash
ros2 launch lidar_pointcloud_filters pcl_filters.launch.py
```

This launches all three nodes with default parameters. The pipeline will:
- Subscribe to `/cx/lslidar_point_cloud` (voxel input)
- Publish filtered output to `/points/voxel_filter` (voxel output)
- Publish ground-segmented output to `/points/no_ground` (ground output)

#### Custom Input Topic

To change the input topic for the voxel filter:

```bash
ros2 launch lidar_pointcloud_filters pcl_filters.launch.py \
    voxel_input_topic:=/your/lidar/topic
```

#### Custom Voxel Size

To adjust voxel grid resolution:

```bash
ros2 launch lidar_pointcloud_filters pcl_filters.launch.py \
    leaf_size_x:=0.05 \
    leaf_size_y:=0.05 \
    leaf_size_z:=0.05
```

#### Enable Debug Outputs

To enable debug outputs for all nodes:

```bash
ros2 launch lidar_pointcloud_filters pcl_filters.launch.py \
    voxel_debug:=true \
    ground_debug:=true \
    outliers_debug:=true
```

#### Complete Custom Configuration

Example with multiple custom parameters:

```bash
ros2 launch lidar_pointcloud_filters pcl_filters.launch.py \
    voxel_input_topic:=/sensing/lidar/top/rectified/pointcloud \
    leaf_size_x:=0.08 \
    leaf_size_y:=0.08 \
    leaf_size_z:=0.08 \
    ground_threshold:=0.15 \
    max_range:=60.0 \
    min_range:=1.5 \
    outliers_mean_k:=30 \
    outliers_stddev:=2.0 \
    use_sim_time:=false
```

### Topic Flow

The default topic flow in the launch file is:

```
/cx/lslidar_point_cloud (input)
    ↓
[voxel_grid_node]
    ↓
/points/voxel_filter
    ↓
[ground_segmentation] → /points/no_ground
    ↓
[outliers_filters] → /points/inliers
```

**Note:** The ground segmentation and outliers filter both subscribe to `/points/voxel_filter` by default, but you can configure them to subscribe to different topics if needed.

### Pipeline Configuration Tips

1. **Voxel Grid First**: Always run voxel grid filtering first to reduce point count before ground segmentation and outlier removal.

2. **Topic Chaining**: The default configuration chains topics automatically:
   - Voxel output → Ground input
   - Voxel output → Outliers input (if configured)

3. **Performance Tuning**: 
   - Start with larger voxel sizes (0.1m) for faster processing
   - Reduce voxel size for higher detail (0.05m or smaller)
   - Adjust `outliers_mean_k` based on point density after voxel filtering

4. **Debug Mode**: Enable debug outputs (`*_debug:=true`) to visualize intermediate results in RViz, but disable in production for better performance.

5. **Simulation Time**: Set `use_sim_time:=false` when working with real sensor data.

---

PCL filters applied in sequence, illustrating how they collectively process a LiDAR dataset
<img width="1725" height="808" alt="filters" src="https://github.com/user-attachments/assets/7b14746a-5d13-49d8-8e6f-40da0746b30f" />


