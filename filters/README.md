# Point Cloud Library  Filters

## Voxel Grid Filter Node

The **VoxelGridNode** is a ROS 2 node that subscribes to a raw `sensor_msgs/PointCloud2` topic and downsamples it using the **PCL VoxelGrid filter**. This process reduces the number of points in the cloud while maintaining the overall structure, improving computational efficiency for perception and mapping tasks.

---

### Inputs/Outputs

| Type | Topic Name | Description |
|------|-------------|--------------|
| **Input** | `/points/raw` | Incoming unfiltered point cloud data |
| **Output** | `/points/voxel_filter` | Downsampled point cloud after applying voxel grid filter |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|----------------|--------------|
| `leaf_size_x` | `0.1` | Size of each voxel leaf along the X-axis (in meters) |
| `leaf_size_y` | `0.1` | Size of each voxel leaf along the Y-axis (in meters) |
| `leaf_size_z` | `0.1` | Size of each voxel leaf along the Z-axis (in meters) |
| `debug` | `false` | When true, prints the number of raw and downsampled points for debugging |

---

### Example Debug Output
```
[INFO] [1762586008.093982142] [voxel_grid_node]: Downsampled cloud: 65536 → 39126 points (40.3% reduction)

```
