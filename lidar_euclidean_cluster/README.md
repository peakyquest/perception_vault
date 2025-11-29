# LiDAR Euclidean Cluster

This package provides a ROS 2 node for performing **Euclidean clustering** on LiDAR point clouds using the Point Cloud Library (PCL). The node segments filtered point clouds into distinct clusters based on Euclidean distance, which is essential for object detection and tracking in autonomous vehicles and robotics applications. The node groups nearby points into clusters and assigns unique cluster IDs to each point via the intensity field. It also provides optional visualization markers showing cluster centers and bounding boxes.

---

## Euclidean Cluster Node

The **LidarEuclideanClusterNode** is a ROS 2 node that subscribes to a filtered `sensor_msgs/PointCloud2` topic (typically after voxel filtering and ground removal) and performs Euclidean distance-based clustering to identify and segment objects in the point cloud.

This clustering process groups nearby points together, allowing downstream perception systems to treat each cluster as a potential object (e.g., vehicles, pedestrians, obstacles) for further analysis.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|-------------|-------------|
| **Input** | `/filtered_points` | Incoming filtered point cloud (typically after voxel grid filtering and ground removal) |
| **Output** | `/cluster_points` | Point cloud with cluster IDs stored in the intensity field (each point's intensity = its cluster ID) |
| **Output (Optional)** | `/cluster_markers` | Visualization markers for cluster centers and bounding boxes (published only if `publish_markers=true`) |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `input_topic` | `/filtered_points` | Input topic name for the filtered point cloud |
| `output_topic` | `/cluster_points` | Output topic name for the clustered point cloud |
| `marker_topic` | `/cluster_markers` | Topic name for visualization markers (only used if `publish_markers=true`) |
| `cluster_tolerance` | `0.5` | Distance threshold in meters for grouping points into clusters. Smaller values create more clusters (over-segmentation), larger values merge separate objects (under-segmentation) |
| `min_cluster_size` | `30` | Minimum number of points required to form a valid cluster. Clusters with fewer points are filtered out as noise |
| `max_cluster_size` | `5000` | Maximum number of points allowed in a single cluster. Prevents merging of multiple large objects |
| `publish_markers` | `true` | Enable/disable publishing of visualization markers for cluster centers and bounding boxes |
| `verbose` | `false` | Enable detailed logging for debugging and monitoring |

---

### How It Works

- **Input Reception**: The node subscribes to a filtered point cloud (typically from voxel grid filter and ground segmentation).

- **KD-Tree Construction**: A KD-tree data structure is built from the input point cloud to enable efficient nearest-neighbor searches during clustering.

- **Euclidean Clustering**:
   - The PCL Euclidean Cluster Extraction algorithm groups points that are within `cluster_tolerance` distance of each other.
   - Clusters must contain at least `min_cluster_size` points (to filter noise).
   - Clusters cannot exceed `max_cluster_size` points (to prevent merging separate objects).

- **Cluster ID Assignment**:
   - Each cluster is assigned a unique integer ID starting from 1.
   - All points in a cluster receive the same ID stored in their intensity field.
   - Points with `intensity = 0` indicate no cluster assignment.

- **Output Publication**:
   - The clustered point cloud is published with cluster IDs in the intensity field.
   - Optional visualization markers are published showing cluster centroids (red spheres) and 3D bounding boxes (semi-transparent green boxes).

- **Marker Management**:
   - The node automatically deletes markers for clusters that no longer exist in subsequent frames, preventing "ghost markers" in RViz.

---

### Visualization in RViz

To visualize the clustering results:

- **Point Cloud Visualization**:
   - Add a **PointCloud2** display and subscribe to `/cluster_points`.
   - Set the **Color Transformer** to "Intensity" to see different clusters in different colors.
   - Each unique intensity value (cluster ID) will appear as a different color.

- **Cluster Markers** (if `publish_markers=true`):
   - Add a **MarkerArray** display and subscribe to `/cluster_markers`.
   - You will see:
     - **Red spheres**: Cluster centroids (centers of mass)
     - **Green semi-transparent boxes**: 3D bounding boxes around each cluster

- **Recommended RViz Settings**:
   - Point Size: 2-4 pixels for better visibility
   - Marker Alpha: Adjust bounding box transparency as needed
   - Fixed Frame: Set to your LiDAR frame (e.g., `velodyne` or `lidar`)
     
- **Expected Visualization**
<p align="center">
  <img src="https://github.com/user-attachments/assets/5d75fe16-456c-457b-9d24-92eee5ef0ca5" width="600">
</p>


---

### Tuning Parameters

The clustering parameters significantly affect the results. Here are guidelines for tuning:

#### `cluster_tolerance`
- **Too small** (e.g., 0.2-0.3m): Over-segmentation - one object split into multiple clusters
- **Too large** (e.g., 1.5-2.0m): Under-segmentation - multiple objects merged together
- **Recommended**: 0.5-1.2m for typical automotive LiDAR after voxel filtering
- **Tip**: Start with 0.8-1.0m and adjust based on typical object spacing in your scene

#### `min_cluster_size`
- **Too small** (e.g., 5-10): Noise points form false clusters
- **Too large** (e.g., 50+): Small objects (pedestrians, small obstacles) are filtered out
- **Recommended**: 20-30 points for voxel-filtered clouds (leaf size 0.1-0.2m)
- **Tip**: Set based on expected minimum object size in your point cloud density

#### `max_cluster_size`
- **Too small** (e.g., 2000): Large vehicles (trucks, buses) are split into multiple clusters
- **Too large** (e.g., 15000+): Multiple separate objects may merge
- **Recommended**: 5000-8000 for standard vehicles, 8000-12000 for large vehicles
- **Tip**: Consider your scene - if large objects are common, increase this value

---


### Launch File

The package includes a launch file with all parameters configurable:
```bash
ros2 launch lidar_euclidean_cluster lidar_euclidean_cluster_launch.py
```


---



### Notes

- **Input Quality**: This node works best with pre-filtered point clouds. Ensure your input has been:
  - Downsampled via voxel grid filtering (reduces computational load)
  - Ground-removed (improves clustering accuracy)

- **Performance**: 
  - Clustering performance scales with point cloud size
  - Disable markers (`publish_markers=false`) if visualization is not needed for better performance
  - Typical processing time: 10-50ms for 10K-50K points on modern hardware

- **Coordinate Frame**: The output point cloud and markers use the same frame ID as the input cloud.

- **Empty Clouds**: The node handles empty input clouds gracefully by publishing empty output clouds and clearing existing markers.

- **Cluster IDs**: Cluster IDs are assigned sequentially starting from 1. ID values may change between frames as objects move and new clusters appear.

---

### Dependencies

- **ROS 2**: Humble/Jazzy
- **PCL**: Point Cloud Library (libpcl-all-dev)
- **pcl_conversions**: ROS 2 PCL conversion utilities

---

### License

Apache License 2.0

---

