# LiDAR Rule-Based Classifier

A ROS2 C++ node for classifying LiDAR point cloud clusters into object types (person and car) using rule-based algorithms. This package extracts comprehensive geometric and statistical features from clusters and applies distance-adaptive classification rules.

## Overview

The `lidar_rule_based_classifier` node processes clustered point clouds from a Euclidean clustering algorithm and classifies each cluster as either a **person** or **car** based on geometric features and rule-based heuristics. It publishes visualization markers with wireframe bounding boxes and text labels.

### What the Code Does

1. **Subscribes to clustered point clouds** (`/cluster_points`) - PointCloud2 messages where cluster IDs are stored in the intensity field
2. **Subscribes to cluster markers** (`/cluster_markers`) - MarkerArray for visualization synchronization
3. **Extracts 22 features** from each cluster using:
   - Geometric properties (height, volume, bounding box dimensions)
   - Statistical measures (mean height, point density)
   - PCA-based shape features (planarity, linearity, sphericity)
4. **Applies rule-based classification** using critical and important rules
5. **Publishes classified markers** (`/cluster_classifier_marker`) with:
   - Wireframe bounding boxes (LINE_LIST markers)
   - Text labels showing classification (person/car)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cluster_points_topic` | string | `/cluster_points` | Input topic for clustered point clouds (cluster ID in intensity field) |
| `cluster_markers_topic` | string | `/cluster_markers` | Input topic for cluster markers (for synchronization) |
| `cluster_classifier_marker_topic` | string | `/cluster_classifier_marker` | Output topic for classified visualization markers |
| `max_detection_distance` | double | `50.0` | Maximum distance (meters) from sensor for detection. Clusters beyond this are ignored |
| `strict_distance_threshold` | double | `30.0` | Distance (meters) beyond which rules become 10% stricter to reduce false positives |
| `detect_classes` | string[] | `["person", "car"]` | List of object classes to detect. Valid options: `"person"`, `"car"` |
| `verbose` | bool | `false` | Enable verbose logging with detailed classification information |

### Example Launch File Configuration

```python
parameters=[{
    'cluster_points_topic': '/cluster_points',
    'cluster_markers_topic': '/cluster_markers',
    'cluster_classifier_marker_topic': '/cluster_classifier_marker',
    'max_detection_distance': 50.0,
    'strict_distance_threshold': 30.0,
    'detect_classes': ['person', 'car'],
    'verbose': False,
}]
```

## Classification Rules

### Person Detection Rules

The classifier uses **11 rules** divided into **5 Critical Rules** (must pass all) and **6 Important Rules** (at least 4 must pass).

#### Critical Rules (ALL must pass):
1. **Height**: 1.3m ≤ height ≤ 2.2m
2. **Height is largest dimension**: Height must be the largest bounding box dimension (upright objects)
3. **Vertical orientation**: Height ratio ≥ 1.3x (height relative to average width/length)
4. **Maximum width**: max(width, length) ≤ 0.85m (stricter beyond 30m: ≤ 0.765m)
5. **Footprint area**: width × length ≤ 0.6m² (stricter beyond 30m: ≤ 0.54m²)

#### Important Rules (at least 4 must pass):
1. **Centroid height**: 0.5m ≤ centroid Z ≤ 2.0m (people are above ground)
2. **Volume**: 0.1m³ ≤ volume ≤ 5.0m³
3. **Point count**: 30 ≤ num_points ≤ 2000
4. **Point density**: 100 ≤ density ≤ 50,000 points/m³
5. **Shape check**: planarity < 0.7 AND sphericity > 0.2 (not flat, somewhat rounded)
6. **Width/Length ratio**: max(width, length) / min(width, length) ≤ 2.2 (roughly circular from above)

### Car Detection Rules

The classifier uses **17 rules** divided into **8 Critical Rules** (must pass all) and **10 Important Rules** (at least 6-7 must pass, depending on distance).

#### Critical Rules (ALL must pass):
1. **Height**: 1.0m ≤ height ≤ 2.2m
2. **Length is largest dimension**: Length must be the largest bounding box dimension
3. **Minimum length**: length ≥ 2.8m
4. **Elongated shape**: length/width ratio ≥ 1.6
5. **Footprint area**: 
   - Near (≤30m): 3.5m² ≤ footprint ≤ 16.0m²
   - Far (>30m): 4.5m² ≤ footprint ≤ 14.0m²
6. **Width check**: 1.5m ≤ width ≤ 2.4m
7. **Ground contact**: Bottom Z ≤ 0.3m (cars are on the ground)
8. **Not a truck**: length < 7.0m AND NOT (width ≥ 1.8m AND footprint ≥ 12.0m²)

#### Important Rules (6-7 must pass, depending on distance):
1. **Centroid height**: 0.2m ≤ centroid Z ≤ 1.2m
2. **Volume**: 2.0m³ ≤ volume ≤ 55.0m³
3. **Maximum length**: length < 7.0m (to avoid truck overlap)
4. **Point count**: 250 ≤ num_points ≤ 18,000
5. **Point density**: 40 ≤ density ≤ 25,000 points/m³
6. **Low profile**: height/length ratio ≤ 0.65 (cars are low and long)
7. **Shape check**: planarity > 0.2 (somewhat flat top)
8. **Not too tall**: height/width ratio ≤ 1.5 (reject vertical objects)
9. **Volume proportional**: 0.7 ≤ volume/footprint ≤ 2.5 (realistic height)
10. **Maximum height**: height ≤ 2.2m

**Note**: Far clusters (>30m) require 7/10 important rules; near clusters require 6/10.

## Feature Extraction

For each cluster, the node extracts **22 features**:

### Geometric Features (8):
- `height`: Z-extent (max_z - min_z)
- `centroid`: 3D centroid (x, y, z)
- `bounding_box`: Axis-aligned bounding box (length, width, height)
- `volume`: Bounding box volume (length × width × height)
- `aspect_ratio`: length / width
- `distance_from_origin`: Distance from sensor origin (0,0,0) to cluster centroid

### Statistical Features (4):
- `mean_height`: Average Z coordinate
- `std_height`: Standard deviation of Z coordinates
- `point_density`: Points per cubic meter (num_points / volume)
- `num_points`: Total number of points in cluster

### Shape Features (10 - PCA-based):
- `eigenvalues`: Three eigenvalues from PCA (λ1 ≥ λ2 ≥ λ3)
- `eigenvalue_ratios`: [λ2/λ1, λ3/λ2]
- `linearity`: (λ1 - λ2) / λ1 (how elongated)
- `planarity`: (λ2 - λ3) / λ1 (how flat)
- `sphericity`: λ3 / λ1 (how spherical)

**Note**: PCA is computed using Eigen library. If eigenvalue computation fails, default values are used (linearity=0.0, planarity=0.0, sphericity=1.0).

## Data Flow

```
Cluster Points (PointCloud2) 
    ↓
Extract cluster_id from intensity field
    ↓
Group points by cluster_id → cluster_dict
    ↓
Wait for cluster_markers (synchronization)
    ↓
For each cluster:
    ├─ Extract 22 features
    ├─ Apply Person Classification Rules
    ├─ If not person: Apply Car Classification Rules
    └─ If classified: Create wireframe bbox + text markers
    ↓
Publish MarkerArray to /cluster_classifier_marker
```

## Visualization

The node publishes visualization markers with:

- **Wireframe Bounding Boxes** (LINE_LIST):
  - **Red** for person detections
  - **Green** for car detections
  - 12 edges forming a 3D wireframe box
  - Line width: 0.05m

- **Text Labels** (TEXT_VIEW_FACING):
  - Classification name in uppercase (PERSON, CAR)
  - Positioned above the bounding box
  - Color matches bounding box color
  - Text height: 0.3m

Markers are automatically deleted when clusters disappear to prevent stale visualizations.

## Building

```bash
cd /path/to/ros2_ws
colcon build --packages-select lidar_rule_based_classifier
source install/setup.bash
```

## Running

### Using Launch File
```bash
ros2 launch lidar_rule_based_classifier lidar_rule_based_classifier_launch.py
```

### Running Directly
```bash
ros2 run lidar_rule_based_classifier lidar_rule_based_classifier_node
```

### Running with Custom Parameters
```bash
ros2 run lidar_rule_based_classifier lidar_rule_based_classifier_node \
  --ros-args \
  -p cluster_points_topic:=/custom_cluster_points \
  -p detect_classes:=['person'] \
  -p verbose:=true
```

## Dependencies

- **ROS2** (tested with Jazzy)
- **rclcpp**: ROS2 C++ client library
- **sensor_msgs**: PointCloud2 message support
- **visualization_msgs**: Marker and MarkerArray messages
- **geometry_msgs**: Point messages
- **PCL (Point Cloud Library)**: Point cloud processing
- **Eigen3**: Linear algebra and PCA computation
- **pcl_conversions**: ROS2-PCL message conversion

## Topic Overview

### Subscribed Topics
- `/cluster_points` (sensor_msgs/PointCloud2): Input clustered point cloud with cluster IDs in intensity field
- `/cluster_markers` (visualization_msgs/MarkerArray): Cluster markers for synchronization

### Published Topics
- `/cluster_classifier_marker` (visualization_msgs/MarkerArray): Classification markers with wireframe bounding boxes and text labels

## Performance

- **Implementation**: C++17 for real-time performance
- **Feature extraction**: ~22 features computed per cluster
- **Classification**: Rule-based (no ML dependencies)
- **Distance-adaptive**: Rules become stricter beyond 30m to reduce false positives

## Algorithm Summary

1. **Feature Extraction**: Compute geometric, statistical, and shape features from cluster point cloud
2. **Distance Check**: Filter clusters beyond max_detection_distance
3. **Person Classification**: Apply 11 rules (5 critical + 6 important, need 4+ important)
4. **Car Classification**: Apply 17 rules (8 critical + 10 important, need 6-7 important) if not a person
5. **Visualization**: Create wireframe bounding boxes and text labels for classified clusters

## Notes

- The node expects clustered point clouds where cluster IDs are stored in the `intensity` field of PointCloud2 messages
- Classification is deterministic (rule-based, no randomness)
- Rules are distance-adaptive: stricter beyond `strict_distance_threshold` (30m default)
- Markers are automatically cleaned up when clusters disappear
- The classifier distinguishes cars from trucks using length threshold (cars < 7.0m)

## Author

Haroon Rasheed (haroon300@hotmail.com)

