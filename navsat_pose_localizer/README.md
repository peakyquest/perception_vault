# navsat_pose_localizer

## NavSat Pose  Node

The `NavsatPoseLocalizer` node is a ROS 2 Python node that converts GNSS data (`sensor_msgs/msg/NavSatFix`) into a localized odometry message (`nav_msgs/msg/Odometry`). This node enables robots to use GPS latitude/longitude as a position source within a local reference frame. The node calculates odometry relative to a reference origin provided via a `geographic_msgs/msg/GeoPoint` message, and estimates orientation (yaw) based on movement direction.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|------------|-------------|
| Input | `gps/fix`  | GNSS Fix data as `sensor_msgs/msg/NavSatFix` |
| Input | `reference_origin` | Reference origin as `geographic_msgs/msg/GeoPoint` |
| Output | `navsat_odometry`| Localized position & orientation as `nav_msgs/msg/Odometry` |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `gps_fix_topic` | `gps/fix` | Input GPS Fix topic (`sensor_msgs/msg/NavSatFix`) |
| `reference_origin_topic` | `reference_origin` | Reference origin topic (`geographic_msgs/msg/GeoPoint`) |
| `odom_topic` | `navsat_odometry` | Output odometry topic (`nav_msgs/msg/Odometry`) |
| `odom_frame` | `odom` | Frame ID for the odometry message header.frame_id |
| `base_frame` | `base_link` | Child frame ID used in odometry child_frame_id |
| `yaw_min_distance` | `0.2` | Minimum travelled distance (metres) before yaw is updated |

---

### How It Works

- The node subscribes to both the GPS Fix topic and the reference origin topic.
- **Reference Origin**: The node waits for a `geographic_msgs/msg/GeoPoint` message on the reference origin topic. Once received, this point is converted to UTM coordinates and used as the origin (0, 0) for all subsequent odometry calculations. The reference origin is set only once; subsequent messages are ignored.
- **GPS Processing**: The node subscribes to GPS Fix messages (`sensor_msgs/msg/NavSatFix`). Each valid GPS Fix is converted to UTM coordinates using the `utm` library.
- **Position Calculation**: The position in the odometry message is calculated as the displacement from the reference origin in UTM coordinates (x, y). The z-coordinate is set to 0.0 (2D odometry).
- **Orientation (Yaw) Estimation**: The yaw angle is estimated from the direction of movement. The node tracks the last UTM position and calculates yaw using `atan2(delta_y, delta_x)` when the vehicle has moved at least `yaw_min_distance` metres. This prevents noisy yaw updates when the vehicle is stationary or moving slowly.
- **Odometry Publishing**: The odometry message is published with the calculated position and orientation. Invalid GPS Fix data (status < 0) results in skipped publications.

---



---

### Visualization in RViz

- Add an **Odometry** display in RViz.
- Set the topic to your configured odometry topic (default: `odom`).
- Ensure the `odom_frame` matches your global frame (e.g., `odom` or `Car1`).
- You will see the odometry marker update whenever a new GPS Fix is processed.

---

### Notes

- **Reference Origin Required**:  
  The node requires a reference origin to be published before it can calculate odometry. GPS fixes received before the reference origin is set are ignored.

- **Yaw Estimation**:  
  Orientation (yaw) is estimated from movement direction, not from GPS heading. The `yaw_min_distance` parameter controls how sensitive the yaw updates are to movement. Smaller values make yaw more responsive but noisier; larger values make it smoother but less responsive.

- **Altitude**:  
  Flat 2D output (z=0). Altitude from the reference origin is not used in the odometry calculation.

- **UTM Projection**:  
  The node uses UTM (Universal Transverse Mercator) projection to convert latitude/longitude to local coordinates. This provides accurate local positioning for navigation.

- **Dependencies**:  
  - `rclpy`  
  - `sensor_msgs`  
  - `nav_msgs`  
  - `geometry_msgs`  
  - `geographic_msgs`  
  - `utm` (Python library)

---

### TODO

- Add odometry covariance computation based on GPS Fix covariance.
- Add support for velocity estimation from GPS data.  

