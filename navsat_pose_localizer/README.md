# navsat_pose_localizer

## NavSat Pose Localizer Node

The `NavSatPoseLocalizer` node is a ROS 2 node that converts GNSS data (`sensor_msgs/msg/NavSatFix`) into a localized odometry message (`nav_msgs/msg/Odometry`).  
This node enables robots to use GPS latitude/longitude/altitude as a position source within a local reference frame. It is intended for navigation systems that require odometry-style positioning sourced from GNSS measurements. The node supports configuration of input/output topics, reference frame selection, altitude usage, and future extensions for attitude-based orientation estimation.

---

### Inputs / Outputs

| Type | Topic Name | Description |
|------|------------|-------------|
| Input | `/gps/fix` | GNSS Fix data as `sensor_msgs/msg/NavSatFix` |
| Output | `/gps/odom` | Localized position & orientation as `nav_msgs/msg/Odometry` |

---

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `gps_topic` | `gps/fix` | Input GPS Fix topic |
| `odom_topic` | `gps/odom` | Output odometry topic |
| `frame_id` | `odom` |  Frame for the odometry message |
| `child_frame_id` | `base_link` | Child frame used in odometry |
| `reference_latitude` | `0.0` | Latitude of the ENU/UTM origin |
| `reference_longitude` | `0.0` | Longitude of the ENU/UTM origin |
| `reference_altitude` | `0.0` | Altitude of the ENU/UTM origin |

---

### How It Works

- The node loads configuration parameters such as the input GPS topic, odometry topic, local frame, reference coordinates, and projection mode.
- The node subscribes to the GPS Fix topic (default: `/gps/fix`) and waits for incoming `sensor_msgs/msg/NavSatFix` data.
- Once the first GPS Fix message is received, the node initializes or loads the origin used for local coordinate conversion.
- Each incoming GPS Fix is converted into (x, y, z) coordinates using UTM projection relative to the reference origin.
- Velocities are left as zero unless integrated or fused externally.
- The odometry message is timestamped and published on the configured odometry topic.
- Invalid GPS Fix data (no fix, poor quality, NaN fields) results in warnings and skipped publications.

---

### Visualization in RViz

- Add an **Odometry** display in RViz.
- Set the topic to `/gps/odom` (or your configured topic).
- Ensure the `frame_id` matches your global frame (e.g., `map`).
- You will see the odometry marker update whenever a new GPS Fix is processed.

---

### Notes

- **Reference Origin**:  
  If no reference coordinates are provided, the first valid GPS Fix is used as the local origin.

- **Position Only**:  
  Orientation is currently identity. Attitude support is planned.

- **Altitude**:  
  Flat 2D output (z=0).


- **Dependencies**:  
  - `rclcpp`  
  - `sensor_msgs`  
  - `nav_msgs`  
  - `geometry_msgs`  

---

### TODO

- Add **use_attitude** parameter to include GNSS attitude in odometry orientation.  
- Add odometry covariance computation based on GPS Fix covariance.  

