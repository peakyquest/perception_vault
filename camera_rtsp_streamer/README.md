# Camera RTSP Streamer

## RTSP Camera Streamer Node

The CameraNode is a ROS 2 node that connects to an RTSP (Real-Time Streaming Protocol) camera feed and publishes the video stream as sensor_msgs/Image messages.
This node enables integration of network cameras (such as IP cameras, security cameras, or Hikvision cameras) into ROS 2 systems for computer vision, perception, and monitoring applications.
The node supports configurable connection parameters, frame rate control, and optional image rotation for cameras mounted in different orientations.

### Inputs / Outputs

| Type | Topic Name | Description |
|------|------------|-------------|
| Input | RTSP Stream (External) | RTSP video stream from network camera (accessed via URL) |
| Output | `/rtsp_stream` | Published video frames as sensor_msgs/Image messages |

### Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `hostname` | `192.168.1.1` | IP address or hostname of the RTSP camera |
| `username` | `admin` | Username for RTSP camera authentication |
| `password` | `admin` | Password for RTSP camera authentication |
| `port` | `554` | RTSP port number (default is 554) |
| `stream` | `Streaming/Channels/102` | RTSP stream path (varies by camera model) |
| `inverted` | `false` | When true, rotates the image 180 degrees (useful for upside-down mounted cameras) |
| `publish_rate` | `30` | Publishing rate in Hz (frames per second) |

### How It Works

1. The node initializes with the configured RTSP connection parameters (hostname, username, password, port, and stream path).
2. It constructs the RTSP URL in the format: `rtsp://username:password@hostname:port/stream`
3. OpenCV's VideoCapture is used to establish a connection to the RTSP stream.
4. The node creates a timer that triggers at the specified `publish_rate` to read frames from the stream.
5. Each frame is captured using `cv2.VideoCapture.read()`, which retrieves the latest frame from the buffer.
6. If the `inverted` parameter is set to `true`, the frame is rotated 180 degrees using OpenCV's rotation function.
7. The OpenCV frame (BGR format) is converted to a ROS sensor_msgs/Image message using CvBridge.
8. The image message is stamped with the current ROS time and assigned the frame_id `"camera_link"`.
9. The converted image message is published on the `/rtsp_stream` topic.
10. If the connection is lost or a frame cannot be read, the node automatically attempts to reconnect to the camera.

### Launching the Node

The node can be launched using the provided launch file with customizable parameters:

```bash
ros2 launch camera_rtsp_streamer start_camera_stream.launch.py
```

You can override default parameters via launch arguments:

```bash
ros2 launch camera_rtsp_streamer start_camera_stream.launch.py \
    hostname:=192.168.1.63 \
    username:=admin \
    password:=your_password \
    port:=554 \
    stream:=Streaming/Channels/102 \
    inverted:=false \
    publish_rate:=30
```

### Visualization in RViz

1. Add an **Image** display in RViz.
2. Subscribe to the `/rtsp_stream` topic to visualize the camera feed in real-time.
3. The image will update at the rate specified by the `publish_rate` parameter.

### Notes

- **RTSP Stream Path**: The `stream` parameter varies by camera manufacturer and model. Common formats include:
  - Hikvision: `Streaming/Channels/101` (main stream) or `Streaming/Channels/102` (sub stream)
  - Generic RTSP: `/stream1`, `/live`, or `/h264`
  - Consult your camera's documentation for the correct stream path.
  
- **Network Requirements**: Ensure the ROS 2 system has network connectivity to the camera's IP address and that the RTSP port (default 554) is not blocked by firewalls.

- **Frame Rate**: The `publish_rate` parameter controls how often frames are published, but the actual frame rate may be limited by:
  - The camera's encoding and streaming capabilities
  - Network bandwidth and latency
  - Processing power of the system

- **Image Rotation**: The `inverted` parameter rotates the image 180 degrees, which is useful for cameras mounted upside-down. For other rotation angles or mirroring, the code can be extended.

- **Connection Resilience**: The node automatically attempts to reconnect if the camera connection is lost. If frames cannot be read, it logs a warning and continues trying.

- **Security**: The password is masked in log messages for security. However, avoid hardcoding credentials in launch files or source code. Consider using environment variables or ROS 2 parameter files for sensitive information.

- **Dependencies**: This package requires:
  - OpenCV (python3-opencv) for RTSP stream capture
  - cv_bridge for converting between OpenCV and ROS image formats
  - sensor_msgs for image message types

