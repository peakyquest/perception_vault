# Image Lane Detection

## Introduction

The `image_lane_detection` package is a ROS 2 node that performs real-time lane line detection on road images using computer vision techniques. The package implements a classic lane detection pipeline based on edge detection, Hough transform, and line fitting algorithms. It can process images from either a video dataset stored in the package resources folder or from a live camera feed via ROS 2 topics.

The lane detection algorithm follows standard computer vision approaches:
- **Canny Edge Detection** for identifying edges in the image
- **Region of Interest (ROI)** masking to focus on the road area
- **Hough Line Transform** for detecting line segments
- **Slope-based filtering** to separate left and right lane lines
- **Line averaging** to produce smooth, continuous lane markings

The detected lanes are visualized as green lines with a filled polygon overlay, making it easy to see the detected lane boundaries in real-time.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_dataset` | bool | `true` | If `true`, reads frames from a video file in the resources folder. If `false`, subscribes to an image topic for live processing. |
| `dataset_file` | string | `dataset_3.mp4` | Name of the video file located in the `resources/` folder. The file must exist or the node will raise an exception. |
| `image_topic` | string | `camera/image_raw` | ROS 2 topic name to subscribe to when `use_dataset` is `false`. |
| `debug` | bool | `true` | If `true`, publishes additional debug image topics for visualization of intermediate processing steps (Canny edges, ROI mask, and detected lines). |
| `use_sim_time` | bool | `false` | Use simulation time (e.g., from Gazebo) if `true`. |

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image_raw` (or `image_topic` parameter) | `sensor_msgs/Image` | Input image topic when `use_dataset` is `false`. The node subscribes to this topic to receive live camera frames for lane detection. |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `detected_lane` | `sensor_msgs/Image` | Output image with detected lane lines overlaid. Green lines represent the detected lane boundaries, and a blue filled polygon shows the lane area. |
| `debug/canny_image` | `sensor_msgs/Image` | (Debug mode only) Canny edge detection result showing all detected edges in the image. |
| `debug/roi_image` | `sensor_msgs/Image` | (Debug mode only) Region of interest mask applied to the Canny edges, showing only edges within the trapezoidal ROI. |
| `debug/line_image` | `sensor_msgs/Image` | (Debug mode only) Detected lane lines drawn on a blank image, showing only the lane visualization without the original frame. |

## How It Works

The lane detection pipeline consists of several sequential processing steps:

### 1. **Image Preprocessing**
   - The input image (BGR format) is converted to grayscale
   - A Gaussian blur (5x5 kernel) is applied to reduce noise
   - Canny edge detection is performed with thresholds (50, 150) to identify edges

### 2. **Region of Interest (ROI) Masking**
   - A trapezoidal mask is applied to focus on the road area where lane lines are expected
   - The ROI is defined as a trapezoid:
     - Bottom left: 10% of image width
     - Bottom right: 90% of image width
     - Top left: 45% of image width at 60% of image height
     - Top right: 55% of image width at 60% of image height
   - This trapezoidal shape is more suitable for lane detection than a simple triangle

### 3. **Line Detection (Hough Transform)**
   - The Probabilistic Hough Line Transform (`cv2.HoughLinesP`) is applied to detect line segments
   - Parameters:
     - `rho = 2`: Distance resolution in pixels
     - `theta = π/180`: Angular resolution (1 degree)
     - `threshold = 100`: Minimum votes for a line to be detected
     - `minLineLength = 40`: Minimum line length in pixels
     - `maxLineGap = 50`: Maximum gap between line segments to connect them

### 4. **Line Filtering and Averaging**
   - Detected lines are filtered based on their slopes:
     - Lines with slope magnitude < 0.5 are discarded (too horizontal, likely not lane lines)
     - Lines with negative slope are classified as left lane lines
     - Lines with positive slope are classified as right lane lines
   - For each lane (left/right), the slopes and intercepts are averaged to produce a single representative line
   - Both left and right lanes must be detected; if either is missing, no lanes are drawn

### 5. **Visualization**
   - The averaged lane lines are drawn in green on the original image
   - A filled polygon (blue) is drawn between the left and right lane lines to highlight the detected lane area
   - The final image is created by blending the original frame (80% opacity) with the lane visualization (100% opacity)

### 6. **Output**
   - The processed image is published to the `detected_lane` topic
   - If debug mode is enabled, intermediate processing results are published to separate debug topics

## Usage Examples

### Using Dataset Video (Default)

```bash
# Launch with default settings (uses dataset_3.mp4)
ros2 launch image_lane_detection image_lane_detection_launch.py

# Launch with a specific dataset file
ros2 launch image_lane_detection image_lane_detection_launch.py dataset_file:=dataset_1.mp4

# Launch with debug mode disabled
ros2 launch image_lane_detection image_lane_detection_launch.py debug:=false
```

### Using Live Camera Feed

```bash
# Subscribe to a camera topic
ros2 launch image_lane_detection image_lane_detection_launch.py \
  use_dataset:=false \
  image_topic:=/my_camera/image_raw

# With simulation time enabled
ros2 launch image_lane_detection image_lane_detection_launch.py \
  use_dataset:=false \
  use_sim_time:=true
```

### Viewing the Results

```bash
# View the detected lanes
ros2 run rqt_image_view rqt_image_view /detected_lane

# View debug images (if debug mode is enabled)
ros2 run rqt_image_view rqt_image_view /debug/canny_image
ros2 run rqt_image_view rqt_image_view /debug/roi_image
ros2 run rqt_image_view rqt_image_view /debug/line_image
```

## Requirements

- ROS 2 (tested with Jazzy)
- OpenCV (cv2)
- NumPy
- cv_bridge
- sensor_msgs

## File Structure

```
image_lane_detection/
├── image_lane_detection/
│   └── image_lane_detection_node.py    # Main node implementation
├── launch/
│   └── image_lane_detection_launch.py  # Launch file
├── resources/
│   ├── dataset_1.mp4                    # Sample video datasets
│   ├── dataset_2.mp4
│   └── dataset_3.mp4
├── CMakeLists.txt
├── package.xml
└── README.md
```


## Rviz Visualization

<img width="1203" height="781" alt="Screenshot from 2025-12-23 23-20-34" src="https://github.com/user-attachments/assets/5fae3175-0c04-4ac4-be8e-76f276d65c0d" />


## Notes

- Video files must be placed in the `resources/` folder of the package
- The node will raise a `FileNotFoundError` if the specified dataset file does not exist
- When using dataset mode, the video will loop automatically when it reaches the end
- The algorithm works best with clear lane markings and good lighting conditions
- Adjust Hough transform parameters in the code if detection quality needs improvement for specific scenarios

