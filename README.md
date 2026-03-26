# Proyecto-Construccion

**Proyecto de Construcción – Laboratorio de Robótica y Automática**

This repository contains experiments and tools related to camera acquisition and ROS2 data collection for the robotics laboratory.

---

## Requirements

- Ubuntu 22.04
- ROS2 Humble

Make sure ROS2 is installed and sourced before running any commands.

```bash
source /opt/ros/humble/setup.bash
```

---

## USB Camera Setup (ROS2 Humble)

Update the system and install the USB camera driver:

```bash
sudo apt update
sudo apt install ros-humble-usb-cam
```

---

## Running the Camera Node

Launch the USB camera node:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```

If your camera device is different, check available devices:

```bash
ls /dev/video*
```

---

## Viewing the Camera Stream

You can visualize the camera feed using **rqt_image_view**:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select the topic:

```
/image_raw
```

Alternatively, you can visualize the stream in **RViz2**:

```bash
rviz2
```

Add an **Image** or **Camera** display and select the corresponding topic.

---

## ROSBAG Files

This repository contains recorded ROS2 datasets used for testing and debugging.
The Rosbag files can be found: https://drive.google.com/drive/folders/1X39_cXXQb7YtfracFyYDbJilWbTsT5vm?usp=drive_link
```
.
├── Elenas_video
│   ├── metadata.yaml
│   └── rosbag2_2026_03_12-17_50_01_0.mcap
│
└── Ricardo_image
    └── rosbag2_all_topics
        ├── metadata.yaml
        └── rosbag2_2026_03_13-04_38_21_0.db3
```

---

## Playing a Rosbag

To replay a rosbag:

```bash
ros2 bag play <rosbag_folder>
```

Example:

```bash
ros2 bag play Elenas_video
```

---

## Recording a Rosbag

Record all available topics:

```bash
ros2 bag record -a
```

Record only specific topics:

```bash
ros2 bag record /image_raw
```

---

---
## USING UR3e
1.  ros2 launch moveit_setup_robot_lab driver_ur34.launch.py
1.  (a) (For simulation) ros2 launch moveit_setup_robot_lab driver_ur34.launch.py "use_fake_hardware:=true"
2. ros2 control load_controller scaled_joint_trajectory_controller
3. ros2 control set_controller_state scaled_joint_trajectory_controller active
4. ros2 launch moveit_setup_robot_lab move_group.launch.py
5. ros2 launch moveit_setup_robot_lab moveit_rviz.launch.py

## For taking a picture
1. ros2 run calibration_package calibration_node
if successful 
2. ros2 service call /calibration/picture std_srvs/srv/Trigger "{}"

(Image will be save at install/calibration_package/share/calibration_package/config, change the name of the picture, or you will lose, also if you want to keep it, move it to src/calibration_package/config)

## For getting weight colors
1. ros2 run calibration_package calibration_node (only if it is not already running)
Consider this a current bug:
2. Assuming there is an image with several blocks at install/calibration_package/share/calibration_package/config/picture_taken.jpg, run:

ros2 service call /calibration/SelectCustomColor custom_interfaces/srv/CustomColorDetector "{color: 'color-chosen'}"