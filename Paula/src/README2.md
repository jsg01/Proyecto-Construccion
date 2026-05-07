# Full System Startup Order

## 1. Start Calibration services
```bash
ros2 run calibration_services plane_transform_server
```
---
## 2. Start Loose Piece Detection
```bash
ros2 run piece_detection_pkg piece_detection_node
```
---
## 3. Start robot in simulation mode (unless you have a simpler way to send the pieza_sueltas message)
```bash
ros2 launch moveit_setup_robot_lab full_system_launch.launch.py "use_fake_hardware:=true"
```

## 4. There are two types of rosbags
Keep in mind you should only run one at a time. For instance, use cntrl + c to stop running the background and then play the pieces recording

### For running the background
```bash
 ros2 bag play background_elena_2 --loop
```
### For running the pieces
```bash
 ros2 bag play piece_elena_2 --loop
```

# Expected Startup Behavior

When launched correctly, the node will:

1. Wait for calibration services
2. Request the robot to move to:

```text
DetectaPiezasSueltas
```

3. Wait for robot confirmation
4. Ask the user to capture the empty background with:

```text
b
```

5. Allow snapshot processing with:

```text
p
```

---

# YAML Export

Detections are automatically saved to:

```bash
/tmp/detections.yaml
```

---

# Camera Calibration

Calibration is loaded from:
(you might want to change this path)

```bash
/home/ricardo/Desktop/LabRobot/Paula/src/config/camera_intrinsics.yaml 
```

---




