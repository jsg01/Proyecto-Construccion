# Gantry Description Model
Include URDF model for gantry system and example launch file for simulation 

## Structure
- config: config files for ros params (params for environment, controllers and robot configuration)
- launch: provide launch files
- meshes: mesh file to visualize components 
- rviz: configuration for rviz GUI
- scripts: main code for controllers
- urdf: URDF models of system 

## Requirement 
0. Setup_robot_driver
    ```
    sudo apt install ros-humble-ur
    ```
2. Extract calibration information from real robot 
    ```
    ros2 launch ur_calibration calibration_correction.launch.py \
    robot_ip:=<robot_ip> target_filename:="${HOME}/robot_calibration.yaml"
    ```
3. Change the default kinematic of ur robot 
    ```
    cd workspace/src/gantry_description/config
    ```
    copy the content of robot_calibration.yaml to default_kinematics.yaml

## Build 
1. clone repo to your workspace/src
2. Build package
    ```
    cd WORKSPACE 
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    colcon build --symlink-install
    source install/local_setup.bash
    ```

## See robot model on Rviz 

1. You can visualize and check the URDF model of system by command:
```
ros2 launch gantry_description view_model.launch.py
```
2. Adjust the joint angles in bar to make movements 

## Run simulation in Gazebo 
run example of gazebo simulation without gazebo GUI
    ```
    ros2 launch gantry_description simulation_ros2_python.launch.py gazebo_gui:=false
    ```

## Run simulation with mock hardware of ros2_control 
```
ros2 launch gantry_description simulation_mock_hardware.launch.py 
```

## Run demo test (real world)
# Turn on robot
1. Turn on the "Remote" mode on Teaching padent

2. Run this command on computer to activate communication in headless mode
```
ros2 launch gantry_description demo_real.launch.py ur_type:=ur10e robot_ip:=192.168.0.20 launch_rviz:=true headless_mode:=true
```

3. Run the following command to start system and turn on robot

ros2 service call /dashboard_client/power_on std_srvs/srv/Trigger {}

ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger {}

4. 

## Notice
- In urdf folder there are 3 model: 
    - system_ros2.urdf.xacro: combine of all sytem  
    - end_effector.urdf.xacro: end-effector model
    - gantry_frame.urdf.xacro: frame of gantry

- Change the gantry_environment.yaml file to modify setup of environment
