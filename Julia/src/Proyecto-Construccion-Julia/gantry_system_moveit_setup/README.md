# Custom config file 
- There are 2 file needed to change in config folder
    - gantry_system.srdf: define parameters of gantry system for moveit interface
    - moveit_controllers.yaml: select which controllers in ros2_controllers.yaml should be activated for moveit interface 
- Other files are default, don't need to change

# Launch file
- We will official_moveit_launch.launch.py for main launch file of moveit. This launch file will do following tasks:
    1. Launch the system control driver with some parameters: 
        ros2 launch gantry_description system_control_driver.launch.py
        - robot_ip: ip of robot (need to check)
        - use_fake_hardware: set to false if control real robot, true if use simulation
    2. Moveit node (mandatory to activate moveit)
    3. Rviz 
    4. Servo for realtime control of UR and gantry
    5. Moveit services developed by Luxolis

# Change speed of robot

ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
  "def my_prog():

    textmsg(\"Set the scaling speed\")

    socket_open(\"192.168.0.10\",50002)

    socket_send_string("Hello from robot")

    socket_close()

  end"}'