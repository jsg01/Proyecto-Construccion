# Proyecto-Construccion
Proyecto de Construcción. Laboratorio de Robótica y Automática.

Para la camara en ros2 humble:
sudo apt update

sudo apt install ros-humble-usb-cam

source /opt/ros/humble/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2

ros2 run rqt_image_view rqt_image_view o abrir rviz2 y add el topic image

Para mover el robot a un punto (en simulacion):

# Control del robot UR3e en simulación con ROS 2 + MoveIt

Este repositorio incluye la parte de control y planificación de movimiento para un UR3e usando ROS 2 Humble, MoveIt 2 y el driver oficial de Universal Robots.

Actualmente esta parte del proyecto permite:

- lanzar el robot en **simulación** con `fake_hardware`
- lanzar **MoveIt**
- mover el robot a un punto objetivo usando planificación
- usar una estrategia de movimiento tipo:
  - `PTP` hasta una prepose
  - `LIN` para bajar al objetivo

---

# Estructura usada

Se trabaja con:

- **ROS 2 Humble**
- **MoveIt 2**
- **Universal_Robots_ROS2_Driver**
- **ur_moveit_config**
- paquete propio: **robot_control**

---

# Requisitos

Tener instalados:

- ROS 2 Humble
- MoveIt 2
- Universal Robots ROS 2 Driver
- workspace compilado con `colcon`

---


# Compilar

Desde el workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/robotica_ws
colcon build --packages-select robot_control
source install/setup.bash


#Lanzar la simulación
#Terminal 1
source /opt/ros/humble/setup.bash
source ~/robotica_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=true initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

#Terminal 2
source /opt/ros/humble/setup.bash
source ~/robotica_ws/install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true

#Terminal 3
source /opt/ros/humble/setup.bash
source ~/robotica_ws/install/setup.bash
ros2 launch robot_control nodo90grados.launch.py



#NOTAS:
Notas importantes
La simulación se ejecuta con use_fake_hardware:=true, por lo que no hace falta tener el robot real conectado.
Es importante lanzar primero:
ur_control.launch.py
ur_moveit.launch.py
el nodo de robot_control
El controlador usado en simulación debe ser joint_trajectory_controller.
