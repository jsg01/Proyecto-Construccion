# Proyecto-Construccion
Proyecto de Construcción. Laboratorio de Robótica y Automática.

Para la camara en ros2 humble:
sudo apt update

sudo apt install ros-humble-usb-cam

source /opt/ros/humble/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2

ros2 run rqt_image_view rqt_image_view

