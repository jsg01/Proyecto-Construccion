COMANDOS 

Para abrir camara:
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2

En otra terminal: 
rviz2
Add -> Topic -> Image

Para ver los fps reales de la cámara: 
ros2 topic hz /image_raw

Para grabar y verlo después:
ros2 bag record /image_raw
Ctrl+c para detener grabación


Para lo último (detectar tamaños de las piezas en tiempo real):
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2

En otra terminal:
ros2 run piece_detection_pkg piece_detection_sinBackground \
  --ros-args \
  -p bg_threshold:=0.08 \
  -p min_area:=500.0 \
  -p show_debug:=true
