USO DE MOVIMIENTO_ROBOT

El nodo movimiento_robot permite mover el UR3e de dos formas:

A una pose predefinida por ángulos articulares.
A un punto concreto en el espacio cartesiano.

Además, añade automáticamente obstáculos estáticos al entorno de planificación:

una mesa
una pared frontal
Lanzamiento

Primero lanza el robot, MoveIt y RViz:

ros2 launch moveit_setup_robot_lab driver_ur34.launch.py use_fake_hardware:=true
ros2 launch moveit_setup_robot_lab move_group.launch.py
ros2 launch moveit_setup_robot_lab moveit_rviz.launch.py

Después lanza el nodo de movimiento:

ros2 launch robot_control_julia movimiento_robot.launch.py

Para ver el estado de ejecución:

ros2 topic echo /robot_feedback
1. Ir a una pose predefinida

Las poses predefinidas se definen en el código mediante los ángulos de las 6 articulaciones.

Ejemplo de poses disponibles:

PoseIntermedia
DetectaPiezas
DetectaPiezasSueltas
Ejemplo
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'PoseIntermedia'}"
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'DetectaPiezas'}"
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'DetectaPiezasSueltas'}"
2. Ir a un punto concreto

Para enviar un objetivo cartesiano se usa el topic:

/ir_a_punto_simple

El formato del mensaje es:

"x y z modo yaw"

donde:

x y z son las coordenadas objetivo
modo puede ser:
down: herramienta hacia abajo
front: herramienta hacia delante
yaw es la rotación en radianes sobre el plano XY cuando se usa down
Ejemplos

Herramienta hacia abajo sin giro:

ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.25 0.0 0.20 down 0.0'}"

Herramienta hacia abajo girada 90°:

ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.25 0.0 0.20 down 1.57'}"

Herramienta hacia delante:

ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.20 0.10 0.18 front 0.0'}"
3. Enviar una pose completa

También se puede enviar una pose completa usando PoseStamped en el topic:

/ir_a_punto_pose
Ejemplo
ros2 topic pub --once /ir_a_punto_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.25, y: 0.0, z: 0.20},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  }
}"
Notas
Las poses predefinidas se ejecutan por ángulos articulares.
Los puntos concretos se ejecutan por objetivo cartesiano.
El nodo decide automáticamente si usar movimiento cartesiano o joint según la distancia al objetivo.
Si un movimiento no puede planificarse, se publica un mensaje de error en /robot_feedback.
