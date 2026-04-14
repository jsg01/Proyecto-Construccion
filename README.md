# 🛠️ USO DE `movimiento_robot`

El nodo `movimiento_robot` permite controlar el robot UR3e utilizando MoveIt de dos formas:

- Ejecutando poses predefinidas mediante ángulos articulares
- Enviando objetivos cartesianos en el espacio

Además, el nodo añade automáticamente obstáculos al entorno de planificación:

- Una mesa
- Una pared frontal

---

## 🚀 1. Lanzamiento

Primero lanza el robot, MoveIt y RViz:

```bash
ros2 launch moveit_setup_robot_lab driver_ur34.launch.py use_fake_hardware:=true
ros2 launch moveit_setup_robot_lab move_group.launch.py
ros2 launch moveit_setup_robot_lab moveit_rviz.launch.py
```

Después lanza el nodo:

```bash
ros2 launch robot_control_julia movimiento_robot.launch.py
```

Para monitorizar el estado:

```bash
ros2 topic echo /robot_feedback
```

---

## 🎯 2. Poses predefinidas

Las poses predefinidas están definidas en el código mediante los valores de las articulaciones del robot.

### Poses disponibles

- `PoseIntermedia`
- `DetectaPiezas`
- `DetectaPiezasSueltas`

### Ejemplos

```bash
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'PoseIntermedia'}"
```

```bash
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'DetectaPiezas'}"
```

```bash
ros2 topic pub --once /ir_a_pose_guardada std_msgs/msg/String "{data: 'DetectaPiezasSueltas'}"
```

---

## 📍 3. Puntos cartesianos

Permite enviar un objetivo en el espacio cartesiano.

### Topic

```
/ir_a_punto_simple
```

### Formato del mensaje

```
"x y z modo yaw"
```

### Parámetros

- `x y z`: coordenadas objetivo
- `modo`:
  - `down`: herramienta hacia abajo
  - `front`: herramienta hacia delante
- `yaw`: rotación en radianes sobre el plano XY (solo en modo `down`)

### Ejemplos

Herramienta hacia abajo sin rotación:

```bash
ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.25 0.0 0.20 down 0.0'}"
```

Herramienta hacia abajo con rotación de 90°:

```bash
ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.25 0.0 0.20 down 1.57'}"
```

Herramienta hacia delante:

```bash
ros2 topic pub --once /ir_a_punto_simple std_msgs/msg/String "{data: '0.20 0.10 0.18 front 0.0'}"
```

---

## 🧭 4. Pose completa (PoseStamped)

Permite enviar directamente una pose completa.

### Topic

```
/ir_a_punto_pose
```

### Ejemplo

```bash
ros2 topic pub --once /ir_a_punto_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.25, y: 0.0, z: 0.20},
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  }
}"
```

---

## ⚠️ Notas

- Las poses predefinidas se ejecutan mediante espacio articular
- Los puntos cartesianos se ejecutan mediante objetivos en el espacio
- El nodo decide automáticamente si usar planificación cartesiana o joint
- Si un movimiento falla, se notificará en el topic `/robot_feedback`
