# Proyecto de Construcción con Robot

## Descripción

Este proyecto implementa un sistema de percepción y manipulación robótica basado en visión por computador.
Incluye calibración de cámara, detección de piezas y control del robot para tareas de pick-and-place.

---

## Calibración

Se han implementado dos tipos de calibración:

### Calibración intrínseca

Permite modelar la cámara y corregir la distorsión de la imagen.

Archivo:

```
config/camera_intrinsics.yaml
```

---

### Calibración extrínseca (plano)

Permite transformar coordenadas de imagen a coordenadas del robot mediante una homografía.

Archivo:

```
config/plane_calibration.yaml
```

---

## Nodo de calibración

El paquete `calibration_services` implementa un nodo que carga la homografía y ofrece dos servicios:

### 1. pixel_to_robot_plane

Convierte coordenadas de imagen (u,v) en coordenadas del robot (x,y,z)

```bash
ros2 service call /pixel_to_robot_plane construccion_interfaces/srv/PixelToRobot "{u: 300.0, v: 250.0}"
```

---

### 2. robot_to_pixel

Convierte coordenadas del robot (x,y) en coordenadas de imagen (u,v)

```bash
ros2 service call /robot_to_pixel construccion_interfaces/srv/RobotToPixel "{x: -0.104, y: 0.310}"
```

---

## Ejecución

### 1. Compilar

```bash
colcon build
source install/setup.bash
```

---

### 2. Lanzar nodo de calibración

```bash
ros2 run calibration_services plane_transform_server
```

---

# Proyecto-Construccion

Sistema basado en ROS 2 para la detección de piezas mediante visión por computador y su transformación a coordenadas del plano de trabajo del robot mediante calibración cámara-robot.

---

## 📁 Estructura del repositorio

```
Proyecto-Construccion/
├── piece_detection_pkg/         # Nodo de detección de piezas
├── calibration_services/        # Nodo de calibración (transformaciones)
├── construccion_interfaces/     # Definición de servicios ROS 2
├── config/                      # Archivos de calibración
├── README.md
└── .gitignore
```

---

## ⚙️ Requisitos

* ROS 2
* OpenCV
* cv_bridge
* numpy
* yaml

---

## 🔧 Compilación

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Ejecución de la integración

### 1. Lanzar el nodo de calibración

```bash
ros2 run calibration_services plane_transform_server
```

Este nodo:

* Carga la matriz de homografía desde `config/plane_calibration.yaml`
* Proporciona el servicio:

  * `/pixel_to_robot_plane`

---

### 2. Comprobar que el servicio está activo

```bash
ros2 service list | grep pixel
```

---

### 3. Lanzar el nodo de detección

```bash
ros2 run piece_detection_pkg piece_detection_node
```

---

## 🔄 Flujo de uso

1. El nodo de detección solicita al robot ir a la pose `DetectaPiezasSueltas`.
2. Cuando el robot confirma que ha llegado:

   * Colocar la mesa vacía
   * Pulsar `b` para capturar el fondo
3. Colocar las piezas sobre la mesa.
4. Pulsar `p` para procesar una imagen fija.
5. El nodo:

   * Detecta las piezas
   * Calcula sus propiedades (posición, orientación, color, tamaño)
   * Llama al servicio `/pixel_to_robot_plane`
6. Se obtienen las coordenadas del robot para cada pieza.
7. Los resultados se publican en `/loose_pieces`.

---

## 🔌 Servicios

Definidos en `construccion_interfaces`:

* `PixelToRobot`
  Convierte coordenadas de imagen `(u, v)` en coordenadas del robot `(x, y, z)`.

* `RobotToPixel`
  Transformación inversa `(x, y, z)` → `(u, v)` (usado para validación).

---

## 📡 Tópicos relevantes

* `/image_raw` → imagen de la cámara
* `/robot_phase_feedback` → estado del robot
* `/vision_phase_command` → comandos al robot
* `/loose_pieces` → salida de detección
* `/foreground_loose` → máscara de segmentación

---

## 📊 Calibración

* Calibración intrínseca basada en patrón checkerboard.
* Calibración extrínseca mediante homografía planar.
* Transformación implementada mediante el nodo `plane_transform_server`.

---

## 📌 Estado del proyecto

* ✔️ Detección de piezas implementada
* ✔️ Calibración intrínseca y extrínseca completadas
* ✔️ Nodo de calibración implementado como servicio ROS 2
* ✔️ Integración básica detección ↔ calibración funcional


