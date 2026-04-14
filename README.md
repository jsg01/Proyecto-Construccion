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

## Estado actual

* ✔ Calibración intrínseca completada
* ✔ Calibración extrínseca completada
* ✔ Servicios de transformación implementados
* 🔜 Integración con nodo de detección


