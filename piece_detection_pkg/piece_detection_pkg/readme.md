# Piece Detection Node (ROS2)

## 📌 Descripción

Este nodo detecta piezas (tipo LEGO) mediante visión por computador en ROS2.

El sistema utiliza **sustracción de fondo** para segmentar objetos y calcula:

* Posición (x, y)
* Orientación
* Tamaño (small, medium, large)

Compatible con:

* Cámara en tiempo real
* Rosbag

---

## 🚀 Ejecución

### 1. Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select piece_detection_pkg
source install/setup.bash
```

---

### 2. Ejecutar nodo

```bash
ros2 run piece_detection_pkg piece_detection_node_final \
  --ros-args \
  -p background_path:=/ruta/a/background.jpeg \
  -p bg_threshold:=0.08 \
  -p min_area:=500.0 \
  -p show_debug:=true
```

---

### 3. Ejecutar con rosbag

```bash
cd /ruta/al/rosbag
ros2 bag play .
```

---

## ⚠️ IMPORTANTE

La imagen de fondo debe:

* ser de la misma escena
* misma cámara
* misma posición
* misma iluminación

---

## 🔧 Parámetros

| Parámetro       | Descripción               |
| --------------- | ------------------------- |
| background_path | Ruta del fondo            |
| bg_threshold    | Sensibilidad de detección |
| min_area        | Área mínima de detección  |
| show_debug      | Visualización de debug    |

---

## 🧠 Funcionamiento

```text
Imagen → Sustracción de fondo → Segmentación → Contornos → Detección
```

---

## 📍 Estado del proyecto

✔ Detección robusta
✔ Separación de piezas
✔ Cálculo de orientación
✔ Preparado para integración con robot

---

## 🔜 Próximos pasos

* Calibración cámara (pixel → mundo)
* Integración con robot
* Pick & place automático

---

