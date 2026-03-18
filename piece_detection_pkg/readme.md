# 🧩 Detección de piezas con ROS2

Este proyecto permite detectar piezas (tipo LEGO) usando una cámara en ROS2 y obtener su posición para que un robot pueda utilizarlas.

---

# 🎯 Objetivo

El objetivo es:

* Detectar piezas en una imagen
* Obtener su posición (cx, cy)
* Calcular tamaño y orientación
* Publicar esta información en ROS2

---

# 🧠 ¿Cómo funciona?

El sistema sigue este flujo:

1. Una cámara (real o rosbag) publica imágenes en:

```
/image_raw
```

2. Este nodo:

* recibe la imagen
* detecta piezas
* calcula su posición

3. Publica resultados en:

```
/piece_detections
```

---

# ⚙️ PREPARACIÓN (ANTES DE TODO)

Abrir una terminal y ejecutar:

```bash
cd ~/ros2_ws
colcon build
```

---

# 🔄 IMPORTANTE: USO DE TERMINALES

👉 Este sistema necesita **3 o 4 terminales abiertas a la vez**

⚠️ En TODAS las terminales debes ejecutar:

```bash
source ~/ros2_ws/install/setup.bash
```

---

# 🎥 OPCIÓN 1: USAR CÁMARA USB

---

## 🟢 TERMINAL 1 — CÁMARA

Ver cámaras disponibles:

```bash
v4l2-ctl --list-devices
```

Buscar el dispositivo (ej: `/dev/video2`)

Ejecutar:

```bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -p video_device:=/dev/video2
```

👉 Publica imágenes en `/image_raw`

---

## 🔵 TERMINAL 2 — DETECCIÓN

```bash
ros2 run piece_detection_pkg node_detection
```

👉 Este nodo:

* recibe imágenes
* detecta piezas
* publica `/piece_detections`

---

## 🟡 TERMINAL 3 — VISUALIZAR IMAGEN

```bash
ros2 run rqt_image_view rqt_image_view
```

Seleccionar:

```
/image_raw
```

---

## 🟣 TERMINAL 4 — VER RESULTADOS

```bash
ros2 topic echo /piece_detections
```

---

# 📼 OPCIÓN 2: USAR ROSBAG (SIN CÁMARA)

---

## 🟢 TERMINAL 1 — ROSBAG

```bash
ros2 bag play my_bag --loop
```

👉 IMPORTANTE:

* `--loop` hace que el bag se repita continuamente
* sin esto, el sistema se para al terminar

---

## 🐢 OPCIONAL (más lento)

```bash
ros2 bag play my_bag --loop -r 0.5
```

---

## 🔵 TERMINAL 2 — DETECCIÓN

```bash
ros2 run piece_detection_pkg node_detection
```

---

## 🟡 TERMINAL 3 — VISUALIZAR

```bash
ros2 run rqt_image_view rqt_image_view
```

Seleccionar:

```
/image_raw
```

---

## 🟣 TERMINAL 4 — RESULTADOS

```bash
ros2 topic echo /piece_detections
```

---

# 📡 RESULTADO

El nodo publica en:

```
/piece_detections
```

Ejemplo:

```json
{
  "detections": [
    {
      "cx": 320,
      "cy": 240,
      "w": 50,
      "h": 48,
      "area": 2300,
      "angle_deg": 10.5,
      "size_class": "mediana"
    }
  ]
}
```

---

# 🧠 ¿Qué hace el código?

El algoritmo realiza:

1. Recorta una zona de la imagen (ROI)
2. Convierte a HSV
3. Detecta zonas oscuras
4. Aplica filtros para eliminar ruido
5. Detecta contornos
6. Filtra por:

   * tamaño
   * forma
7. Calcula:

   * centro (cx, cy)
   * orientación
   * tipo de pieza

---

# ⚠️ PROBLEMAS COMUNES

## ❌ No aparece nada

```bash
source ~/ros2_ws/install/setup.bash
```

---

## ❌ El rosbag se para

👉 Usa siempre:

```bash
ros2 bag play my_bag --loop
```

---


