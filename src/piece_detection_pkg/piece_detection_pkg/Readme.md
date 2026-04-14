# Proyecto Construcción: detección, planificación y montaje de torre con robot

Este proyecto divide el sistema en varios nodos ROS 2 para que la detección, la planificación y el movimiento del robot estén separados y sean fáciles de depurar.

La idea es evitar un único script grande que haga todo a la vez, porque cuando algo falla no queda claro si el problema está en visión, en la lógica de selección de piezas, en la transformación píxel→robot, en el movimiento o en la pinza.

---

# Objetivo general

El sistema debe ser capaz de:

1. Llevar al robot a la pose `DetectaTorre`
2. Capturar una imagen de la torre original
3. Detectar las piezas de la torre y construir un plan de montaje
4. Llevar al robot a la pose `DetectaPiezasSueltas`
5. Detectar las piezas sueltas disponibles
6. Seleccionar la pieza que corresponde al siguiente paso del plan
7. Ir a cogerla
8. Llevarla a la zona de colocación
9. Depositarla en su posición correspondiente
10. Repetir el proceso hasta completar la torre

---

# Arquitectura

El sistema se divide en 4 nodos principales.

## 1. `tower_detection_node`

Responsabilidad:
- pedir al robot que vaya a la pose `DetectaTorre`
- esperar la confirmación del robot
- capturar imagen
- detectar las piezas de la torre
- construir la lista objetivo de colocación
- publicar el plan de torre

Este nodo solo se encarga de la detección de la torre y de generar el plan inicial.

El sistema clasifica las piezas en dos grupos simples: square y rectangular. Por otra parte, la clasificación de color distingue:
  red
  orange
  yellow
  green
  blue
  pink
  neutral (piezas con poca saturación)
  unknown (el color no se puede determinar con suficiente claridad)

---

## 2. `loose_piece_detection_node`

- El nodo manda al robot a DetectaPiezasSueltas
- Espera la confirmación de que el robot ha llegado
- Con la mesa vacía, el usuario captura el fondo
- El usuario coloca las piezas manualmente
- El usuario captura una foto fija con las piezas
- El nodo procesa únicamente esa foto

Este nodo solo ve las piezas disponibles.

Uso de teclas para capturar imágenes:

1. Esperar a que el robot llegue a DetectaPiezasSueltas, con la mesa vacía, pulsar: b
2. Colocar las piezas manualmente sobre la mesa y pulsar: p
3. Para repetir la captura con las mismas condiciones de fondo, pulsar: r

---

## 3. `assembly_coordinator_node`

Responsabilidad:
- actuar como cerebro del sistema
- recibir el plan de torre
- recibir las piezas sueltas detectadas
- seleccionar la pieza correcta para cada paso
- mandar órdenes al robot
- esperar feedback
- avanzar al siguiente paso

Este nodo gestiona la lógica global del montaje.

No debe:
- hacer OpenCV
- hacer cinemática
- abrir/cerrar la pinza directamente
- implementar movimientos detallados

---

## 4. `robot_executor_node`

Responsabilidad:
- recibir comandos de movimiento de alto nivel
- ejecutar las acciones físicas
- controlar movimiento y pinza
- publicar feedback de éxito o error

Este nodo encapsula el movimiento real del robot.

Debe encargarse de cosas como:
- ir a una pose nombrada
- aproximarse a una pieza
- bajar a coger
- cerrar pinza
- subir
- ir a la zona de colocación
- bajar a dejar
- abrir pinza
- volver a una pose segura

No debe:
- decidir la estrategia global
- recalcular el plan de montaje
- hacer detección visual

---

# Flujo completo del sistema

## Fase 1: detección de la torre

1. El sistema solicita al robot ir a la pose `DetectaTorre`
2. El robot confirma que ha llegado
3. Se captura una imagen de la torre
4. Se detectan las piezas de la torre
5. Se construye una lista ordenada de piezas objetivo
6. Se publica el plan de torre

---

## Fase 2: detección de piezas sueltas

7. El coordinador solicita al robot ir a la pose `DetectaPiezasSueltas`
8. El robot confirma que ha llegado
9. Se captura una imagen superior de las piezas sueltas
10. Se detectan las piezas candidatas disponibles

---

## Fase 3: selección de la pieza correcta

11. El coordinador toma la primera pieza pendiente del plan
12. Busca en las piezas sueltas una que coincida con:
   - color
   - tamaño (`size_class`)
13. Si hay varias candidatas, aplica un criterio de desempate

Ejemplos de criterio:
- mayor confianza
- mejor visibilidad
- más centrada
- mayor área
- más accesible

---

## Fase 4: pick

14. El coordinador manda una orden de pick al ejecutor del robot
15. El robot:
   - calcula la posición del objeto
   - se aproxima
   - baja
   - cierra pinza
   - sube
16. El robot publica feedback

---

## Fase 5: place

17. El coordinador manda una orden de place
18. El robot:
   - va a la zona de colocación
   - compensa según nivel o columna
   - baja
   - abre pinza
   - sube
19. El robot publica feedback

---

## Fase 6: siguiente paso

20. El coordinador marca la pieza como completada
21. Pasa al siguiente elemento del plan
22. Repite hasta completar toda la torre

---

# Máquina de estados

Para evitar comportamientos confusos, el `assembly_coordinator_node` debe implementarse como una máquina de estados.

Estados recomendados:

- `WAIT_TOWER_PLAN`
- `MOVE_TO_LOOSE_SCAN`
- `WAIT_LOOSE_SCAN_READY`
- `WAIT_LOOSE_PIECES`
- `SELECT_CANDIDATE`
- `WAIT_PICK_DONE`
- `WAIT_PLACE_DONE`
- `NEXT_STEP`
- `DONE`
- `ERROR`

Ventaja:
cada fase está claramente separada y el estado actual del sistema siempre es conocido.

Esto facilita muchísimo el debug.

---

# Comunicación entre nodos

Aunque de momento se puede usar `std_msgs/String` con JSON, es importante mantener una estructura clara y consistente.

## Topics recomendados

### Publicados por `tower_detection_node`
- `/tower_plan`

### Publicados por `loose_piece_detection_node`
- `/loose_pieces`

### Publicados por `assembly_coordinator_node`
- `/robot_command`

### Publicados por `robot_executor_node`
- `/robot_feedback`

---

# Formato de mensajes

## Ejemplo de plan de torre (`/tower_plan`)

```json
{
  "phase": "tower_plan_ready",
  "image_width": 640,
  "image_height": 480,
  "tower_pose_name": "DetectaTorre",
  "next_pose_name": "DetectaPiezasSueltas",
  "copy_offset_x_px": 0,
  "copy_offset_y_px": 0,
  "detected_pieces": [
    {
      "id": 0,
      "level": 0,
      "col": 0,
      "x": 150,
      "y": 220,
      "w": 40,
      "h": 42,
      "cx": 170,
      "cy": 241,
      "area": 1400,
      "size_class": "square",
      "color": "red"
    }
  ],
  "target_list": [
    {
      "step": 0,
      "piece_id": 0,
      "level": 0,
      "col": 0,
      "size_class": "square",
      "color": "red",
      "place_cx": 170,
      "place_cy": 241,
      "source_cx": 170,
      "source_cy": 241,
      "bbox": {
        "x": 150,
        "y": 220,
        "w": 40,
        "h": 42
      }
    }
  ]
}