#!/usr/bin/env python3
import json
import time
import threading
from typing import Any, Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from construccion_interfaces.srv import PixelToRobot, MoveNamedPose, MoveCartesian
from ur_msgs.srv import SetIO


class ExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__('executor_node')

        self.declare_parameter('pick_offset_x', 0.0)
        self.declare_parameter('pick_offset_y', 0.0)
        self.declare_parameter('place_offset_x', 0.0)
        self.declare_parameter('place_offset_y', 0.0)

        self.declare_parameter('tower_plan_topic', '/tower_plan')
        self.declare_parameter('loose_pieces_topic', '/loose_pieces')
        self.declare_parameter('command_topic', '/vision_phase_command')
        self.declare_parameter('feedback_topic', '/robot_phase_feedback')

        self.declare_parameter('pixel_to_robot_service', '/pixel_to_robot_plane')
        self.declare_parameter('move_named_pose_service', '/move_named_pose')
        self.declare_parameter('move_cartesian_service', '/move_cartesian')
        self.declare_parameter('set_io_service', '/io_and_status_controller/set_io')

        self.declare_parameter('safe_pose_name', 'PoseIntermedia')

        self.declare_parameter('z_approach', 0.30)
        self.declare_parameter('z_pick', 0.136)
        self.declare_parameter('z_place', 0.136)

        self.declare_parameter('motion_mode', 'down')
        self.declare_parameter('gripper_wait_sec', 0.5)
        self.declare_parameter('service_timeout_sec', 8.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('executor_tick_sec', 1.0)

        self.tower_plan_topic = str(self.get_parameter('tower_plan_topic').value)
        self.loose_pieces_topic = str(self.get_parameter('loose_pieces_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)

        self.pixel_to_robot_service = str(self.get_parameter('pixel_to_robot_service').value)
        self.move_named_pose_service = str(self.get_parameter('move_named_pose_service').value)
        self.move_cartesian_service = str(self.get_parameter('move_cartesian_service').value)
        self.set_io_service = str(self.get_parameter('set_io_service').value)
        self.safe_pose_name = str(self.get_parameter('safe_pose_name').value)

        self.z_approach = float(self.get_parameter('z_approach').value)
        self.z_pick = float(self.get_parameter('z_pick').value)
        self.z_place = float(self.get_parameter('z_place').value)
        self.pick_offset_x = float(self.get_parameter('pick_offset_x').value)
        self.pick_offset_y = float(self.get_parameter('pick_offset_y').value)
        self.place_offset_x = float(self.get_parameter('place_offset_x').value)
        self.place_offset_y = float(self.get_parameter('place_offset_y').value)
        self.motion_mode = str(self.get_parameter('motion_mode').value)
        self.gripper_wait_sec = float(self.get_parameter('gripper_wait_sec').value)
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        tick_sec = float(self.get_parameter('executor_tick_sec').value)

        self.tower_plan: Optional[Dict[str, Any]] = None
        self.loose_pieces: Optional[Dict[str, Any]] = None
        self.used_piece_ids: Set[int] = set()
        self.current_step: int = 0
        self.mission_started = False
        self.mission_finished = False
        self.mission_aborted = False
        self.mission_thread: Optional[threading.Thread] = None

        self.create_subscription(String, self.tower_plan_topic, self.tower_plan_callback, 10)
        self.create_subscription(String, self.loose_pieces_topic, self.loose_pieces_callback, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 10)

        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)

        self.pixel_to_robot_client = self.create_client(PixelToRobot, self.pixel_to_robot_service)
        self.move_named_pose_client = self.create_client(MoveNamedPose, self.move_named_pose_service)
        self.move_cartesian_client = self.create_client(MoveCartesian, self.move_cartesian_service)
        self.set_io_client = self.create_client(SetIO, self.set_io_service)

        self._wait_for_clients()
        self.timer = self.create_timer(tick_sec, self.executor_tick)

        self.get_logger().info('executor_node listo')
        self.get_logger().info(f'Escuchando tower plan en: {self.tower_plan_topic}')
        self.get_logger().info(f'Escuchando loose pieces en: {self.loose_pieces_topic}')
        self.get_logger().info(f'Escuchando comandos de fase en: {self.command_topic}')
        self.get_logger().info(f'Publicando feedback robot en: {self.feedback_topic}')

    def _wait_for_clients(self) -> None:
        required_clients = [
            (self.pixel_to_robot_client, self.pixel_to_robot_service),
            (self.move_named_pose_client, self.move_named_pose_service),
            (self.move_cartesian_client, self.move_cartesian_service),
        ]

        for client, name in required_clients:
            self.get_logger().info(f'Esperando servicio {name} ...')
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Servicio no disponible todavía: {name}')
            self.get_logger().info(f'Servicio disponible: {name}')

        if self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Servicio disponible: {self.set_io_service}')
        else:
            self.get_logger().warn(
                f'Servicio {self.set_io_service} no disponible. '
                'Continúo sin bloquear; la pinza se ignorará si no responde.'
            )

    def publish_robot_feedback(self, pose: str, status: str, message: str = '') -> None:
        msg = String()
        msg.data = json.dumps({
            'pose': pose,
            'status': status,
            'message': message,
        })
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'Feedback publicado: {msg.data}')

    def command_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parseando /vision_phase_command: {e}')
            return

        command = str(data.get('command', ''))
        pose = str(data.get('pose', ''))

        if command != 'go_to_pose' or not pose:
            self.get_logger().warn(f'Comando no reconocido: {msg.data}')
            return

        self.get_logger().info(f'Recibido comando go_to_pose: {pose}')

        def worker():
            ok = self.move_named_pose(pose)

            if ok:
                self.publish_robot_feedback(pose, 'reached', 'move_named_pose ok')
            else:
                self.get_logger().warn(
                    f'move_named_pose({pose}) falló o timeout, pero envío reached igualmente'
                )
                self.publish_robot_feedback(
                    pose,
                    'reached',
                    'fallback: se asume que el robot ya está en la pose'
                )

        threading.Thread(target=worker, daemon=True).start()

    def tower_plan_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parseando /tower_plan: {e}')
            return

        if 'target_list' not in data:
            self.get_logger().warn('Mensaje /tower_plan sin target_list')
            return

        self.tower_plan = data
        self.current_step = 0
        self.used_piece_ids.clear()
        self.mission_started = False
        self.mission_finished = False
        self.mission_aborted = False

        self.get_logger().info(
            f'Recibido tower plan con {len(self.tower_plan.get("target_list", []))} pasos.'
        )

    def loose_pieces_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parseando /loose_pieces: {e}')
            return

        if 'pieces' not in data:
            self.get_logger().warn('Mensaje /loose_pieces sin pieces')
            return

        self.loose_pieces = data
        self.get_logger().info(
            f'Recibidas {len(self.loose_pieces.get("pieces", []))} piezas sueltas.'
        )

    def executor_tick(self) -> None:
        if self.mission_finished or self.mission_aborted:
            return
        if self.tower_plan is None or self.loose_pieces is None:
            return
        if self.mission_started:
            return
        if not self.auto_start:
            return

        self.mission_started = True
        self.mission_thread = threading.Thread(target=self.execute_mission_safe, daemon=True)
        self.mission_thread.start()

    def execute_mission_safe(self) -> None:
        try:
            self.execute_mission()
        except Exception as e:
            self.get_logger().error(f'Excepción no controlada en misión: {e}')
            self.abort_mission('Excepción en execute_mission')

    def execute_mission(self) -> None:
        assert self.tower_plan is not None
        assert self.loose_pieces is not None

        target_list = self.tower_plan.get('target_list', [])
        if not target_list:
            self.get_logger().warn('No hay pasos en target_list. Nada que ejecutar.')
            self.mission_finished = True
            return

        self.get_logger().info('===== INICIO DE MISIÓN =====')

        if not self._safe_gripper_open_on_start():
            self.abort_mission('No se pudo abrir la pinza al inicio')
            return

        while self.current_step < len(target_list):
            target = target_list[self.current_step]
            self.get_logger().info(
                f'Paso {self.current_step + 1}/{len(target_list)} -> '
                f'color={target.get("color")} size={target.get("size_class")} '
                f'place=({target.get("place_cx")},{target.get("place_cy")}) '
                f'place_yaw_deg={target.get("place_yaw_deg", 0.0)}'
            )

            loose_piece = self.select_matching_piece(target)
            if loose_piece is None:
                self.abort_mission(
                    f'No se encontró pieza compatible para step={target.get("step", self.current_step)}'
                )
                return

            self.get_logger().info(
                f'Pieza seleccionada -> id={loose_piece.get("id")} '
                f'color={loose_piece.get("color")} size={loose_piece.get("size_class")} '
                f'pick=({loose_piece.get("pick_cx")},{loose_piece.get("pick_cy")}) '
                f'angle_deg={loose_piece.get("angle_deg", 0.0)}'
            )

            ok = self.execute_step(target, loose_piece)
            if not ok:
                return

            piece_id = int(loose_piece.get('id', -1))
            if piece_id >= 0:
                self.used_piece_ids.add(piece_id)

            self.current_step += 1

        self.get_logger().info('===== MISIÓN COMPLETADA =====')
        self.go_to_safe_pose()
        self.mission_finished = True

    def execute_step(self, target: Dict[str, Any], loose_piece: Dict[str, Any]) -> bool:
        pick_u = float(loose_piece['pick_cx'])
        pick_v = float(loose_piece['pick_cy'])
        place_u = float(target['place_cx'])
        place_v = float(target['place_cy'])

        pick_xy = self.pixel_to_robot(pick_u, pick_v)
        if pick_xy is None:
            self.abort_mission('Falló pixel_to_robot para pick')
            return False

        place_xy = self.pixel_to_robot(place_u, place_v)
        if place_xy is None:
            self.abort_mission('Falló pixel_to_robot para place')
            return False

        x_pick, y_pick, _ = pick_xy
        x_place, y_place, _ = place_xy

        x_pick += self.pick_offset_x
        y_pick += self.pick_offset_y
        x_place += self.place_offset_x
        y_place += self.place_offset_y

        pick_yaw_deg = float(loose_piece.get('angle_deg', 0.0))
        place_yaw_deg = float(target.get('place_yaw_deg', 0.0))

        self.get_logger().info(
            f'Pick robot=({x_pick:.4f}, {y_pick:.4f}, {self.z_pick:.4f}) yaw={pick_yaw_deg:.2f} deg'
        )
        self.get_logger().info(
            f'Place robot=({x_place:.4f}, {y_place:.4f}, {self.z_place:.4f}) yaw={place_yaw_deg:.2f} deg'
        )

        if not self.execute_pick(x_pick, y_pick, pick_yaw_deg):
            self.abort_mission('Falló secuencia de pick')
            return False

        if not self.execute_place(x_place, y_place, place_yaw_deg):
            self.abort_mission('Falló secuencia de place')
            return False

        return True

    def select_matching_piece(self, target: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        assert self.loose_pieces is not None
        pieces = self.loose_pieces.get('pieces', [])

        desired_color = str(target.get('color', 'unknown'))
        desired_size = str(target.get('size_class', 'unknown'))

        candidates: List[Dict[str, Any]] = []

        for piece in pieces:
            piece_id = int(piece.get('id', -1))
            if piece_id in self.used_piece_ids:
                continue

            if str(piece.get('color', 'unknown')) != desired_color:
                continue

            if str(piece.get('size_class', 'unknown')) != desired_size:
                continue

            candidates.append(piece)

        if not candidates:
            self.get_logger().warn(
                f'No hay coincidencia exacta color={desired_color}, size={desired_size}. '
                f'Se usará la primera pieza libre para poder probar.'
            )
            for piece in pieces:
                piece_id = int(piece.get('id', -1))
                if piece_id not in self.used_piece_ids:
                    candidates.append(piece)

        if not candidates:
            return None

        image_width = float(self.loose_pieces.get('image_width', 640))
        center_x = image_width / 2.0

        def score(piece: Dict[str, Any]) -> Tuple[float, float]:
            dx = abs(float(piece.get('pick_cx', piece.get('cx', center_x))) - center_x)
            area = -float(piece.get('area', 0.0))
            return dx, area

        candidates.sort(key=score)
        return candidates[0]

    def pixel_to_robot(self, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        req = PixelToRobot.Request()
        req.u = float(u)
        req.v = float(v)

        future = self.pixel_to_robot_client.call_async(req)
        if not self._spin_until_future(future, 'pixel_to_robot'):
            return None

        res = future.result()
        if res is None or not res.success:
            msg = getattr(res, 'message', 'sin respuesta válida') if res is not None else 'future sin resultado'
            self.get_logger().error(f'pixel_to_robot falló: {msg}')
            return None

        return float(res.x), float(res.y), float(res.z)

    def move_named_pose(self, pose_name: str) -> bool:
        req = MoveNamedPose.Request()
        req.pose_name = pose_name

        future = self.move_named_pose_client.call_async(req)
        if not self._spin_until_future(future, f'move_named_pose({pose_name})'):
            return False

        res = future.result()
        if res is None:
            self.get_logger().error('move_named_pose devolvió resultado vacío')
            return False

        if not res.success:
            self.get_logger().error(f'move_named_pose falló: {res.message}')
            return False

        self.get_logger().info(f'move_named_pose ok: {pose_name}')
        return True

    def move_cartesian(self, x: float, y: float, z: float, yaw_deg: float) -> bool:
        req = MoveCartesian.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw_deg = float(yaw_deg)
        req.modo = self.motion_mode

        future = self.move_cartesian_client.call_async(req)
        label = f'move_cartesian({x:.3f},{y:.3f},{z:.3f},{yaw_deg:.1f})'

        if not self._spin_until_future(future, label):
            return False

        res = future.result()
        if res is None:
            self.get_logger().error('move_cartesian devolvió resultado vacío')
            return False

        if not res.success:
            self.get_logger().error(f'move_cartesian falló: {res.message}')
            return False

        self.get_logger().info(
            f'move_cartesian ok -> x={x:.3f} y={y:.3f} z={z:.3f} yaw_deg={yaw_deg:.1f}'
        )
        return True

    def set_digital_output(self, pin: int, state: float) -> bool:
        req = SetIO.Request()
        req.fun = 1
        req.pin = int(pin)
        req.state = float(state)

        self.get_logger().info(f'SetIO -> fun={req.fun}, pin={req.pin}, state={req.state}')

        future = self.set_io_client.call_async(req)

        deadline = time.time() + 0.5
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().warn(
                    f'set_io no respondió, pero se continúa (pinza OK): pin={req.pin}, state={req.state}'
                )
                return True

        return True

    def _spin_until_future(self, future: Any, label: str) -> bool:
        deadline = time.time() + self.service_timeout_sec

        while rclpy.ok() and not future.done():
            time.sleep(0.05)

            if time.time() > deadline:
                self.get_logger().error(f'Timeout esperando {label}')
                return False

        return future.done()

    def attempt_once_retry(self, action_name: str, func, *args, **kwargs) -> bool:
        self.get_logger().info(f'Intentando {action_name} (1/2)')
        ok = func(*args, **kwargs)
        if ok:
            return True

        self.get_logger().warn(f'Falló {action_name}. Reintentando (2/2) ...')
        ok = func(*args, **kwargs)
        if ok:
            return True

        self.get_logger().error(f'Falló {action_name} tras 2 intentos.')
        return False

    def open_gripper(self) -> bool:
        ok1 = self.set_digital_output(17, 0.0)
        time.sleep(0.1)
        ok2 = self.set_digital_output(16, 1.0)
        time.sleep(self.gripper_wait_sec)
        return ok1 and ok2

    def close_gripper(self) -> bool:
        ok1 = self.set_digital_output(16, 0.0)
        time.sleep(0.1)
        ok2 = self.set_digital_output(17, 1.0)
        time.sleep(self.gripper_wait_sec)
        return ok1 and ok2

    def _safe_gripper_open_on_start(self) -> bool:
        return self.attempt_once_retry('open_gripper_inicio', self.open_gripper)

    def execute_pick(self, x_pick: float, y_pick: float, yaw_deg: float) -> bool:
        if not self.attempt_once_retry(
            'move_pick_approach', self.move_cartesian, x_pick, y_pick, self.z_approach, yaw_deg
        ):
            return False

        if not self.attempt_once_retry(
            'move_pick_descend', self.move_cartesian, x_pick, y_pick, self.z_pick, yaw_deg
        ):
            return False

        if not self.attempt_once_retry('close_gripper', self.close_gripper):
            return False

        if not self.attempt_once_retry(
            'move_pick_ascend', self.move_cartesian, x_pick, y_pick, self.z_approach, yaw_deg
        ):
            return False

        return True

    def execute_place(self, x_place: float, y_place: float, yaw_deg: float) -> bool:
        if not self.attempt_once_retry(
            'move_place_approach', self.move_cartesian, x_place, y_place, self.z_approach, yaw_deg
        ):
            return False

        if not self.attempt_once_retry(
            'move_place_descend', self.move_cartesian, x_place, y_place, self.z_place, yaw_deg
        ):
            return False

        if not self.attempt_once_retry('open_gripper', self.open_gripper):
            return False

        if not self.attempt_once_retry(
            'move_place_ascend', self.move_cartesian, x_place, y_place, self.z_approach, yaw_deg
        ):
            return False

        return True

    def go_to_safe_pose(self) -> None:
        self.attempt_once_retry('move_safe_pose', self.move_named_pose, self.safe_pose_name)

    def abort_mission(self, reason: str) -> None:
        if self.mission_aborted:
            return

        self.mission_aborted = True
        self.get_logger().error(f'===== MISIÓN ABORTADA ===== {reason}')

        self.attempt_once_retry('open_gripper_abort', self.open_gripper)
        self.go_to_safe_pose()

    def destroy_node(self):
        self.get_logger().info('Cerrando executor_node')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()