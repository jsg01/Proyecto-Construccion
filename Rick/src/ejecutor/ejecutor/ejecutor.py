import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from construccion_interfaces.srv import MoveCartesian, GripperController, GrabPiece, MoveNamedPose
from std_srvs.srv import Trigger
from ur_msgs.srv import SetIO
from .helper_functions import open_gripper, close_gripper, approach_to_piece, move_to_pose, send_piece
import time
import yaml


poses = ["PoseIntermedia", "DetectaTorre","DetectaPiezasSueltas","DetectaPiezasSueltasNegative"]

class Ejecutor(Node):
    def __init__(self):
        super().__init__("NodeEjecutor")
        self.clients_and_callbacks_setup()
        self.service_setup()
        self.get_logger().info("Node Ejecutor fully setup2")
    
    def clients_and_callbacks_setup(self):
        self.move_cartesian_group = MutuallyExclusiveCallbackGroup()
        self.ejecutor_group = MutuallyExclusiveCallbackGroup()
        self.gripper_group = MutuallyExclusiveCallbackGroup()
        self.gripper_group_srv = MutuallyExclusiveCallbackGroup()

        self.move_cartesian_client = self.create_client(MoveCartesian, "/move_cartesian", callback_group=self.move_cartesian_group)
        while not self.move_cartesian_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().info("/move_cartesian not available, waiting again...")
        
        self.gripper_client = self.create_client(SetIO, "/io_and_status_controller/set_io", callback_group=self.gripper_group)
        while not self.gripper_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().info("/io_and_status_controller/set_io, waiting again...")
        
        self.move_named_pose_client = self.create_client(MoveNamedPose, "/move_named_pose", callback_group=self.move_cartesian_group)
        while not self.move_cartesian_client.wait_for_service(timeout_sec=20.0):
            self.get_logger().info("/move_named_pose, waiting again...")

    def service_setup(self):
        self.sequence_srv = self.create_service(Trigger, "/ejecutor/main", self.ejecutor_function, callback_group=self.ejecutor_group)
        self.gripper_controller_srv = self.create_service(GripperController, "/ejecutor/gripper", self.gripper_control, callback_group=self.gripper_group_srv)
        self.grab_piece_srv = self.create_service(GrabPiece, "/ejecutor/grab_piece", self.grabber_control, callback_group=self.ejecutor_group)
    
    async def execute_gripper(self, open: bool):
        self.get_logger().info("I got here")
        if open:
            req1, req2 = open_gripper()
            action = "opening"
        else:
            req1, req2 = close_gripper()
            action = "closing"
        resultFuture = self.gripper_client.call_async(req1)
        result = await resultFuture
        if not result.success:
            return False, f"Error in first request during {action}"
        resultFuture = self.gripper_client.call_async(req2)
        result = await resultFuture
        if not result.success:
            return False, f"Error in second request during {action}"
        time.sleep(1)
        return True, f"{action} successful"
    
    def execute_grabbing(self, piece_id: int, file: dict, Z: int):
        req = approach_to_piece(piece_id=piece_id, file=file, Z=Z)
        result = self.move_cartesian_client.call(req)
        return result.success, result.message
    
    def execute_send_piece(self, x: float, y: float, z: float, angle_deg: float):
        req = send_piece(x=x , y=y, z=z, angle_deg=angle_deg)
        result = self.move_cartesian_client.call(req)
        return result.success, result.message
    
    def execute_pose(self, pose: str):
        req = move_to_pose(pose)
        result = self.move_named_pose_client.call(req)
        return result.success, result.message

    async def gripper_control(self, request, response):
        response.success = False
        try:
            success, message = await self.execute_gripper(request.open)
            response.success = success
            response.message = message
        except Exception as e:
            response.message = str(e)
        return response
    
    def grabber_control(self, request, response):
        response.success = False
        try:
            with open("/tmp/detections.yaml", "r") as f:
                file = yaml.safe_load(f)
            if file is None:
                response.message = "The file was found, but it was empty"
                return response
            if (request.pieceid + 1) > len(file["detections"]):
                #self.get_logger().info(f"The len of file is {len(file)}")
                #self.get_logger().info(f"The len of file is {file}")
                response.message = "The id does not exists in the list"
                return response
            #success, message = self.execute_grabbing(file=file, piece_id=request.pieceid, Z=0.172)
            success, message = self.execute_grabbing(file=file, piece_id=request.pieceid, Z=0.150)
            response.success = success
            response.message = message
        except Exception as e:
            response.message = str(e)
        return response
    
    def get_ordered_detection_ids(self, plan_path="/tmp/tower_plan.yaml", detections_path="/tmp/detections.yaml"):
        with open(plan_path) as f:
            plan = yaml.safe_load(f)
        with open(detections_path) as f:
            detections = yaml.safe_load(f)
        ordered_ids = []
        for piece in plan["pieces"]:
            color = piece["color"]
            match = next(
                (d for d in detections["detections"] if d["color"] == color),
                None
            )
            if match is None:
                raise ValueError(f"No se encontró pieza con color {color}")
            ordered_ids.append(match["id"])
        return ordered_ids
    
    async def ejecutor_function(self, request, response):
        response.success = False
        with open("/tmp/detections.yaml", "r") as f:
            file = yaml.safe_load(f)
        try:
            ids = self.get_ordered_detection_ids()
            for num, id_picked in enumerate(ids):
                success, message = self.execute_grabbing(id_picked, file, 0.22)
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_grabbing(id_picked, file, 0.150)
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = await self.execute_gripper(open=False)
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_pose("DetectaPiezasSueltas")
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_pose("DetectaPiezasSueltasNegative")
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_send_piece(x=-0.167, y = -0.291, z=(0.150 + (0.019*num)), angle_deg=115.038)
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = await self.execute_gripper(open=True)
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_pose("DetectaPiezasSueltasNegative")
                if not success:
                    response.success = False
                    response.message = message
                    return response
                success, message = self.execute_pose("DetectaPiezasSueltas")
                if not success:
                    response.success = False
                    response.message = message
                    return response
            response.success = True
            response.message = "Task completed"
            return response
        except Exception as e:
            response.message = str(e)
        return response

    
    

def main():
    rclpy.init()
    Node = Ejecutor()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(Node, executor=executor)
    except KeyboardInterrupt:
        print('Shutting down')
        Node.destroy_node()
       
if __name__ == '__main__':
    main() 