#! /usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from mecharm_interfaces.srv import (GripperStatus,
                                    SetCoords,
                                    SetAngles,
                                    PumpStatus,
                                    GetCoords,
                                    GetAngles
                                    )
from mecharm_interfaces.msg import (MecharmAngles,
                                    MecharmCoords,
                                    MecharmSetAngles,
                                    MecharmSetCoords,
                                    MecharmGripperStatus,
                                    MecharmPumpStatus,
                                    MecharmErrorStatus
                                    )
from std_srvs.srv import Empty
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from pymycobot.mecharm270 import MechArm270
from tf2_ros.transform_listener  import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot import MechArmSocket
from rclpy.executors import MultiThreadedExecutor
import threading
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import json



class IgniInterfaces(Node):

    def __init__(self):
        super().__init__('pick_place')
        _package_name = "igni_interfaces"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', '1000000')
        self.declare_parameter('server_ip', '192.168.38.177')
        self.declare_parameter('socket_flag', True)
        self.declare_parameter('speed', 30)
        self.declare_parameter('model', 1)
        
        _port = self.get_parameter("port").get_parameter_value().string_value
        _baud = self.get_parameter("baud").get_parameter_value().string_value
        _has_socket = self.get_parameter('socket_flag').get_parameter_value().bool_value
        _srver_ip = self.get_parameter("server_ip").get_parameter_value().string_value
        self.default_speed = self.get_parameter("speed").get_parameter_value().integer_value
        self.default_model = self.get_parameter("model").get_parameter_value().integer_value
        self.get_logger().info(f"Connecting to port -> {_port} with baud rate -> {_baud}")
        
        
        self.lock = threading.Lock()
        self.mc = None
        self.error_state = 0
        self.prv_state = 0
        self.error_message =""
        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutually_callback= MutuallyExclusiveCallbackGroup()
        #self.home_pose = [40.0, 2.3, 230.1, 179.5, 90.0, 179.59]
        self.home_pose = [40.0, 2.3, 230.1, 179.5, 70.0, 179.59]
        self.place_pose = [150.6, -138.3, 110.8, -70.01, 81.84, -85.9]
        self.package_path =self.get_package_path(_package_name)
        self.home_json_file_path = self.package_path +'/data/home_pose.json'
        self.place_json_file_path = self.package_path +'/data/place_pose.json'
        self.camera_pose = SetCoords.Request()
        self.camera_pose.x = 0.0
        self.camera_pose.y = 0.0
        self.camera_pose.z = 0.0
        self.camera_pose.rx = 0.0
        self.camera_pose.ry = 0.0
        self.camera_pose.rz = 0.0
        self.camera_pose.speed = 50
        self.camera_pose.model = 1
        try:
            if _has_socket:
                self.mc = MechArmSocket(_srver_ip,9000)
                
            else:
                self.mc = MechArm270(port=_port, baudrate=_baud)
            time.sleep(0.05)
        except Exception:
            self.get_logger().info(f"Connect failed with port -> {_port} and baud rate -> {_baud}")

        while self.check_power_state():
            time.sleep(1)
        if self.mc:
            self.mc.focus_all_servos()
            self.get_logger().info(f"next error state : {self.mc.read_next_error()}")
            #self.mc.send_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 50)
            self.mc.send_coords(self.home_pose, 30, 0)
            
        self.system_state_pub = self.create_publisher(msg_type=MecharmErrorStatus,
                                                      topic='system_state',
                                                      qos_profile=10,
                                                      callback_group=self.reentrant_callback
                                                      )
        self.joint_state_pub = self.create_publisher(msg_type=JointState,
                                                     topic="joint_states",
                                                     qos_profile=10,
                                                     callback_group=self.reentrant_callback
                                                     )
        self.coordinates_pub = self.create_publisher(msg_type=MecharmCoords,
                                                     topic="coords_state",
                                                     qos_profile=5,
                                                     callback_group=self.reentrant_callback
                                                     )
        self.camera_pose_sub = self.create_subscription(msg_type=MecharmCoords,topic='camera_pose', callback=self.camera_pose_callback,qos_profile=10, callback_group=self.mutually_callback)
        self.pick_srv = self.create_service(SetCoords, 'pick_object', self.pick_callback,callback_group=self.reentrant_callback)
        self.place_srv = self.create_service(SetCoords, 'place_object', self.place_callback,callback_group=self.reentrant_callback)
        self.pick_place_srv = self.create_service(SetCoords, 'pick_place_object', self.pick_place_callback,callback_group=self.reentrant_callback)
        self.load_home_srv = self.create_service(Empty, 'load_home',callback=self.load_home_callback, callback_group=self.reentrant_callback)
        self.load_place_srv = self.create_service(Empty, 'load_place',callback=self.load_place_callback, callback_group=self.reentrant_callback)
        self.release_srv = self.create_service(Empty, 'release_arm',callback=self.release_callback, callback_group=self.reentrant_callback)
        self.exce_place_srv = self.create_service(Empty, 'exce_place_object', self.exce_place_callback,callback_group=self.reentrant_callback)
        self.exce_home_srv = self.create_service(Empty, 'exce_home', self.exce_home_callback,callback_group=self.reentrant_callback)
        self.exce_pick_place_srv = self.create_service(Empty, 'exce_pick_place_object', self.exce_pick_place_callback,callback_group=self.mutually_callback)
        
        self.publisher_timer = self.create_timer(timer_period_sec=0.5,callback=self.publishers_callback, callback_group=self.reentrant_callback)
        
    def validate_coordinates(self,tolerance=0.300):
        """
        Validates if x, y, and z values meet the specified conditions:
        - x: within ±0.3
        - y: within ±0.3
        - z: within ±0.3

        Args:
            x (float): The x-coordinate value to validate.
            y (float): The y-coordinate value to validate.
            z (float): The z-coordinate value to validate.

        Returns:
            bool: True if all conditions are met, False otherwise.
        """
        return abs(self.camera_pose.x) <= tolerance and abs(self.camera_pose.y) <= tolerance and abs(self.camera_pose.z) <= tolerance

    def exce_pick_place_callback(self, req, res):
        if self.camera_pose:
            if not self.validate_coordinates():
                self.get_logger().error("Camera pose out of range!")
            self.get_logger().info(f"Sending POSE \nx: {self.camera_pose.x}\n y:{self.camera_pose.y}\n z: {self.camera_pose.z}\n")
            self.pick_place_action(self.camera_pose)
        return res
        
    def camera_pose_callback(self,msg):
        if msg:
            """ if self.camera_pose.x >= 0:
                self.camera_pose.x = msg.x - 0.02
            else:
                self.camera_pose.y = msg.x 
            if self.camera_pose.y >= 0:
                self.camera_pose.y = msg.y - 0.02
            else:
                self.camera_pose.y = msg.y + 0.02 """
            self.camera_pose.x = msg.x
            self.camera_pose.y = msg.y
            self.camera_pose.z = msg.z
            self.camera_pose.rx = 0.0
            self.camera_pose.ry = 0.0
            self.camera_pose.rz = 0.0
            self.camera_pose.speed = 50
            self.camera_pose.model = 1
            #self.get_logger().info(f"Pose comming from camera: {msg}")
            
            
        
    def exce_home_callback(self, req, res):
        if self.mc:
            self._logger.info(f"sending home position: {self.home_pose}")
            self.mc.send_coords(self.home_pose, 30, 0)
        return res
    def exce_place_callback(self, req, res):
        if self.mc:
            self._logger.info(f"sending place position: {self.place_pose}")
            self.mc.send_coords(self.place_pose, 30, 0)
        return res
    def release_callback(self, req, res):
        if self.mc:
            self.mc.release_all_servos()
        return res
    
    def load_place_callback(self, req, res):
        if self.mc:
            self.place_pose = self.mc.get_coords()
            self._logger.info(f"place position loaded: {self.place_pose}")
            self.save_to_json(self.place_json_file_path, self.place_pose)
        return res
    def save_to_json(self, file_path, data):
        # Save the list of angle data to a JSON file
        with open(file_path, 'w') as json_file:
            json.dump(data, json_file)
        self.get_logger().info(f"Data written to {file_path}")
        
    def get_package_path(self, package_name):
        try:
            package_path = get_package_share_directory(package_name)
            self.get_logger().info(f"The path for the package '{package_name}' is: {package_path}")
            return package_path
        except PackageNotFoundError:
            self.get_logger().error(f"Package '{package_name}' not found.")
            return None
        
    def load_home_callback(self, req, res):
        if self.mc:
            self.home_pose = self.mc.get_coords()
            self._logger.info(f"home position loaded: {self.home_pose}")
            self.save_to_json(self.home_json_file_path, self.home_pose)
        return res

        
    def pick_place_callback(self, req, res):
        cam_pose = req
        res.flag =  self.pick_place_action(cam_pose)
        return res
    
    def place_action(self, pose,speed=30, model=0):
        base_pose = pose
        if self.mc and base_pose:
            self.get_logger().info("Going to place location")
            result = self.mc.sync_send_coords(base_pose,speed,model,timeout=5)
            if self.error_state:
                
                return False
            self.get_logger().info("Reached place location and oppening gripper")
            self.mc.set_gripper_state(0, 30, 1)
            #self.wait_gripper_action()
            if self.error_state:
                return False
            self.get_logger().info("Gripper opened and going to home")
            result = self.mc.sync_send_coords(self.home_pose, speed,model, timeout=5)
            if self.error_state:
                return False
            if result:
                return True
        return False
    
    def pick_action(self, pose):
        cam_pose = pose
        base_pose = self.get_object_pose(cam_pose)
        self.get_logger().info(f"POSE W.R.T BASE: {base_pose[0]}")
        if self.mc and base_pose:
            self.mc.clear_error_information()
            self.mc.set_gripper_state(0, 30, 1)
            #self.wait_gripper_action()
            if self.error_state:
                return False
            self.get_logger().info("Gripper opened")
            result = self.mc.sync_send_coords(base_pose[0],pose.speed,pose.model,timeout=5)
            if self.error_state:
                self.mc.send_coords(self.home_pose, 30, 0)
                return False
            self.get_logger().info("Pick point reached")
            pre_pick_pose = base_pose[0]
            if pre_pick_pose:
                result = self.mc.send_coord(1,pre_pick_pose[0]+ 40,30)
            if self.error_state:
                self.mc.send_coords(self.home_pose, 30, 0)
                return False
            self.mc.set_gripper_state(1, 30, 1)
            #self.wait_gripper_action()
            if self.error_state:
                self.mc.send_coords(self.home_pose, 30, 0)
                return False
            self.get_logger().info("Gripper Closed")
            result = self.mc.sync_send_coords(self.home_pose, pose.speed, pose.model,timeout=5)
            if self.error_state:
                return False
            self.get_logger().info("Home reached")
            if result:
                return True
        return False
    
    def pick_place_action(self, pose):
        self.get_logger().info(f"Sending POSE \nx: {pose.x}\n y:{pose.y}\n z: {pose.z}\n")
        result = self.pick_action(pose)
        if result:
            result = self.place_action(self.place_pose,pose.speed, pose.model)
            return result
        return False
    
    def wait_gripper_action(self):
        if self.mc:
            while self.mc.is_gripper_moving():
                    self.get_logger().warn("Gripper is moving")
                    time.sleep(0.5)
                    
    def place_callback(self, request, response):
        cam_pose = request
        #response.flag = self.place_action(cam_pose, cam_pose.speed, cam_pose.model)
        return response
        
    
    def pick_callback(self, request, response):
        cam_pose = request
        response.flag = self.pick_action(cam_pose)
        return response
        
        
    def transform_pose_to_base_frame(self, pose):
        
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform(
                'base',  # Target frame
                'camera_link',  # Source frame
                rclpy.time.Time(),  # Use the latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout duration
            )

            # Transform the PoseStamped message
            transformed_pose = do_transform_pose(pose.pose, transform)
            
            # Log the transformed pose
            self.get_logger().info(f"Transformed pose:\n"
                                   f"x: {transformed_pose.position.x}\n"
                                   f"y: {transformed_pose.position.y}\n"
                                   f"z: {transformed_pose.position.z}\n"
                                   f"orientation (qx, qy, qz, qw): {transformed_pose.orientation}")
            
            # Here you can use `transformed_pose` as needed in the base frame
            # For example, send this pose to control the robot arm
            return transformed_pose
            
        except Exception as e:
            self.get_logger().error(f"Could not transform pose: {e}")
            return None
        
    def euler_to_quat(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        # Convert degrees to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = rotation.as_quat()
        return qx, qy, qz, qw
    
    def get_object_pose(self, req):
        qx, qy, qz, qw = self.euler_to_quat(req.rx, req.ry, req.rz)
        camera_pose = PoseStamped()
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        camera_pose.header.frame_id = 'camera_link'
        camera_pose.pose.position.x = req.x
        camera_pose.pose.position.y = req.y 
        camera_pose.pose.position.z = req.z
        camera_pose.pose.orientation.x = qx
        camera_pose.pose.orientation.y = qy
        camera_pose.pose.orientation.z = qz
        camera_pose.pose.orientation.w = qw
        base_pose = self.transform_pose_to_base_frame(camera_pose)
        if base_pose:
            qx = base_pose.orientation.x
            qy = base_pose.orientation.y
            qz = base_pose.orientation.z
            qw = base_pose.orientation.w

            # Create a Rotation object from the quaternion
            rotation = Rotation.from_quat([qx, qy, qz, qw])

            # Get Euler angles in radians
            roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)

            # Log the result
            self.get_logger().info(f"Roll: {roll}°, Pitch: {pitch}°, Yaw: {yaw}°")
            #TODO: add roll, pitch, yow to coord if camera giving 6d pose

            coords_data = [
                (base_pose.position.x - 0.125)*1000,
                (base_pose.position.y + 0.015)*1000,
                (base_pose.position.z + 0.01)*1000,
                179.0,
                90.0,
                179.0 
            ]
            
            sp = req.speed
            mod = req.model
            return [coords_data, sp, mod]
        return[]
        
    
    def coords_publisher(self):
        coords_msg = MecharmCoords()
        if self.mc:
            coords = self.mc.get_coords()
            #self.get_logger().info(f"coords: {coords}")
            if coords:
                coords_msg.x = float(coords[0])
                coords_msg.y = float(coords[1])
                coords_msg.z = float(coords[2])
                coords_msg.rx = float(coords[3])
                coords_msg.ry = float(coords[4])
                coords_msg.rz = float(coords[5])
        self.coordinates_pub.publish(coords_msg)
    
    def mecharm_joint_publisher(self):
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            "joint1_to_base",
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "link6_to_gripper_base",
            "gripper_controller",
            "gripper_base_to_gripper_left2",
            "gripper_left3_to_gripper_left1",
            "gripper_base_to_gripper_right3",
            "gripper_base_to_gripper_right2",
            "gripper_right3_to_gripper_right1",
            "gripper_camera_joint"
        ] 
        joint_state_send.velocity = [0.0,]
        joint_state_send.effort = []
        joint_state_send.header.stamp = self.get_clock().now().to_msg()
        
        data_list = []
        if self.mc:
            try:
                angles = self.mc.get_radians()
                for _, value in enumerate(angles):
                    data_list.append(value)
                for i in range(8):
                    data_list.append(0.0)
            except Exception:
                pass
        else:
            for i in range(14):
                    data_list.append(0.0)
        # self.get_logger().info('radians: {}'.format(data_list))
        joint_state_send.position = data_list
        # print('data_list:',data_list)
        self.joint_state_pub.publish(joint_state_send)
        
    def get_error_message(self, error_code):
        # Check for known error ranges
        if error_code is None:
            return f"Unknown error code."
        if error_code == 0:
            return "No error message."
        elif 1 <= error_code <= 6:
            return f"The corresponding joint {error_code} exceeds the limit position."
        elif 16 <= error_code <= 19:
            return f"Collision protection.code"
        elif error_code == 32:
            return f"Kinematics inverse solution has no solution."
        elif 33 <= error_code <= 34:
            return f"Linear motion has no adjacent solution."
        else:
            return f"Unknown error code."
        
    def system_state_publisher(self):
        
        system_state = MecharmErrorStatus()
        error_code = -1
        if self.mc:
            error_code = self.mc.get_error_information()
            self.error_message = self.get_error_message(error_code)
        else:
            self.error_message = "Controller None, Check system"
        system_state.code = error_code
        system_state.message = self.error_message
        self.system_state_pub.publish(system_state)
        with self.lock:
            self.error_state = error_code
        if error_code not in [-1, 0] and error_code != self.prv_state:
            self.get_logger().error(f"Code : {error_code}, Message : {self.error_message}")
        self.prv_state =  error_code
            
        
    def publishers_callback(self):
        self.mecharm_joint_publisher()
        self.system_state_publisher()
        #self.coords_publisher()
        
        
    def check_power_state(self):
        """
        Checks the current power state of the machine controller (mc) once.
        Logs different messages based on the power state:
            - Logs info if power is on and returns 0.
            - Logs a warning if power is off.
            - Logs an error if there's an issue retrieving the power state or if mc is None.

        Returns:
            int: 0 if power is on, -1 otherwise.
        """
        # Ensure that the machine controller (mc) instance exists
        if self.mc:
            # Retrieve the current power state
            power_state = self.mc.is_power_on()
            
            # Conditional handling based on the power state
            if power_state == 1:
                # Power is on, log info and return 0
                self.get_logger().info('Power is on. Proceeding with the subsequent program...')
                return 0
            elif power_state == 0:
                # Power is off, log a warning
                self.get_logger().warning('Power is off. Waiting for power-on state...')
                
            elif power_state == -1:
                # Error in retrieving power state, log an error
                self.get_logger().error('Power state returned an error. Please check the system.')
                
        else:
            # If machine controller (mc) is None, log an error
            self.get_logger().error('Controller is None, Please check the system.')
        
        # Return -1 if power is not on or there's an error
        return 1


def main():
    rclpy.init()
    interfaces_node = IgniInterfaces()
    interfaces_executor = MultiThreadedExecutor()
    interfaces_executor.add_node(interfaces_node)
    try:
        interfaces_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        interfaces_executor.shutdown()
        interfaces_node.destroy_node()
        #rclpy.shutdown()


if __name__ == '__main__':
    main()
