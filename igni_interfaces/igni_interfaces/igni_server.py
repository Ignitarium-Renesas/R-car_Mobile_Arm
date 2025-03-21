#! /usr/bin/env python3
from mecharm_interfaces.srv import GripperStatus, SetCoords, SetAngles, PumpStatus, GetCoords, GetAngles
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
import rclpy
from rclpy.node import Node
import time
import math


class IgniInterfaces(Node):

    def __init__(self):
        super().__init__('igni_server')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', '1000000')
        self.declare_parameter('socket_flag', True)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().string_value
        self.get_logger().info(f"Connecting to port -> {port} with baud rate -> {baud}")
        
        #self.mc = MyCobot(port, baud)
        self.mc = None
        try:
            #self.mc = MechArm270(port, baud)
            self.mc = MechArmSocket("10.42.0.1",9000)
            time.sleep(0.05)
            self.mc.release_all_servos()
        except Exception:
            self.get_logger().info(f"Connect failed with port -> {port} and baud rate -> {baud}")
        
        self.switch_gripper_srv = self.create_service(GripperStatus, 'switch_gripper_status', self.switch_gripper_callback)
        self.set_coordinate_srv = self.create_service(SetCoords, 'set_coordinate', self.set_coordinate_callback)
        self.set_angle_srv = self.create_service(SetAngles, 'set_joint_angle', self.set_angle_callback)
        self.get_coordinate_srv = self.create_service(GetCoords, 'get_coordinate', self.get_coordinate_callback)
        self.get_angle_srv = self.create_service(GetAngles, 'get_joint_angle', self.get_angle_callback)
        self.switch_pump_srv = self.create_service(PumpStatus, 'switch_pump_status', self.switch_pump_callback)
        self.set_camera_pose_srv = self.create_service(SetCoords, 'set_camera_pose', self.set_camera_pose_callback)
        self.joint_state_pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        # Timer to periodically check and execute the state machine
        self.timer = self.create_timer(0.5, self.pub_callback)
        """ self.joint_subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        ) """
    def init_arm(self):
        if self.mc:
           pass 

    def pub_callback(self):
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
        

        # self.get_logger().info('radians: {}'.format(data_list))
        joint_state_send.position = data_list
        # print('data_list:',data_list)
        self.joint_state_pub.publish(joint_state_send)
    def euler_to_quat(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        # Convert degrees to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = rotation.as_quat()
        return qx, qy, qz, qw
    def set_camera_pose_callback(self, req, res):
        res.flag = False
        if self.mc is None:
            return res
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

            coords_data = [
                base_pose.position.x*1000,
                base_pose.position.y*1000,
                base_pose.position.z*1000,
                roll,
                pitch,
                yaw 
            ]
            sp = req.speed
            mod = req.model
            self.get_logger().info(f"Moving to : {coords_data}")
            result = self.mc.sync_send_coords(coords_data, sp, mod,timeout=5)
            if result:
                self.get_logger().info(f"reached to : {coords_data}")
                self.mc.set_gripper_state(1, 30)
                while self.mc.is_gripper_moving():
                    self.get_logger().info(f"Waiting gripper feedback:{self.mc.is_gripper_moving()}")
                    time.sleep(0.5)
                self.get_logger().info(f"Gripper closed")
                result = self.mc.sync_send_coords([20, -5, 233, 179.0, 63.0, -175.0 ], sp, mod,timeout=5)
                if result:
                    self.get_logger().info(f"reached to : [150.0, 0.0, 140.0, -1.0, 90.0, -1.0 ]")  
                result = self.mc.sync_send_coords(coords_data, sp, mod, timeout=5)
                if result:
                    self.get_logger().info(f"reached to : {coords_data}")
                    self.mc.set_gripper_state(0, 30)
                    while self.mc.is_gripper_moving():
                        self.get_logger().info(f"Waiting gripper feedback:{self.mc.is_gripper_moving()}")
                        time.sleep(0.5)
                self.get_logger().info(f"Gripper open")
                result = self.mc.send_coords([20, -5, 233, 179.0, 63.0, -175.0 ], sp, mod)
                self.get_logger().info(f"reached to : [150.0, 0.0, 140.0, -1.0, 90.0, -1.0 ]")
                res.flag = True
        return res
        
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
        
        

    def switch_pump_callback(self, request, response):
        response.flag = True
        return response
        
    def get_angle_callback(self, request, response):
        if self.mc:
            angles = self.mc.get_angles()
            response.joint_1 = angles[0]
            response.joint_2 = angles[1]
            response.joint_3 = angles[2]
            response.joint_4 = angles[3]
            response.joint_5 = angles[4]
            response.joint_6 = angles[5]
            self.get_logger().info("get_angles, over")
        return response
        
    def get_coordinate_callback(self, request, response):
        
        if self.mc:
            coords = self.mc.get_coords()
            response.x = coords[0]
            response.y = coords[1]
            response.z = coords[2]
            response.rx = coords[3]
            response.ry = coords[4]
            response.rz = coords[5]
            self.get_logger().info("get_coords, over")
        return response
        
    def set_angle_callback(self, request, response):
        response.flag = False
        angles = [
            request.joint_1,
            request.joint_2,
            request.joint_3,
            request.joint_4,
            request.joint_5,
            request.joint_6,
        ]
        sp = request.speed
        if self.mc:
            try:
                res = self.mc.sync_send_angles(angles, sp,timeout=15)
                self.get_logger().info(f"set_angles, over with response: {res}")
                if res:
                    response.flag = True
            except Exception:
                response.flag = False

        
    def set_coordinate_callback(self, request, response):
        response.flag = False
        coords = [
            request.x,
            request.y,
            request.z,
            request.rx,
            request.ry,
            request.rz,
        ]
        sp = request.speed
        mod = request.model

        if self.mc:
            try:
                self.get_logger().info(f"set_coords{coords}")
                res = self.mc.sync_send_coords(coords, sp, mod)
                if res:
                    response.flag = True
            except Exception:
                response.flag = False
        self.get_logger().info("set_coords, over")
        return response

    def switch_gripper_callback(self, request, response):
        response.flag = False
        if self.mc:
            gripper_mode = 0
            if request.status:
                gripper_mode = 1
            try:
                self.mc.set_gripper_state(gripper_mode, 30)
                response.flag = True
            except Exception:
                response.flag = False
        return response





def main():
    rclpy.init()

    interfaces_node = IgniInterfaces()

    rclpy.spin(interfaces_node)
    #interfaces_node.destroy_service(interfaces_node.switch_gripper_srv)
    #interfaces_node.destroy_subscription(interfaces_node.joint_subscription)
    interfaces_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

