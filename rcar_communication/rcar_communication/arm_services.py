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
                                    GetAngles,
                                    PickObject,
                                    ArmMoveToPick,
                                    ArmLinearControl,
                                    ArmGripperControl,
                                    SearchObject,
                                    )
from mecharm_interfaces.msg import (MecharmAngles,
                                    MecharmCoords,
                                    MecharmSetAngles,
                                    MecharmSetCoords,
                                    MecharmGripperStatus,
                                    MecharmPumpStatus,
                                    MecharmErrorStatus,
                                    MecharmCameraPose
                                    )
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from pymycobot.mecharm270 import MechArm270
from pymycobot.mycobot280 import MyCobot280
from tf2_ros.transform_listener  import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot import MechArmSocket
from pymycobot import MyCobot280Socket
from rclpy.executors import MultiThreadedExecutor
import threading
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from mecharm_interfaces.srv import CapturePose 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class Pick_Object(Node):

    def __init__(self):
        super().__init__('arm_services')
        _package_name = "rcar_communication"    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', '1000000')
        self.declare_parameter('server_ip', '192.168.0.146')
        self.declare_parameter('socket_flag', True)
        self.declare_parameter('speed', 30)
        self.declare_parameter('model', 1)
        
        # Declare the home_pose parameter with a default value
        #self.declare_parameter('home_pose', [0, 45, 50, -90, 0, 48]) #48 degrees for the gripper    ........... for normal picking 
        self.declare_parameter('home_pose', [25, 5, -51, -35, 4, 75])        # .............................. for Bin Picking 
        self.declare_parameter('place_pose', [150.6, -138.3, 110.8, -70.01, 81.84, -85.9])
        
        # Declare start_pose and end_pose as parameters
        self.declare_parameter('pick_location', [1.115, 0.0, 0.0])
        self.declare_parameter('place_location', [1.8, 0.0, 0.0])
        self.declare_parameter('search_offset', 25)
        self.declare_parameter('pick_x_offset', 40)

        # Get the parameter values
        self.pick_x_offset = self.get_parameter('pick_x_offset').value
        self.search_offset = self.get_parameter('search_offset').value
        self.pick_location = self.get_parameter('pick_location').value
        self.place_location = self.get_parameter('place_location').value
        self.get_logger().info(f"Pick Location: {self.pick_location}")
        self.get_logger().info(f"Place Location: {self.place_location}")
        self.home_pose = self.get_parameter('home_pose').value
        self.get_logger().info(f"Home pose: {self.home_pose}")
        self.place_pose = self.get_parameter('place_pose').value
        self.get_logger().info(f"Place pose: {self.place_pose}")
        _port = self.get_parameter("port").get_parameter_value().string_value
        _baud = self.get_parameter("baud").get_parameter_value().string_value
        _has_socket = self.get_parameter('socket_flag').get_parameter_value().bool_value
        _srver_ip = self.get_parameter("server_ip").get_parameter_value().string_value
        self.default_speed = self.get_parameter("speed").get_parameter_value().integer_value
        self.default_model = self.get_parameter("model").get_parameter_value().integer_value
        self.get_logger().info(f"Connecting to port -> {_port} with baud rate -> {_baud}")

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        self.data_list = [0.0] * 14
        self.mc = None
        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutually_callback= MutuallyExclusiveCallbackGroup()
        self.mecharm_error_code = None
        self.mecharm_error_msg = ""
        self.object_pose = MecharmCameraPose()
        self.transformed_pose = None        
        #self.connector_pose = CapturePose()

        try:
            if _has_socket:
                self.mc = MyCobot280Socket(_srver_ip,9000)
                
            else:
                self.mc = MyCobot280(port=_port, baudrate=_baud)
            time.sleep(0.05)
        except Exception:
            self.get_logger().info(f"Connect failed with port -> {_port} and baud rate -> {_baud}")

        while self.check_power_state():
            time.sleep(3)
        if self.mc:
            self.mc.focus_all_servos()
            self.get_logger().info(f"next error state : {self.mc.read_next_error()}")
            #self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
            self.mc.send_angles(self.home_pose, self.default_speed)

        self.mecharm_joint_pub = self.create_publisher(msg_type=JointState,
                                                     topic="joint_states",
                                                     qos_profile=10
                                                     )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.move_to_pick_pose = self.create_service(ArmMoveToPick,'set_pick_pose', self.set_pick_pose_callback,callback_group=self.mutually_callback)

        self.move_to_pick_pose = self.create_service(ArmMoveToPick, 'set_pose', self.set_pose_callback,callback_group=self.mutually_callback)

        self.move_obj_loc = self.create_service(ArmLinearControl, 'arm_linear_control', self.linear_control_callback,callback_group=self.mutually_callback)

        self.gripper = self.create_service(ArmGripperControl, 'arm_gripper_control', self.gripper_control_callback,callback_group=self.mutually_callback)
        
        self.search_object = self.create_service(SearchObject, 'arm_search_object', self.search_obj_callback,callback_group=self.mutually_callback)
        self.relase_srv = self.create_service(Trigger, 'release_joint', self.move_and_release_callback)


        self.publisher_timer = self.create_timer(timer_period_sec=0.05,
                                                 callback=self.publishers_callback, 
                                                 callback_group=self.reentrant_callback)
        self.get_logger().info(f"RCAR RADY TO GO")


    
    def move_and_release_callback(self, request, response):
        
        angles = [2044, 1163, 2208, 1982, 3160, 2099]
        speeds = [0, 0, 0, 0, 0, 0]

        # Move the arm to the defined pose
        self.mc.set_encoders_drag(angles, speeds)

        time.sleep(2) 

        # Release servo motor 5
        self.get_logger().info("Releasing servo motor 5...")
        self.mc.release_servo(5)

        response.success = True
        response.message = "Successfully moved and released servo 1."
        return response


    def set_pick_pose_callback(self,req, res):
        self.mc.clear_error_information()
        self.get_logger().info("Move to base point for pick up.")

        
        res.is_pose_reachable = True
        res.do_base_correction = False

        self.transformed_pose = self.get_pose_about_base(req.base_pose) # MecharmCoords
        self.get_logger().info(f"Transformed Pose: {self.transformed_pose}")

        # Check if recievec pose is none
        if self.transformed_pose == None:
            self.get_logger().error("Pose is none")         # validate the pose
            res.error_code = False
            res.is_pose_reachable = False 
            return res

        # Checks whether the pose is within reach or not.
        if not self.validate_coordinates(self.transformed_pose):
            self.get_logger().error("Pose out of reach")
            new_goal, correction_response = self.do_base_correction(pose=self.transformed_pose)
            res.do_base_correction = correction_response
            res.is_pose_reachable = False 
            res.new_goal_coords = new_goal            
            return res
        
        res.error_code = self.sync_move_to_coords(self.transformed_pose, timeout=0.25)
        if not res.error_code:
            # res.error_code = self.mc.get_error_information()
            self.get_logger().info("Failed to reach Base point.")    
            res.error_code = False
            res.is_pose_reachable = False
            return res
        
        self.get_logger().info("Base point reached.")
        res.new_goal_coords = [999.0, 999.0, 999.0]
        return res
    

    def set_pose_callback(self,req, res):
        self.mc.clear_error_information() 
        self.get_logger().info("Move to base point for pick up.")

        res.is_pose_reachable = True

        transformed_pose = req.base_pose #MecharmCoords
        self.get_logger().info(f"Transformed Pose: {transformed_pose}")
        if transformed_pose == None:
            self.get_logger().error("Pose is none")
            res.error_code = False
            res.is_pose_reachable = False
            return res
        
        self.transformed_pose = [transformed_pose.x,transformed_pose.y,transformed_pose.z,
                            transformed_pose.rx,transformed_pose.ry,transformed_pose.rz]
        
        # Checks whether the pose is within reach or not
        if not self.validate_coordinates(self.transformed_pose):
            self.get_logger().error("Pose out of reach")
            res.is_pose_reachable = False 
            return res
        
        # Move arm
        res.error_code = self.sync_move_to_coords(self.transformed_pose, timeout=0.25)
        if not res.error_code:
            # res.error_code = self.mc.get_error_information()
            res.error_message = "Failed to reach Base point."
            self.get_logger().info("Failed to reach Base point.")    
            res.error_code = False
            return res
        
        self.get_logger().info("Base point reached.")
        return res


    def linear_control_callback(self,req,res):
        self.mc.clear_error_information() 

        self.get_logger().info("Move to object location for pick up.")
        transformed_pose = self.transformed_pose[0] if self.transformed_pose != None else self.mc.get_coords()[0]
        pick_point_x = transformed_pose + req.pick_point_x

        res.error_code = self.move_and_check_error(1, pick_point_x)
        if not res.error_code:
            self.get_logger().info("Failed to reach Pick point.")
            # res.error_code = self.mc.get_error_information()
            res.error_code = False
            return res
        
        self.get_logger().info("Pick point reached.")
        return res
    

    def gripper_control_callback(self, req, res):
        """
        Controlles the Mecharm Gripper.
        
        Parameters:

            req.gripper_state (Bool) : True for opening and False for closing the gripper 
        """
        self.mc.clear_error_information()

        gripper_flag = req.gripper_state
        if not gripper_flag:
            error_code = self.mc.set_gripper_state(1, 30, 1)
            res.error_code = True
            if not error_code:
                res.error_code = False
                self.get_logger().error("Failed to close gripper.")
                return res
            
            self.get_logger().info("Gripper closed.")
            return res
        else:
            error_code = self.mc.set_gripper_state(0, 30, 1)
            res.error_code = True
            if not error_code:
                res.error_code = False
                self.get_logger().error("Failed to open gripper.")
                return res

            self.get_logger().info("Gripper opened.")
            return res
        
        
    def search_obj_callback(self,req,res):
        """
        Try to locate an object by moving the robotic arm along different axes and orientations.
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        if not self.mc:
            res.search_status = False
            return res
        
        search_offset = req.search_offset if req.search_offset else 25
        
        # Step 1: Check at home position
        if req.search_direction == "H":
            res.search_status = self.send_home_and_check_detection()
            return res

        
        # Step 2: Search up & down
        if req.search_direction == "R":  # Y-axis
            res.search_status = self.search_positive_direction(1, search_offset)
            return res
        
        if req.search_direction == "L":  # Y-axis
            res.search_status = self.search_negative_direction(1, search_offset)
            return res

        # Step 3: Search left and right
        if req.search_direction == "T":  # Y-axis
            res.search_status = self.search_positive_direction(5, search_offset)
            return res
        
        if req.search_direction == "D":  # Y-axis
            res.search_status = self.search_negative_direction(5, search_offset)
            return res
        
    
   
    def publishers_callback(self):
        self.mecharm_joint_publisher()

    
    def sync_move_to_coords(self, coords, timeout=0.25):
        """
        Moves the robot to the specified coordinates synchronously.

        Parameters:
            coords (list): Target coordinates for the robot arm.
            timeout (int): Maximum time to wait for the motion to complete.

        Returns:
            bool: True if the motion succeeds, otherwise False.
        """
        if self.mc.sync_send_coords(coords, self.default_speed, self.default_model, timeout=timeout):
            if self.mc.get_error_information() == 0:
                return True
        self.get_logger().error("Failed to reach target coordinates.")
        return False
    
    
    def move_and_check_error(self, axis_index, target_position):
        """
        Moves the robot arm on a specific axis and checks for errors.

        Parameters:
            axis_index (int): The coordinate axis index (e.g., 1 for X-axis).
            target_position (float): The target position on the specified axis.

        Returns:
            bool: True if the motion succeeds without errors, otherwise False.
        """
        if self.mc.send_coord(axis_index, target_position, self.default_speed):
            if self.mc.get_error_information() == 0:
                return True
        self.get_logger().error(f"Error moving to position on axis {axis_index}.")
        return False
            
    
    def mecharm_joint_publisher(self):
        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.header.stamp = self.get_clock().now().to_msg()
        
        # Define joint names
        joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
            "joint6_flange_to_gripper_base",
            "gripper_controller",
            "gripper_base_to_gripper_left2",
            "gripper_left3_to_gripper_left1",
            "gripper_base_to_gripper_right3",
            "gripper_base_to_gripper_right2",
            "gripper_right3_to_gripper_right1",
            "gripper_camera_joint"
        ]
        
        # Default velocity and effort
        joint_state_send.velocity = [0.0]
        joint_state_send.effort = []
        
        # Initialize positions with zeros
        
        
        # Get joint angles from `mc` if available
        if self.mc:
            try:
                angles = self.mc.get_radians()
                # Check if `angles` is iterable
                if hasattr(angles, '__iter__'):
                    # Try to convert all elements in `angles` to float
                    try:
                        angles = [float(angle) for angle in angles]
                        self.data_list[:len(angles)] = angles  # Update with converted angles
                    except ValueError:
                        self.get_logger().warn("Not all elements in 'angles' are convertible to float.")
                else:
                    self.get_logger().warn("'angles' is not iterable.")
            except Exception as e:
                self.get_logger().warn(f"Error fetching joint angles: {e}")
        
        # Assign positions to the JointState message
        joint_state_send.position = self.data_list
        
        # Publish the joint states
        self.mecharm_joint_pub.publish(joint_state_send)


    def send_home_and_check_detection(self):
        """Move to the home position and check if the object is detected."""
        _ = self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        return True
    
    def search_positive_direction(self,axis_index, search_offset):
        current_angles = self.mc.get_angles()
        self.get_logger().info(f"Searching positive direction with current angles {current_angles}")
        if current_angles:
            move_axis = current_angles[axis_index - 1] - (search_offset + 10.0)
            if self.turn_and_check_detection(axis_index, move_axis):
                return True
        return False
            
    def search_negative_direction(self, axis_index, search_offset):
        current_angles = self.mc.get_angles()
        self.get_logger().info(f"Searching negative direction with current angles {current_angles}")
        if current_angles:
            move_axis = current_angles[axis_index - 1] + search_offset
            if self.turn_and_check_detection(axis_index, move_axis):
                return True
        return False
            

    def turn_and_check_detection(self, axis_index, offset):
        """
        Adjust a specific coordinate by an offset and check if the object is detected.
        
        Parameters:
            axis_index (int): The coordinate index to adjust (e.g., 2 for Y-axis, 3 for Z-axis).
            offset (float): The offset value to apply.
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        result = self.mc.send_angle(axis_index, offset, self.default_speed)
        return result
    
         
    def do_teleop(self, x_speed=0.0, y_speed=0.0, distance=0.0):
        """
        Optimized teleop function to move the robot with assisted teleoperation.
        
        Parameters:
        - x_speed (float): Linear speed along the x-axis.
        - y_speed (float): Linear speed along the y-axis.
        - distance (float): Distance to move before stopping.
        
        Returns:
        - bool: True if task completes, False otherwise.
        """
        self.navigator.assistedTeleop(time_allowance=15)

        # Initialize PoseStamped once
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Get the initial pose
        initial_pose = self.get_transformed_pose(pose, target_frame='map', source_frame='base_footprint')

        if not initial_pose:
            self.get_logger().warn("Failed to get the initial pose.")
            return False

        # Loop until the task is complete
        rate = self.navigator.create_rate(5)  # Equivalent to 5 Hz loop rate
        self.get_logger().info(f"distance: {distance}")
        while not self.navigator.isTaskComplete():
            current_pose = self.get_transformed_pose(pose, target_frame='map', source_frame='base_footprint')
            if current_pose:
                displacement = current_pose.position.x - initial_pose.position.x
                if abs(displacement) < distance:
                    self.publish_cmd(x=x_speed, y=y_speed)  # Publish velocity commands
                else:
                    self.get_logger().info("Target distance reached. Stopping teleop.")
                    self.publish_cmd()  # Stop the robot
                    self.navigator.cancelTask()  # Cancel assisted teleop
                    break
            else:
                self.get_logger().warn("Failed to get the current pose. Stopping teleop.")
                self.publish_cmd()  # Stop the robot
                break

            rate.sleep()  # Maintain loop timing

        self.publish_cmd()  # Stop the robot after exiting the loop
        return True
    
    def publish_cmd(self, x=0.0, y=0.0):
        vel_cmd = Twist()
        vel_cmd.linear.x = x
        vel_cmd.linear.y = y
        self.cmd_vel_pub.publish(vel_cmd)

        
    def do_base_correction(self, pose, cmd=False):
        self.get_logger().info("Calculating new goal for base correction")
        if cmd:
            if not self.check_base_correction(pose[1]):
                return None, True

            x_speed = 0.08 if pose[1] > 0 else -0.08
            return self.do_teleop(x_speed=x_speed, y_speed=0.0, distance=abs(pose[1]))
        #qx, qy, qz, qw = self.euler_to_quat(0.0, 0.0, 0.0)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]/1000.0
        goal_pose.pose.position.y = pose[1]/1000.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Get the initial pose
        map_pose = self.get_transformed_pose(goal_pose, target_frame='map', source_frame='base')
        if map_pose:
            x_value = map_pose.position.x
            y_value = map_pose.position.y - 0.15
            # Create a Rotation object from the quaternion
            #rotation = Rotation.from_quat([qx, qy, qz, qw])

            # Get Euler angles in radians
            #roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
            
            new_goal = [x_value, y_value, 0.0]
            self.get_logger().info(f"New goal: {new_goal}")
            return new_goal, True
        return [999.0, 999.0, 999.0], False
    
    def check_base_correction(self, y_value, min_y=50, max_y=1000):
        """
        Check if the base correction is required based on the Y-value.

        Parameters:
        y_value (float): The Y-value to check (e.g., object's Y-position relative to the base).
        min_y (float): The minimum allowable Y-value.
        max_y (float): The maximum allowable Y-value.

        Returns:
        bool: True if the Y-value is within range, False otherwise.
        """
        if min_y <= abs(y_value) <= max_y:
            return True
        else:
            return False

     
    def get_pose_about_base(self, pose=MecharmCoords()):
        self.get_logger().info(f"{pose.rx}, {pose.ry}, {pose.rz}")
        self.get_logger().info(f"{pose.x}, {pose.y}, {pose.z}")
        qx, qy, qz, qw = self.euler_to_quat(pose.rx, pose.ry, pose.rz)
        camera_pose = PoseStamped()
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        camera_pose.header.frame_id = 'camera_link'
        camera_pose.pose.position.x = pose.x
        camera_pose.pose.position.y = pose.y 
        camera_pose.pose.position.z = pose.z
        camera_pose.pose.orientation.x = qx
        camera_pose.pose.orientation.y = qy
        camera_pose.pose.orientation.z = qz
        camera_pose.pose.orientation.w = qw
        base_pose = self.get_transformed_pose(camera_pose)
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
            if pose.rx != 0.0 or pose.ry != 0.0 or pose.rz != 0.0:
                if yaw < 0:
                    yaw = yaw - 45.0
                    if yaw < -180.0:
                        yaw = -180.0 - yaw
                else:
                    yaw = yaw + 45.0
                    if yaw > 180.0:
                        yaw = -(yaw - 180.0)
                        
                coords_data = [
                    (base_pose.position.x - 0.0)*1000,
                    (base_pose.position.y - 0.0)*1000,  
                    (base_pose.position.z + 0.10)*1000,
                    roll,
                    pitch,
                    yaw
                ]
            else:
                coords_data = [
                    #(base_pose.position.x - 0.140)*1000,       for normal picking.....
                    #(base_pose.position.y - 0.007)*1000,       for normal picking.....
                    #(base_pose.position.z - 0.015)*1000,       for normal picking.....
                    (base_pose.position.x - 0.0)*1000,
                    (base_pose.position.y - 0.0)*1000,  
                    (base_pose.position.z + 0.10)*1000,
                    -176,            # -90 for normal picking....
                    0,               #48.0  for notmal picking...
                    -137             #-90  for normal picking....
                ]
            return coords_data
        return[]
    

    def euler_to_quat(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        # Convert degrees to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = rotation.as_quat()
        return qx, qy, qz, qw
    

    def get_transformed_pose(self, pose, target_frame='joint1', source_frame='camera_link'):
        
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # Target frame
                source_frame,  # Source frame
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
        

    def validate_coordinates(self, pose, threshold=400):
        """
        Validates that the list is not empty and all values in the list are within ±threshold.

        Parameters:
            pose (list or tuple): The pose to validate (e.g., [x, y, z, rx, ry, rz]).
            threshold (float): The threshold range for each element (±threshold).

        Returns:
            bool: False if the list is empty, or if any value exceeds ±threshold. True if all values are within range.
        """
        # Check if the list is empty
        if not pose or not isinstance(pose, (list, tuple)):
            self.get_logger().error("Pose not list***************")
            return False

        # Check if all values are within ±threshold
        for value in pose:
            if abs(value) > threshold:
                return False

        return True   
    

        
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
        
        # Return 1 if power is not on or there's an error
        return 1

        
    def get_error_message(self, error_code):
        # Check for known error ranges
        if error_code is None:
            return "Error code is None."
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
        if error_code == 50:
            return "Object not found during pick action."
        if error_code == 55:
            return "Gripper action Failed."
        if error_code == 60:
            return "Invalid arm controller."
        if error_code == 65:
            return "Object is out of reach."
        if error_code == 70:
            return "Camera pose empty."
        if error_code == 80:
            return "Navigation Goal succeeded!."
        if error_code == 81:
            return "Navigation Goal was canceled!."
        if error_code == 82:
            return "Navigation Goal failed!."
        if error_code == 83:
            return "Navigation Goal has an invalid return status!"
        if error_code == 84:
            return "Base correction failed"
        if error_code == 90:
            return "Object Pose Service call failed."
        
        else:
            return f"Unknown error code is {error_code}."
        

    


def main(args=None):
    rclpy.init(args=args)
    pick_node = Pick_Object()
    pick_executor = MultiThreadedExecutor()
    pick_executor.add_node(pick_node)
    try:
        pick_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pick_executor.shutdown()
        # cam_node.cam_data.close()
        # if pick_node.display:
        #     pick_node.stop_vis()
        pick_node.destroy_node()



if __name__ == "__main__":
    main()
