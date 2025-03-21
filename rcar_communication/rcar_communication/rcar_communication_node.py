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
                                    PickObject
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
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from mecharm_interfaces.srv import CapturePose 

class RcarCommunication(Node):

    def __init__(self):
        super().__init__('rcar_communication_node')
        _package_name = "rcar_communication"    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', '1000000')
        self.declare_parameter('server_ip', '10.42.0.1')
        self.declare_parameter('socket_flag', True)
        self.declare_parameter('speed', 30)
        self.declare_parameter('model', 1)
        
        # Declare the home_pose parameter with a default value
        self.declare_parameter('home_pose', [40.0, 2.3, 265.1, 179.5, 90.0, 179.59])
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
        self.package_path =self.get_package_path(_package_name)
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
        self.lock = threading.Lock()
        self.mc = None
        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutually_callback= MutuallyExclusiveCallbackGroup()
        self.mecharm_error_code = None
        self.mecharm_error_msg = ""
        self.object_pose = MecharmCameraPose()
        self.capture_pose_client = None
        #self.connector_pose = CapturePose()


        try:
            if _has_socket:
                self.mc = MechArmSocket(_srver_ip,9000)
                
            else:
                self.mc = MechArm270(port=_port, baudrate=_baud)
            time.sleep(0.05)
        except Exception:
            self.get_logger().info(f"Connect failed with port -> {_port} and baud rate -> {_baud}")

        while self.check_power_state():
            time.sleep(3)
        if self.mc:
            self.mc.focus_all_servos()
            self.get_logger().info(f"next error state : {self.mc.read_next_error()}")
            self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        
        self.mecharm_joint_pub = self.create_publisher(msg_type=JointState,
                                                     topic="joint_states",
                                                     qos_profile=10
                                                     )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.camera_pose_sub = self.create_subscription(msg_type=MecharmCameraPose,topic='camera_pose', 
                                                        callback=self.camera_pose_callback,
                                                        qos_profile=10, 
                                                        callback_group=self.reentrant_callback)
        
        self.pick_srv = self.create_service(PickObject, 'pick_object', self.pick_callback,callback_group=self.mutually_callback)
        self.place_srv = self.create_service(PickObject, 'place_object', self.place_callback,callback_group=self.mutually_callback)
        self.pick_place_srv = self.create_service(PickObject, 'pick_place_object', self.pick_place_callback,callback_group=self.mutually_callback)
        self.pick_place_location_srv = self.create_service(PickObject, 'pick_place_move', self.move_callback,callback_group=self.mutually_callback)
        self.run_demo_srv = self.create_service(PickObject, 'run_demo', self.run_demo_callback,callback_group=self.mutually_callback)
        self.search_pick_srv = self.create_service(PickObject, 'search_demo', self.search_demo_callback,callback_group=self.mutually_callback)
        self.service_demo_srv = self.create_service(PickObject, 'service_demo_srv', self.service_demo_callback,callback_group=self.mutually_callback)
        self.capture_pose_client = self.create_client(CapturePose, 'capture_pose', callback_group=self.reentrant_callback)
        # Wait until the service is available
        while not self.capture_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the capture_pose service...')

        self.get_logger().info('capture_pose service is available.')

        self.publisher_timer = self.create_timer(timer_period_sec=0.05,
                                                 callback=self.publishers_callback, 
                                                 callback_group=self.reentrant_callback)
        self.get_logger().info(f"RCAR RADY TO GO")
        self.x_distance = 0
        self.y_distance = 0
        self.x_base_th = 200
        self.y_base_th = 50


    def service_demo_callback(self, req, res):
        error_code = self.demo_srv_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        
        return res

    def get_obj_pose(self):
        request = CapturePose.Request()

        # Call  service
        future = self.capture_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.pose.has_detection:
                pose = response.pose.pose
                self.get_logger().info(f"Detected Pose: x={pose.x}, y={pose.y}, z={pose.z}")
                return pose
            else:
                self.get_logger().info("No object detected.")
        else:
            self.get_logger().error("Service call failed.")
        return None
    


    def demo_srv_action(self):

        obj_pose = self.get_obj_pose()
        if obj_pose is None:
            return 90
        
        error_code = self.camera_service_pick_action()
        if error_code != 0:
            return error_code
        
        error_code = self.place_action()
        if error_code != 0:
             return error_code
        return 0

    def publish_cmd(self, x=0.0, y=0.0):
        vel_cmd = Twist()
        vel_cmd.linear.x = x
        vel_cmd.linear.y = y
        self.cmd_vel_pub.publish(vel_cmd)
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
    def do_base_correction(self, pose, cmd=False):
        if cmd:
            if not self.check_base_correction(pose[1]):
                return True

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
            self.get_logger().info(f"Base Moving to a new goal: {new_goal}")
            return not self.move_to_location(new_goal)
        return False  
        
    def search_pick_action(self):
        
        if self.mc is None:
            self.get_logger().error("Invalid arm controller")
            return 60  
        

        # Step 1: Clear previous errors and open the gripper
        self.mc.clear_error_information()
        if not self.mc.set_gripper_state(0, 30, 1):  # Open gripper
            self.get_logger().error("Failed to open gripper.")
            return 55
        self.get_logger().info("Gripper opened.")
        
        if not self.find_object():
            self.get_logger().error("Object not found during pick action.")
            return 50
        cam_pose = self.object_pose.pose
        base_pose = self.get_pose_about_base(cam_pose)
        
        self.get_logger().info(f"POSE W.R.T camera: {cam_pose} , POSE W.R.T BASE: {base_pose}")
        # Step 2: Check if the object is detected
        if not self.do_base_correction(base_pose):
            self.get_logger().error("Base correction Failed!")
            return 84
        
        if not self.find_object():
            self.get_logger().error("Object not found during pick action.")
            return 50
        cam_pose = self.object_pose.pose
        base_pose = self.get_pose_about_base(cam_pose)
        
        self.get_logger().info(f"POSE W.R.T camera: {cam_pose} , POSE W.R.T BASE: {base_pose}")
        if not self.validate_coordinates(base_pose):
            self.get_logger().error("Pose out of reach")
            return 65
        # Step 3: Move to the object's base pose
        if not self.sync_move_to_coords(base_pose, timeout=5):
            return self.mc.get_error_information()

        # Step 4: Move to the pick point (with offset)
        pick_point_x = base_pose[0] + self.pick_x_offset
        if not self.move_and_check_error(1, pick_point_x):
            return self.mc.get_error_information()

        self.get_logger().info("Pick point reached.")

        # Step 5: Close the gripper to pick the object
        if not self.mc.set_gripper_state(1, 30, 1):  # Close gripper
            self.get_logger().error("Failed to close gripper.")
            return 55

        self.get_logger().info("Gripper closed.")
        self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        # Success
        return 0
    
    def search_demo_callback(self, req, res):
        error_code = self.search_demo_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        
        return res
    
    
    def run_demo_callback(self, req, res):
        error_code = self.demo_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        
        return res
    def search_demo_action(self):
        error_code = self.move_to_location(self.pick_location)
        if error_code != 0:
            return error_code
        #time.sleep(10)
        error_code = self.search_pick_action()
        if error_code != 0:
            return error_code
        error_code = self.move_to_location(self.place_location)
        if error_code != 0:
            return error_code
        error_code = self.place_action()
        if error_code != 0:
            return error_code
        return 0
    
    def demo_action(self):
        error_code = self.move_to_location(self.pick_location)
        if error_code != 0:
            return error_code
        #time.sleep(10)
        error_code = self.camera_pick_action()
        if error_code != 0:
            return error_code
        error_code = self.move_to_location(self.place_location)
        if error_code != 0:
            return error_code
        error_code = self.place_action()
        if error_code != 0:
            return error_code
        return 0
        
    def move_callback(self, req, res):
        error_code = self.goto_pick_place_location()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        return res
    
    def move_to_location(self, location):
        qx, qy, qz, qw = self.euler_to_quat(0.0, 0.0, location[2])
        pick_goal_pose = PoseStamped()
        pick_goal_pose.header.frame_id = 'map'
        pick_goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pick_goal_pose.pose.position.x = location[0]
        pick_goal_pose.pose.position.y = location[1]
        pick_goal_pose.pose.orientation.x = qx
        pick_goal_pose.pose.orientation.y = qy
        pick_goal_pose.pose.orientation.z = qz
        pick_goal_pose.pose.orientation.w = qw
        return self.send_goal_pose(pick_goal_pose)
    
    def goto_pick_place_location(self):
        for location in [self.pick_location, self.place_location]:
            error_code = self.move_to_location(location)
            if error_code:
                return error_code
        return 0

    
    def send_goal_pose(self, pose):
        goal_pose = pose
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')
        self.navigator.goToPose(goal_pose)
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return 0
        elif result == TaskResult.CANCELED:
            return 81
        elif result == TaskResult.FAILED:
            return 82
        else:
            return 83
        
    def pick_callback(self,req, res):
        error_code = self.camera_pick_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        return res
    
    def place_callback(self, req, res):
        error_code = self.place_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        return res
    
    def pick_place_action(self):
        for action in [self.camera_pick_action, self.place_action]:
            error_code = action()
            if error_code:
                return error_code
        return 0
    
    def pick_place_callback(self, req, res):
        
        error_code = self.pick_place_action()
        if isinstance(error_code, int):
            res.error_code = error_code
        else:
            self.get_logger().info(f"The variable is not an integer. It is of type: {type(error_code).__name__}")
        res.error_message  = self.get_error_message(res.error_code)
        return res
        
    def get_transformed_pose(self, pose, target_frame='base', source_frame='camera_link'):
        
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
        
        
    def euler_to_quat(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        # Convert degrees to radians
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = rotation.as_quat()
        return qx, qy, qz, qw
    
    def get_pose_about_base(self, pose=MecharmCoords()):
        
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

            coords_data = [
                (base_pose.position.x - 0.125)*1000,
                (base_pose.position.y + 0.008)*1000,
                (base_pose.position.z + 0.01)*1000,
                179.0,
                90.0,
                179.0 
            ]
            return coords_data
        return[]
    
        
    def camera_pose_callback(self, camera_pose_msg):
        if camera_pose_msg:
            self.object_pose = camera_pose_msg
        else:
            self.object_pose.has_detection = False
            
    def send_home_and_check_detection(self):
        
        """Move to the home position and check if the object is detected."""
        if self.mc.send_coords(self.home_pose, self.default_speed, self.default_model):
            time.sleep(5)
        return self.object_pose.has_detection

    def adjust_and_check_detection(self, axis_index, offset):
        """
        Adjust a specific coordinate by an offset and check if the object is detected.
        
        Parameters:
            axis_index (int): The coordinate index to adjust (e.g., 2 for Y-axis, 3 for Z-axis).
            offset (float): The offset value to apply.
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        if self.mc and self.mc.send_coord(axis_index, offset, self.default_speed):
            time.sleep(5)
            return self.object_pose.has_detection
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
        if self.mc.send_angle(axis_index, offset, self.default_speed):
            time.sleep(5)
            return self.object_pose.has_detection
        return False

    def search_on_axis(self, axis_index):
        """
        Perform a search along a specific axis by moving in both positive and negative directions.
        
        Parameters:
            axis_index (int): The coordinate index to adjust (e.g., 2 for Y-axis, 3 for Z-axis).
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        # Positive direction
        self.get_logger().info(f"Searching .... axis {axis_index}")
        move_axis = self.home_pose[axis_index - 1] + self.search_offset
        if self.adjust_and_check_detection(axis_index, move_axis):
            return True
        if self.send_home_and_check_detection():
            return True
        move_axis = self.home_pose[axis_index - 1] - self.search_offset
        # Negative direction
        if self.adjust_and_check_detection(axis_index, move_axis):
            return True
        
        return self.send_home_and_check_detection()
    def search_on_joint(self, axis_index):
        """
        Perform a search along a specific axis by moving in both positive and negative directions.
        
        Parameters:
            axis_index (int): The coordinate index to adjust (e.g., 2 for Y-axis, 3 for Z-axis).
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        # Positive direction
        current_angles = self.mc.get_angles()
        self.get_logger().info(f"Searching .... axis {axis_index} with current angles {current_angles}")
        if current_angles:
            move_axis = current_angles[axis_index-1] - (self.search_offset + 10.0)
            if self.turn_and_check_detection(axis_index, move_axis):
                return True
            move_axis = current_angles[axis_index-1] + self.search_offset
            # Negative direction
            if self.turn_and_check_detection(axis_index, move_axis):
                return True
        
        return self.send_home_and_check_detection()

    def find_object(self):
        """
        Try to locate an object by moving the robotic arm along different axes and orientations.
        
        Returns:
            bool: True if the object is detected, otherwise False.
        """
        if not self.mc:
            return False
        
        # Step 1: Check at home position
        if self.send_home_and_check_detection():
            return True
        
        # Step 2: Search up and down
        if self.search_on_joint(1):  # Y-axis
            return True

        # Step 3: Search left and right
        if self.search_on_joint(5):  # Z-axis
            return True
        
        # Step 6: Reset to home position and final check
        return self.send_home_and_check_detection()
    def validate_coordinates(self, pose, threshold=340):
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
    
    def pick_service_action(self):
        """
        Executes the pick action for an object at the specified pose.

        Parameters:
            pose (list): The object's pose in camera coordinates.

        Returns:
            int: Error code indicating the outcome of the operation.
                0   - Success
                50  - Object not found during pick action
                55  - Gripper action failed
                >0  - Error code from self.mc.get_error_information()
        """
        if self.mc is None:
            self.get_logger().error("Invalid arm controller")
            return 60  
        

        # Step 1: Clear previous errors and open the gripper
        self.mc.clear_error_information()
        if not self.mc.set_gripper_state(0, 30, 1):  # Open gripper
            self.get_logger().error("Failed to open gripper.")
            return 55
        self.get_logger().info("Gripper opened.")
        
        obj_pose = self.get_obj_pose()  # Ensure pose is fetched
        if obj_pose is None:
            return 90
        base_pose = self.get_pose_about_base(obj_pose)
        self.get_logger().info(f"POSE W.R.T camera: {obj_pose} , POSE W.R.T BASE: {base_pose}")
        # Step 2: Check if the object is detected
        
        if not self.validate_coordinates(base_pose):
            self.get_logger().error("Pose out of reach")
            return 65

        # Step 3: Move to the object's base pose
        if not self.sync_move_to_coords(base_pose, timeout=5):
            return self.mc.get_error_information()

        # Step 4: Move to the pick point (with offset)
        pick_point_x = base_pose[0] + self.pick_x_offset
        if not self.move_and_check_error(1, pick_point_x):
            return self.mc.get_error_information()

        self.get_logger().info("Pick point reached.")

        # Step 5: Close the gripper to pick the object
        if not self.mc.set_gripper_state(1, 30, 1):  # Close gripper
            self.get_logger().error("Failed to close gripper.")
            return 55

        self.get_logger().info("Gripper closed.")
        
        self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        # Success
        return 0
    
    def pick_action(self):
        """
        Executes the pick action for an object at the specified pose.

        Parameters:
            pose (list): The object's pose in camera coordinates.

        Returns:
            int: Error code indicating the outcome of the operation.
                0   - Success
                50  - Object not found during pick action
                55  - Gripper action failed
                >0  - Error code from self.mc.get_error_information()
        """
        if self.mc is None:
            self.get_logger().error("Invalid arm controller")
            return 60  
        

        # Step 1: Clear previous errors and open the gripper
        self.mc.clear_error_information()
        if not self.mc.set_gripper_state(0, 30, 1):  # Open gripper
            self.get_logger().error("Failed to open gripper.")
            return 55
        self.get_logger().info("Gripper opened.")
        #error_code = self.mc.get_error_information()
        #if error_code != 0:
            #return error_code
        
        if not self.find_object():
            self.get_logger().error("Object not found during pick action.")
            return 50
        cam_pose = self.object_pose.pose
        base_pose = self.get_pose_about_base(cam_pose)
        self.get_logger().info(f"POSE W.R.T camera: {cam_pose} , POSE W.R.T BASE: {base_pose}")
        # Step 2: Check if the object is detected
        
        if not self.validate_coordinates(base_pose):
            self.get_logger().error("Pose out of reach")
            return 65

        # Step 3: Move to the object's base pose
        if not self.sync_move_to_coords(base_pose, timeout=5):
            return self.mc.get_error_information()

        # Step 4: Move to the pick point (with offset)
        pick_point_x = base_pose[0] + self.pick_x_offset
        if not self.move_and_check_error(1, pick_point_x):
            return self.mc.get_error_information()

        self.get_logger().info("Pick point reached.")

        # Step 5: Close the gripper to pick the object
        if not self.mc.set_gripper_state(1, 30, 1):  # Close gripper
            self.get_logger().error("Failed to close gripper.")
            return 55

        self.get_logger().info("Gripper closed.")
        #error_code = self.mc.get_error_information()
        #if error_code != 0:
            #return error_code

        # Step 6: Return to the home pose
        #if not self.sync_move_to_coords(self.home_pose, timeout=5):
            #return self.mc.get_error_information()
        self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        # Success
        return 0
    
    
    def camera_pick_action(self):
        return self.pick_action()
    
    def camera_service_pick_action(self):
        return self.pick_service_action()
        
    
    def place_action(self):
        self.mc.clear_error_information()
        if not self.sync_move_to_coords(self.place_pose, timeout=5):
            return self.mc.get_error_information()
        
        if not self.mc.set_gripper_state(0, 30, 1):  # Open gripper
            self.get_logger().error("Failed to open gripper.")
            return 55

        self.get_logger().info("Gripper opened.")
        #error_code = self.mc.get_error_information()
        #if error_code != 0:
            #return error_code
        self.mc.send_coords(self.home_pose, self.default_speed, self.default_model)
        return 0

    def sync_move_to_coords(self, coords, timeout=5):
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


        
    def publishers_callback(self):
        self.mecharm_joint_publisher()
            
    
    def mecharm_joint_publisher(self):
        joint_state_send = JointState()
        joint_state_send.header = Header()
        joint_state_send.header.stamp = self.get_clock().now().to_msg()
        
        # Define joint names
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

        
        
    def get_package_path(self, package_name):
        try:
            package_path = get_package_share_directory(package_name)
            self.get_logger().info(f"The path for the package '{package_name}' is: {package_path}")
            return package_path
        except PackageNotFoundError:
            self.get_logger().error(f"Package '{package_name}' not found.")
            return None
        
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
        
        
def main():
    rclpy.init()
    rcar_communication_node = RcarCommunication()
    rcar_communication_executor = MultiThreadedExecutor()
    rcar_communication_executor.add_node(rcar_communication_node)
    try:
        rcar_communication_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rcar_communication_executor.shutdown()
        rcar_communication_node.destroy_node()
        
        


if __name__ == '__main__':
    main()