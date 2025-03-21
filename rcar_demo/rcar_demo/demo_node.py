import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from mecharm_interfaces.srv import CapturePose
from rclpy.action import ActionClient
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from mecharm_interfaces.srv import ArmMoveToPick,ArmLinearControl,ArmGripperControl,SearchObject
from mecharm_interfaces.srv import GoToGoal,SetInitialPose
from geometry_msgs.msg import PoseStamped
import time

class TaskNode(Node):
    def __init__(self):
        super().__init__('rcar_demo_node')

        # self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.declare_parameter('start_navigation', False)

        # self.initial_pose_client = self.create_client(SetInitialPose, 'set_initial_pose')
        # while not self.initial_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for service...')
        # self.get_logger().info('Service available. Ready to send request.')


        # self.nav2_client = self.create_client(GoToGoal, 'send_navigation_goal')
        # while not self.nav2_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().info('Waiting for navigation service...')

        self.capture_pose_client = self.create_client(CapturePose, 'capture_pose')
        while not self.capture_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the capture_pose service...')
        self.get_logger().info('capture_pose service is available.')
        
        self.arm_set_pose_client = self.create_client(ArmMoveToPick, 'set_pose') 
        while not self.arm_set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')
        self.get_logger().info('set_pose service is available.')

        self.arm_set_pick_pose_client = self.create_client(ArmMoveToPick, 'set_pick_pose')
        # Wait until the service is available
        while not self.arm_set_pick_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')
        
        self.arm_linear_control_client = self.create_client(ArmLinearControl, 'arm_linear_control')
        # Wait until the service is available
        while not self.arm_linear_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arm_linear_control service...')
        # self.gripper_client = self.create_client(ArmGripperControl, 'arm_gripper_control')
        
        self.gripper_client = self.create_client(ArmGripperControl, 'arm_gripper_control')
       # Wait until the service is available
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the arm_gripper_control service...')
        self.arm_search_object_client = self.create_client(SearchObject, 'arm_search_object')

        # Wait until the service is available
        while not self.arm_search_object_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the set_pose service...')




        # Declare a parameter to trigger the demo
        self.declare_parameter('start_demo', False)
        self.timer = self.create_timer(1.0, self.timer_callback)
        # triggered flag to ensure tasks run only when parameter toggles to True
        self.triggered = False
        self.goal=0.0
        self.initial_pose_not_set=True
        self.initial_delay=True
        self.gripper_state=False
        self.pick_completed=False
        self.drop_completed=False
        self.home_reached=True
        self.pose_estimated=False

    def timer_callback(self):
        start_demo = self.get_parameter('start_demo').get_parameter_value().bool_value
        if start_demo and not self.triggered:
            self.get_logger().info('Trigger received! Executing tasks sequentially...')
            self.execute_tasks()
            self.triggered = True
            # Reset parameter after tasks start so that you can trigger again later
            # self.set_parameters([Parameter('start_demo', Parameter.Type.BOOL, False)])
            self.get_logger().info('Tasks initiated.')
        elif not start_demo:
            self.triggered = False

    def execute_tasks(self):        
         # Task 1: setting initial pose
        # if(self.initial_pose_not_set):
        #     request = SetInitialPose.Request()
        #     request.x = 0.0
        #     request.y = 0.0
        #     request.z = 0.0
        #     request.roll = 0.0
        #     request.pitch = 0.0
        #     request.yaw = 0.0
        #     request.pose_set=True
        #     self.initial_pose_not_set=Falselinear

        #     self.get_logger().info('Task 1: Calling initial pose SRV. Setting Robot Pose...')
        #     time.sleep(3)

        #     future_init_pose = self.initial_pose_client.call_async(request)
        #     future_init_pose.add_done_callback(self.init_pose_cb)
        # else:
        #     self.initial_delay=False
        #     request = SetInitialPose.Request()
        #     # request.x = 0.0
        #     # request.y = 0.0
        #     # request.z = 0.0
        #     # request.roll = 0.0
        #     # request.pitch = 0.0
        #     # request.yaw = 0.0
        #     request.pose_set=False
        #     # self.initial_pose_not_set=False

        #     self.get_logger().info('Task 1: Calling initial pose SRV..second.')

        #     future_init_pose = self.initial_pose_client.call_async(request)
        #     future_init_pose.add_done_callback(self.init_pose_cb)

        self.get_logger().info('Going to home pose...')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 20.0
        arm_move_request.base_pose.y =  2.3
        arm_move_request.base_pose.z =  240.1
        arm_move_request.base_pose.rx =  179.5
        arm_move_request.base_pose.ry = 70.0
        arm_move_request.base_pose.rz = 179.59

        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        arm_move_future.add_done_callback(self.open_gripper)


    def open_gripper(self,future):
   
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        self.drop_completed=True
        gripper_future.add_done_callback(self.capture_pose)
        
            

    
    def init_pose_cb(self, future):
        self.get_logger().info('Task 1 Completed: Inital Pose Set..')
        self.get_logger().info("Task 2: Activating gripper...")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        gripper_future.add_done_callback(self.start_nav_cb)
    
    def start_nav_cb(self,future):
        self.get_logger().info("Task 2 Completed: Gripper Activated.")
        if(self.initial_pose_not_set):
            time.sleep(2)
        self.navigate_to_goal(future,pose_x=1.0,pose_y=0.0,pose_yaw=0.0)




    def nav2_callback(self, future):
        try:
            response = future.result()
            if response.success:  # Assuming the service response has a `success` flag
                self.get_logger().info('Navigation successful! Proceeding to next task.')
                self.capture_pose()
            else:
                self.get_logger().error('Navigation failed!')
        except Exception as e:
            self.get_logger().error(f"Failed to reach goal: {e}")

    def capture_pose(self,future):
        self.get_logger().info('Task 1: Calling CapturePose service...')
        request = CapturePose.Request()
        capture_future = self.capture_pose_client.call_async(request)
        
        # capture_future.add_done_callback(self.capture_pose_cb)
        capture_future.add_done_callback(self.capture_pose_response)
        
        # return
        # capture_future.object_detection_callback(self.capture_pose_response)

    # def capture_pose_cb(self,future):
        


    def capture_pose_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Task 1: CapturePose service call failed: {e}")
            return
        
        self.get_logger().info(f"has detcetion {response.pose.has_detection}")

        if response.pose.has_detection:
            self.pose_estimated=True
            pose = response.pose.pose
            self.get_logger().info(f"Task 1: Detected Pose: x={pose.x}, y={pose.y}, z={pose.z}, has_detection={response.pose.has_detection}")
        else:
            self.get_logger().info("Task 1: No object detected.")
            return
        # Task 2: Call AddTwoInts service
        self.get_logger().info('Task : Moving arm to pick pose')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = pose.x
        arm_move_request.base_pose.y = pose.y
        arm_move_request.base_pose.z = pose.z
        arm_move_request.base_pose.rx = pose.rx
        arm_move_request.base_pose.ry = pose.ry
        arm_move_request.base_pose.rz = pose.rz

        arm_move_future = self.arm_set_pick_pose_client.call_async(arm_move_request)
        arm_move_future.add_done_callback(self.move_arm_in_x)

    
    def move_arm_in_x(self,future):
        self.get_logger().info("moving in x to pick object")

        arm_move_x_request=ArmLinearControl.Request()
        arm_move_x_request.pick_point_x = 5.0      #40 default 
        arm_move_x_future = self.arm_linear_control_client.call_async(arm_move_x_request)
        arm_move_x_future.add_done_callback(self.move_x_cb)
    
    def move_x_cb(self,future):
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=False
        gripper_future=self.gripper_client.call_async(gripper_request)
        gripper_future.add_done_callback(self.move_post_pick)

    def move_post_pick(self,future):
        #retract the arm after picking

        self.get_logger().info('Setting post pick pose...')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 50.0
        arm_move_request.base_pose.y =  200.3
        arm_move_request.base_pose.z =  270.0
        arm_move_request.base_pose.rx =  -90.0
        arm_move_request.base_pose.ry = 45.0
        arm_move_request.base_pose.rz = -90.0

        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.pick_completed=True
        # arm_move_future.add_done_callback(self.navigate_to_goal(arm_move_future,pose_x=0.5,pose_y=0.0,pose_yaw=0.0))self.move_drop_pose()
        arm_move_future.add_done_callback(self.drop_object)


    def navigate_to_goal(self, future ,pose_x,pose_y,pose_yaw):
        # if(self.demo_restarted):
        #     self.get_logger().info("Task 3 : Going to Pick Location")
        #     self.demo_restarted=False
        # elif(self.pick_completed):
        #      self.get_logger().info("Task 3 : Going to Drop Location")
        #      self.pick_completed=False
        # elif(self.drop_completed):
        #      self.get_logger().info("All tasks completed, Going to Home")

        self.get_logger().info(f"Sending navigation goal to ")
        goal_msg = NavigateToPose.Goal()
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.orientation.w = 1.0  # Facing forward
        
        goal_msg.pose = pose
        
        self._navigate_action_client.wait_for_server()
        self.get_logger().info("Navigation action server available. Sending goal...")
        
        self._send_goal_future = self._navigate_action_client.send_goal_async(
            goal_msg, feedback_callback=self.navigate_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigate_goal_response_callback)

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received navigation feedback: {feedback}")

    def navigate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            return
        self.get_logger().info("Navigation goal accepted.")
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigate_get_result_callback)

    def navigate_get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info("Navigation succeeded.")

            if(self.pick_completed):
                self.move_drop_pose()
                self.pick_completed=False

            elif(not self.pose_estimated):
                self.capture_pose()
                self.pose_estimated=True
            elif(self.drop_completed):
                self.pick_completed=False
                self.drop_completed=False
                self.pose_estimated=True
                # self.go_to_home()
            else:
                return

               
            # self.navigate_to_goal(future,pose_x=0.0,pose_y=0.0,pose_yaw=0.0)
            

        except Exception as e:
            self.get_logger().error(f"Navigation action failed: {e}")
            return


    def pick_complete_cb(self):
         self.get_logger().error(f"Navigating to Drop Location")



    def move_drop_pose(self,future):

        #go to drop pose (arm)
        self.get_logger().info('dropping object')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 40.0
        arm_move_request.base_pose.y =  200.3
        arm_move_request.base_pose.z =  135.1
        arm_move_request.base_pose.rx =  179.5
        arm_move_request.base_pose.ry = 90.0
        arm_move_request.base_pose.rz = 270.0

        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.pick_completed=True
        arm_move_future.add_done_callback(self.drop_object)
        # self.gripper_state=True
  
    def go_to_home(self):
        #going to home
        # self.pick_completed=True
        # self.drop_completed=True
        self.navigate_to_goal(future=None,pose_x=0.0,pose_y=0.0,pose_yaw=0.0)

    def drop_object(self,future):
        self.get_logger().info("activating gripper")
        gripper_request=ArmGripperControl.Request()
        gripper_request.gripper_state=True
        gripper_future=self.gripper_client.call_async(gripper_request)
        self.drop_completed=True
        gripper_future.add_done_callback(self.move_to_home)
        
    def move_to_home(self,future):
        #retract the arm after picking

        self.get_logger().info('Going to home pose...')
        arm_move_request = ArmMoveToPick.Request()
        arm_move_request.base_pose.x = 20.0
        arm_move_request.base_pose.y =  2.3
        arm_move_request.base_pose.z =  240.1
        arm_move_request.base_pose.rx =  179.5
        arm_move_request.base_pose.ry = 70.0
        arm_move_request.base_pose.rz = 179.59

        arm_move_future = self.arm_set_pose_client.call_async(arm_move_request)
        self.set_parameters([Parameter('start_demo', Parameter.Type.BOOL, False)])
        

        # gripper_future.add_done_callback(self.navigate_to_goal(gripper_future,pose_x=0.0,pose_y=0.0,pose_yaw=0.0))



def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()