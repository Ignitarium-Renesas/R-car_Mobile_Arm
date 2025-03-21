import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mecharm_interfaces.srv import GripperStatus, SetCoords, GetCoords

class PickPlaceStateMachine(Node):
    def __init__(self):
        super().__init__('pick_place')
        
        # State variables
        self.state = 'WAIT_FOR_OBJECT'
        self.object_pose = None
        self.target_pose = None
        self.on_moving = False
        self.is_checking = False
        
        # Service clients
        self.set_camera_pose_client = self.create_client(SetCoords, 'set_camera_pose')
        self.get_coordinate_client = self.create_client(GetCoords, 'get_cordinate')
        self.switch_gripper_client = self.create_client(GripperStatus, 'switch_gripper_status')
        
        # Object pose subscriber
        self.object_pose_subscriber = self.create_subscription(
            Pose, 
            'object_pose',  # Replace with your actual topic name
            self.object_pose_callback, 
            10
        )
        
        # Wait for services to be available
        self.wait_for_services()
        
        # Timer to periodically check and execute the state machine
        self.timer = self.create_timer(0.5, self.state_machine)

    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        self.set_camera_pose_client.wait_for_service()
        self.get_coordinate_client.wait_for_service()
        self.switch_gripper_client.wait_for_service()
        self.get_logger().info('All services are now available.')

    def object_pose_callback(self, msg):
        # Store the object's pose for use in the state machine
        self.object_pose = msg
        self.get_logger().info(f'Received object pose: {msg}')
        if self.state == 'WAIT_FOR_OBJECT' and not self.on_moving:
            self.state = 'OPEN_GRIPPER'
            self.on_moving = True

    def call_set_coordinate(self, pose, callback):
        # Prepare and send the set_coordinate request
        request = SetCoords.Request()
        request.x = pose.position.x
        request.y = pose.position.y
        request.z = pose.position.z
        request.rx = -33.5
        request.ry = 84.83
        request.rz = -38.04
        request.speed = 50
        request.model = 0

        future = self.set_camera_pose_client.call_async(request)
        future.add_done_callback(callback)

    def call_switch_gripper(self, callback, status):
        # Prepare and send the switch_gripper_status request
        request = GripperStatus.Request()
        request.status = status
        future = self.switch_gripper_client.call_async(request)
        future.add_done_callback(callback)

    def check_reached_position(self, target_pose):
        # Call the get_coords service to check the current position
        self.is_checking = True
        request = GetCoords.Request()
        future = self.get_coordinate_client.call_async(request)
        future.add_done_callback(lambda future: self.compare_positions(future, target_pose))

    def compare_positions(self, future, target_pose):
        # Compare current position with target position
        response = future.result()
        if response:
            current_pose = Pose()
            current_pose.position.x = response.x
            current_pose.position.y = response.y
            current_pose.position.z = response.z
            tolerance = 0.05  # Define a small tolerance for position check
            if (abs(current_pose.position.x - target_pose.position.x) < tolerance and
                abs(current_pose.position.y - target_pose.position.y) < tolerance and
                abs(current_pose.position.z - target_pose.position.z) < tolerance):
                # If reached the target, proceed to the next state
                self.state = self.next_state
                self.get_logger().info(f"Reached target position for {self.next_state}")
                self.is_checking = False
            else:
                # Otherwise, re-check in the next cycle
                self.get_logger().info("Not yet at target position, re-checking...")

    def state_machine(self):
        # Main logic to execute the state machine
        
        if self.state == 'WAIT_FOR_OBJECT':
            self.get_logger().info('Waiting for object pose...')
        elif self.state == 'OPEN_GRIPPER':
            self.get_logger().info('Opening the gripper...')
            self.call_switch_gripper(self.on_gripper_opened, False)
            
        elif self.state == 'MOVE_TO_OBJECT':
            self.get_logger().info('Moving to object position...')
            pick_pose = Pose()
            pick_pose = self.object_pose
            pick_pose.position.x += 50.0
            self.target_pose = pick_pose
            self.next_state = 'CLOSE_GRIPPER'
            self.call_set_coordinate(self.target_pose, self.on_reach_object_position)

        elif self.state == 'CHECK_OBJECT_REACHED':
            self.check_reached_position(self.target_pose)

        elif self.state == 'CLOSE_GRIPPER':
            self.get_logger().info('Closing the gripper...')
            self.call_switch_gripper(self.on_gripper_closed, True)

        elif self.state == 'MOVE_TO_PLACE':
            self.get_logger().info('Moving to place position...')
            place_pose = Pose()
            place_pose = self.object_pose
            place_pose.position.x += 50.0
            self.target_pose = place_pose
            self.next_state = 'OPEN_GRIPPER_TO_PLACE'
            self.call_set_coordinate(self.target_pose, self.on_reach_place_position)

        elif self.state == 'CHECK_PLACE_REACHED':
            self.check_reached_position(self.target_pose)

        elif self.state == 'OPEN_GRIPPER_TO_PLACE':
            self.get_logger().info('Opening gripper to place object...')
            self.call_switch_gripper(self.on_gripper_opened_to_place, False)

        elif self.state == 'MOVE_BACK':
            self.get_logger().info('Moving back...')
            back_pose = Pose()
            back_pose.position.x -= 50.0
            self.target_pose = back_pose
            self.next_state = 'FINISH'
            self.call_set_coordinate(self.target_pose, self.on_reach_back_position)

        elif self.state == 'CHECK_BACK_REACHED':
            self.check_reached_position(self.target_pose)

        elif self.state == 'FINISH':
            self.get_logger().info('Pick and place operation completed.')
            self.state = 'WAIT_FOR_OBJECT'  # Reset state for next operation

    # Callback functions for each state transition
    def on_reach_pre_pick_position(self, future):
        self.state = 'CHECK_PICK_REACHED'

    def on_gripper_opened(self, future):
        self.state = 'MOVE_TO_OBJECT'

    def on_reach_object_position(self, future):
        self.state = 'CHECK_OBJECT_REACHED'

    def on_gripper_closed(self, future):
        self.state = 'MOVE_TO_PLACE'

    def on_reach_place_position(self, future):
        self.state = 'CHECK_PLACE_REACHED'

    def on_gripper_opened_to_place(self, future):
        self.state = 'MOVE_BACK'

    def on_reach_back_position(self, future):
        self.state = 'CHECK_BACK_REACHED'

def main(args=None):
    rclpy.init(args=args)
    pick_place_state_machine = PickPlaceStateMachine()
    rclpy.spin(pick_place_state_machine)
    pick_place_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
