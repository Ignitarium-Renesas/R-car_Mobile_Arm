import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from mecharm_interfaces.srv import CapturePose
from pose_estimation_pkg.libs.main import MainApp
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class CamPose(Node):
    def __init__(self):
        super().__init__('camera_pose_srv')

        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutual_callback = MutuallyExclusiveCallbackGroup()

        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.classes = {0: "white_connector", 1: "blue_connector"}
        self.bridge = CvBridge()
        self.display = False
        self.pose_3d = MainApp(object_name="white_connector", display=self.display)

        self.declare_parameter('save_image', False)
        self.declare_parameter('class_id', 0)
        self.declare_parameter('6dof', False)
        self.declare_parameter("white_neg_x_pick_offset",0.30)
        self.declare_parameter("white_pos_x_pick_offset",0.10)
        self.declare_parameter("blue_neg_x_pick_offset",0.20)
        self.declare_parameter("blue_pos_x_pick_offset",0.0)
        

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)

        self.capture_pose_service = self.create_service(
            CapturePose, 'capture_pose', self.capture_pose_callback, callback_group=self.mutual_callback)


    def capture_pose_callback(self, request, response):
        self.get_logger().info("Inside capture pose callback, clearing old images.")

        self.color_image = None
        self.depth_image = None

        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_rect_raw', self.image_callback, 1, callback_group=self.reentrant_callback)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 1, callback_group=self.reentrant_callback)

        time.sleep(1)
        self.get_logger().info("Waiting for new images...")

        timeout = 3.0  # Timeout in seconds
        start_time = time.time()

        while self.color_image is None or self.depth_image is None:
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"Wait time: {elapsed_time:.2f}s")
            
            if elapsed_time > timeout:
                self.get_logger().warn("Timed out waiting for images.")
                response.pose.has_detection = False
                self.cleanup_subscriptions()
                return response
            
            rclpy.spin_once(self, timeout_sec=0.1) 

        save_image = self.get_parameter('save_image').get_parameter_value().bool_value
        dof_6d = self.get_parameter('6dof').get_parameter_value().bool_value
        class_id = self.get_parameter('class_id').get_parameter_value().integer_value
        object_name = self.classes[class_id]
        
        white_neg_x_pick_offset = self.get_parameter('white_neg_x_pick_offset').get_parameter_value().double_value
        white_pos_x_pick_offset = self.get_parameter('white_pos_x_pick_offset').get_parameter_value().double_value

        blue_neg_x_pick_offset = self.get_parameter('blue_neg_x_pick_offset').get_parameter_value().double_value
        blue_pos_x_pick_offset = self.get_parameter('blue_pos_x_pick_offset').get_parameter_value().double_value

        self.get_logger().info("Running Pose Estimation...")
        camera_pose, rgb_img = self.pose_3d.cam_infer(self.color_image, self.depth_image,
                                                     object_name, dof_6d, save_image,
                                                     white_neg_x_pick_offset= white_neg_x_pick_offset,
                                                     white_pos_x_pick_offset= white_pos_x_pick_offset,
                                                     blue_neg_x_pick_offset= blue_neg_x_pick_offset,
                                                     blue_pos_x_pick_offset= blue_pos_x_pick_offset
                                                     )

        if camera_pose is not None and (not dof_6d):
            response.pose.pose.x = camera_pose[0]
            response.pose.pose.y = camera_pose[1]
            response.pose.pose.z = camera_pose[2]
            response.pose.has_detection = self.pose_3d.has_detected
            self.get_logger().info(f"Detected Camera Pose: x={camera_pose[0]}, y={camera_pose[1]}, z={camera_pose[2]}")
        elif camera_pose is not None and dof_6d:
            response.pose.pose.x = camera_pose[0]
            response.pose.pose.y = camera_pose[1]
            response.pose.pose.z = camera_pose[2]
            response.pose.pose.rx = camera_pose[3]
            response.pose.pose.ry = camera_pose[4]
            response.pose.pose.rz = camera_pose[5]

            response.pose.has_detection = self.pose_3d.has_detected
            self.get_logger().info(f"Detected Camera Pose: x={camera_pose[0]}, y={camera_pose[1]}, z={camera_pose[2]}, rx={camera_pose[3]}, ry={camera_pose[4]}, rz={camera_pose[5]}")
        else:
            response.pose.has_detection = False

        self.cleanup_subscriptions()
        return response
    

    def cleanup_subscriptions(self):
        """Destroy all temporary subscriptions to ensure fresh images on next request."""
        if hasattr(self, 'image_sub'):
            self.destroy_subscription(self.image_sub)
        if hasattr(self, 'depth_sub'):
            self.destroy_subscription(self.depth_sub)

    def camera_info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'ppx': msg.k[2],
            'ppy': msg.k[5]
        }

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")


def main(args=None):
    rclpy.init(args=args)
    cam_node = CamPose()
    executor = MultiThreadedExecutor()
    executor.add_node(cam_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        cam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
