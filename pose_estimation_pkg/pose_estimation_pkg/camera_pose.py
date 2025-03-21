import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mecharm_interfaces.msg import (MecharmAngles,
                                    MecharmCoords,
                                    MecharmSetAngles,
                                    MecharmSetCoords,
                                    MecharmGripperStatus,
                                    MecharmPumpStatus,
                                    MecharmErrorStatus,
                                    MecharmCameraPose
                                    )

from pose_estimation_pkg.libs.main import MainApp 
from pose_estimation_pkg.libs.recieve import Consumer

import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class CamPose(Node):
    def __init__(self):
        super().__init__('camera_pose')
        self.reentrant_callback = ReentrantCallbackGroup()
        self.mutual_callback = MutuallyExclusiveCallbackGroup()
        
        # ROS 2 subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 1)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 1)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 1)
        self.camera_pose_pub = self.create_publisher(msg_type=MecharmCameraPose,
                                                     topic="camera_pose",
                                                     qos_profile=5
                                                     )
        self.detected_img_pub = self.create_publisher(msg_type=Image,
                                                     topic="detection_img",
                                                     qos_profile=10
                                                     )
        self.publisher_timer = self.create_timer(timer_period_sec=0.02,callback=self.publishers_callback, callback_group=self.reentrant_callback)
        # self.socket_timer = self.create_timer(timer_period_sec=0.02,callback=self.socket_callback, callback_group=self.mutual_callback)


        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.radius = 3  # Radius in pixels for averaging
        self.pose = MecharmCameraPose()
        self.pose.pose.x = 0.0
        self.pose.pose.y = 0.0
        self.pose.pose.z = 0.0
        self.pose.pose.rx = 0.01
        self.pose.pose.ry = 0.01
        self.pose.pose.rz = 90.0
        self.pose.has_detection = False
        self.prv_detection = False
        self.display = False
        self.pose_3d = MainApp(object_name="white_connector",display=self.display)
        # self.cam_data = Consumer()

    # def socket_callback(self):
    #      self.color_image, self.depth_image = self.cam_data.decode_image()


    def get_camera_pose(self):
        if self.color_image is None or self.depth_image is None:
            return[0.0, 0.0, 0.0]
        camera_pose, rgb_img = self.pose_3d.cam_infer(self.color_image, self.depth_image)
        
        img = self.bridge.cv2_to_imgmsg(rgb_img, encoding='rgb8')
        self.detected_img_pub.publish(img)
        if camera_pose is not None:
            return [camera_pose[0], camera_pose[1], camera_pose[2]]
        
        return[0.0, 0.0, 0.0]
    
    def publishers_callback(self):
        cam_pose = self.get_camera_pose()
        self.pose.pose.x = cam_pose[0]
        self.pose.pose.y = cam_pose[1]
        self.pose.pose.z = cam_pose[2]
        self.pose.has_detection = self.pose_3d.has_detected
        if self.pose.has_detection != self.prv_detection:
            self.get_logger().info(f"Value changed: {self.pose.has_detection} , cam_pose: {cam_pose}" )
            self.prv_detection = self.pose.has_detection
        self.camera_pose_pub.publish(self.pose)
    
    def camera_info_callback(self, msg):
        # Retrieve camera intrinsic parameters
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'ppx': msg.k[2],
            'ppy': msg.k[5]
        }
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def stop_vis(self):
        #self.pose_3d.vis.destroy_window()
        pass
        

def main(args=None):
    rclpy.init(args=args)
    cam_node = CamPose()
    cam_executor = MultiThreadedExecutor()
    cam_executor.add_node(cam_node)
    try:
        cam_executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cam_executor.shutdown()
        # cam_node.cam_data.close()
        if cam_node.display:
            cam_node.stop_vis()
        cam_node.destroy_node()
if __name__ == '__main__':
    main()
