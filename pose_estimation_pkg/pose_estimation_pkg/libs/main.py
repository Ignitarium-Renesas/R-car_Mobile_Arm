#from pcd import PointCloud
from pose_estimation_pkg.libs.pcd import PointCloud
from pose_estimation_pkg.libs.icp import IcpRegistration
from pose_estimation_pkg.libs.utils.utils import select_device, get_leftmost_bbox


from pose_estimation_pkg.libs.utils.display_utils import put_texts,get_random_colors
from pose_estimation_pkg.libs.detection import Detection
from ament_index_python.packages import get_package_share_directory
from pose_estimation_pkg.libs.utils.json_utils import read_json


import cv2
import os
import open3d as o3d
import copy
import numpy as np


class MainApp:
    
    def __init__(self,object_name, device="auto",cam_name="D405_848_C2",inference = False,display=False):
        self.display = display
        self.object_name = object_name
        self.cam_name = cam_name    
        self.window_name = "Pose Estimation"
        package_name = "pose_estimation_pkg"  # Replace with the name of your package
        self.package_path = get_package_share_directory(package_name)
        self.stop_exec = True
        self.inference = inference
        self.debug = False
        self.ratio = 0.15
        self.classes = ["white_connector", "blue_connector"]
        self.center_point = [0.0, 0.0, 0.0]
        self.has_detected = False
        self.detection_count = 0
        self.no_detection_count = 0

        self.device = select_device(device)
        self.cam_config = self.get_cam_config(cam_name=self.cam_name)
        self.load_instances()


    def get_cam_config(self,cam_name="D405_848_C2",config_path="cam_config.json"):
        config_path = f"{self.package_path}/libs/{config_path}"
        config = read_json(config_path)
        config = config[cam_name]
        return config


    def set_paths(self,):
        if not self.inference:
            self.train_path = f"{self.package_path}/libs/datasets/{self.object_name}/data/train"
            os.makedirs(self.train_path, exist_ok=True)
            self.val_path = f"{self.package_path}/libs/datasets/{self.object_name}/data/val"
            os.makedirs(self.val_path, exist_ok=True)
        

    def load_instances(self):
        self.pointcloud = PointCloud(self.cam_config["camera_intrinsic"],depth_scale=self.cam_config["depth_scale"],
                                     clipping_distance_in_meters=self.cam_config["clipping_distance_in_meters"], display=False)
        
        self.icp = IcpRegistration(pointcloud=self.pointcloud,package_path=self.package_path)


        weight_path = f"{self.package_path}/libs/weights/detection/yolo/combined_weights/yolo_weights.pt"
        self.model = Detection(detection_type="yolo",device=self.device)
        self.model.set_object("combined_connectors",weight_path)

        #visualisation
        if self.display:
            self.vis_pcd = o3d.geometry.PointCloud()
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.frame_count = 0
    

    def get_detection(self, rgb_image,object_name):
        annotations = self.model.infer(data=rgb_image)
        bbox = get_leftmost_bbox(annotations=annotations,object_name=object_name)
        return bbox

    def cam_infer(self,color_image=None, depth_image=None,object_name="white_connector",Dof_6d=False,save_image=False,white_neg_x_pick_offset=0.30,
                  white_pos_x_pick_offset=0.10,blue_neg_x_pick_offset=0.20,blue_pos_x_pick_offset=0.0):
        
        if Dof_6d:
            center_point = self.cam_6d_infer(color_image=color_image, depth_image=depth_image)
        else:
            center_point = self.cam_3d_infer(color_image=color_image, depth_image=depth_image,
                              object_name=object_name, save_image=save_image, white_neg_x_pick_offset=white_neg_x_pick_offset,
                              white_pos_x_pick_offset=white_pos_x_pick_offset,blue_neg_x_pick_offset=blue_neg_x_pick_offset,
                              blue_pos_x_pick_offset=blue_pos_x_pick_offset)
        return center_point
    

    def cam_6d_infer(self,color_image, depth_image):
        if not hasattr(self, 'detection_count'):
            self.detection_count = 0
        if not hasattr(self, 'no_detection_count'):
            self.no_detection_count = 0
        self.center_point = None
        rgb_image = color_image.copy()
        self.has_detected = False

        self.center_point, self.has_detected = self.icp.get_pose(target_image=rgb_image, target_depth = depth_image)
        print(f"pose: {self.center_point}")

        return self.center_point, color_image

    
    def cam_3d_infer(self, color_image, depth_image,object_name,save_image,white_neg_x_pick_offset,white_pos_x_pick_offset,blue_neg_x_pick_offset,blue_pos_x_pick_offset):
        if not hasattr(self, 'detection_count'):
            self.detection_count = 0
        if not hasattr(self, 'no_detection_count'):
            self.no_detection_count = 0
        self.center_point = None
        rgb_image = color_image.copy()
        #print("Inside CamInfer")
        # Perform detection
        bbox = self.get_detection(rgb_image=rgb_image, object_name=object_name)

        org_bbox = copy.deepcopy(bbox)
        self.has_detected = False
        if len(org_bbox) == 4:
            self.has_detected = True
            org_bbox = [int(coord) for coord in org_bbox]
            cv2.rectangle(rgb_image, (org_bbox[0], org_bbox[1]), (org_bbox[2], org_bbox[3]), (0, 255, 0), 3)
            cv2.imwrite("current_detection.jpg", rgb_image)
            print("object detected!")
            # Process point cloud data
            pcd = self.pointcloud.crop_pcd(
                roi=org_bbox,
                image_data=color_image.copy(),
                depth_data=depth_image.copy(),
                show_flag=False,
            )
            thresh_pcd, center_point = self.pointcloud.filter_point_cloud(pcd, display=False)
            # center_point = self.shift_center_by_offset(center_point,object_name,white_neg_x_pick_offset,white_pos_x_pick_offset,blue_neg_x_pick_offset,blue_pos_x_pick_offset)
            if isinstance(center_point, np.ndarray):
                self.center_point = center_point

                # Update detection count
                """ self.detection_count += 1
                self.no_detection_count = 0

                if self.detection_count >= 2:
                    self.has_detected = True
                    self.detection_count = 2
                    #print("Detection found at: ", getattr(self, "center_point", None))
            else:
                # Reset detection count and increment no-detection count
                self.no_detection_count += 1
                self.detection_count = 0

                if self.no_detection_count >= 100:
                    self.has_detected = False
                    self.no_detection_count = 100 """

            # Visualization
            if self.display:
                full_pcd = self.pointcloud.get_point_cloud(
                    color_frame=color_image.copy(),
                    depth_frame=depth_image.copy(),
                )
                draw_pcd = full_pcd

                self.vis_pcd.points = draw_pcd.points
                self.vis_pcd.colors = draw_pcd.colors
                flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                self.vis_pcd.transform(flip_transform)

                if self.frame_count == 0:
                    self.vis.add_geometry(self.vis_pcd)

                self.vis.update_geometry(self.vis_pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
                self.frame_count += 1

        """ else:
            # Handle case with no valid bbox
            self.no_detection_count += 1
            self.detection_count = 0

            if self.no_detection_count >= 100:
                self.has_detected = False
                self.no_detection_count = 100 """

        print(f"pose: {self.center_point}")

        if save_image:
            self.save_image(color_image)
        return getattr(self, "center_point", None), cv2.cvtColor(rgb_image.copy(), cv2.COLOR_BGR2RGB)


    def shift_center_by_offset(self, center_point, object_name,white_neg_x_pick_offset,white_pos_x_pick_offset,blue_neg_x_pick_offset,blue_pos_x_pick_offset):

        if not isinstance(center_point, np.ndarray):
            return None
        x, y, z = center_point.tolist()
        if x < 0:
            if object_name == "white_connector":
                x += (x*white_neg_x_pick_offset)
            else:
                x += (x*blue_neg_x_pick_offset)
        elif x > 0:
            if object_name == "white_connector":
                x -= (x*white_pos_x_pick_offset)
            else:
                x -= (x*blue_pos_x_pick_offset)
        
        # y += (y*0.3)
        return np.array([x, y, z])



    def save_image(self, image):
        save_idx = len(os.listdir(self.train_path))
        save_path = os.path.join(self.train_path, f"live_capture_{save_idx}.jpg")        
        cv2.imwrite(save_path,  image)
        print(f"Image saved in {save_path}")


    def get_bbox_patch(self,bbox):
        y_diff = bbox[3] - bbox[1]
        x_diff = bbox[2] - bbox[0]
        y_step = y_diff * self.ratio
        x_step = x_diff * self.ratio
        bbox[0] = bbox[0] + x_step
        bbox[2] = bbox[2] - x_step
        bbox[1] = bbox[1] + y_step
        bbox[3] = bbox[3] - y_step
        top_left = (bbox[0], bbox[1])
        top_right = (bbox[2], bbox[1])
        bottom_right = (bbox[2], bbox[3])
        bottom_left = (bbox[0], bbox[3])

        return [top_left, top_right, bottom_right, bottom_left]
    


    def live_infer(self):
        while self.stop_exec:
            color_image, depth_image = self.camreader.next_frame()
            if self.display:
                img_str = ("Press `s` to start inference",
                           "Press `c` to save image for training",
                           "Press 'esc' to exit",)
                rgb_image = cv2.cvtColor(color_image.copy(),cv2.COLOR_RGB2BGR)       
                image_edit= put_texts(rgb_image.copy(),img_str,draw_bg=True,v_space=30,txt_thickness=1,txt_color=(125, 50, 70))   
                cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
                cv2.imshow(self.window_name, image_edit)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("c"):
                    save_idx = len([path for path in os.listdir(self.val_path) if path.endswith(".jpg")])
                    save_path = f"{self.val_path}/{save_idx}.jpg"
                    cv2.imwrite(save_path, rgb_image)
                    #print(f"Image saved in : {save_path}")

                if key == ord("s"):
                    detection = self.detect.infer(data=rgb_image)
                    bbox = detection["bbox"].squeeze()
                    top_left = (bbox[0], bbox[1])
                    top_right = (bbox[2], bbox[1])
                    bottom_right = (bbox[2], bbox[3])
                    bottom_left = (bbox[0], bbox[3])
                    # x,y,w,h = cv2.selectROI("Draw ROI", color_image)
                    # top_left = (x, y)
                    # top_right = (x + w, y)
                    # bottom_right = (x + w, y + h)
                    # bottom_left = (x, y + h)
                    bbox_contour = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.int32).reshape((-1, 1, 2))

                    # result = self.segement.infer(data=color_image, boxes=[roi])
                    
                    # pcd = self.pointcloud.get_point_cloud(color_frame=color_image,depth_frame=depth_image)
                    pcd = self.pointcloud.crop_pcd(roi=bbox_contour, image_data=color_image, depth_data=depth_image,show_flag=True)
                    
                    # Find the median of depths, that's the 'z', then what is x and y.

                    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                    pcd.transform(flip_transform)
                    # value = 1
                    # for scale in range(1,10):
                    #     pcd_copy = copy.deepcopy(pcd)
                    #     pcd_copy.scale(value, center=pcd_copy.get_center())
                    colors = get_random_colors()
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    points = np.asarray(pcd.points)
                    # Take only the points less than 
                    thresholded_points = points[points[:, 2] >= -0.5 ]
                    # result = thresholded_points[thresholded_points[:, 2] <= -0.1]

                    # thresholded_points = points[(points[:, 2] <= 30) & (points[:, 2]>=5)]
                    
                    median = np.median(thresholded_points, axis=0)
                    
                    o3d.visualization.draw_geometries(
                            [pcd],
                            window_name="Visualize",
                        )

                if key in (27,13):
                    self.stop_exec = False



if __name__ == "__main__":
    app = MainApp(object_name="white_connector",display=False)
    app.infer()

# if we are drawing a bbox any way then let's run the sam model as well maybe a smaller one like efficient sam
# 
# "vit_b", "vit_l", "vit_h", "edge_sam", "edge_sam_3x", "sam_finetuned", None]