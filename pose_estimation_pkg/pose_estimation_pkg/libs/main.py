#from pcd import PointCloud
from pose_estimation_pkg.libs.pcd import PointCloud
from pose_estimation_pkg.libs.utils.utils import select_device


from pose_estimation_pkg.libs.utils.display_utils import put_texts,get_random_colors
from pose_estimation_pkg.libs.detection import Detection
from ament_index_python.packages import get_package_share_directory
from pose_estimation_pkg.libs.utils.json_utils import read_json,write_json


import cv2
import os
import open3d as o3d
import copy
import numpy as np
import random
import time

class MainApp:
    
    def __init__(self,object_name, device="auto",cam_name="D405",inference = False,display=False):
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
        self.center_point = [0.0, 0.0, 0.0]
        self.has_detected = False

        self.device = select_device(device)
        self.cam_config = self.get_cam_config(cam_name=self.cam_name)
        self.set_paths()
        self.load_instances()


    def get_cam_config(self,cam_name="D405",config_path="cam_config.json"):
        config_path = f"{self.package_path}/libs/{config_path}"
        config = read_json(config_path)
        config = config[cam_name]
        return config


    def set_paths(self,):
        if not self.inference:
            self.mesh_dir = f"{self.package_path}/libs/datasets/{self.object_name}/mesh"
            os.makedirs(self.mesh_dir, exist_ok=True)
            self.train_path = f"{self.package_path}/libs/datasets/{self.object_name}/data/train"
            os.makedirs(self.train_path, exist_ok=True)
            self.val_path = f"{self.package_path}/libs/datasets/{self.object_name}/data/val"
            os.makedirs(self.val_path, exist_ok=True)
        

    def load_instances(self):
        #added
        # self.camreader = CamReader()
        #
        self.pointcloud = PointCloud(self.cam_config["camera_intrinsic"],depth_scale=self.cam_config["depth_scale"],
                                     clipping_distance_in_meters=self.cam_config["clipping_distance_in_meters"], display=False)

        mesh_name = [mesh_name for mesh_name in os.listdir(self.mesh_dir) if mesh_name.endswith(".obj")][0]
        mesh_path = os.path.join(self.mesh_dir, mesh_name)
        self.mesh = self.pointcloud.mesh2pcd(mesh_path=mesh_path)

        self.detect = Detection(detection_type="yolo",device=self.device)
        self.detect.set_object(object_name=self.object_name,path=self.package_path)

        #visualisation
        if self.display:
            self.vis_pcd = o3d.geometry.PointCloud()

            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.frame_count = 0


    def cam_infer(self,color_image=None, depth_image=None):
        #added
        # color_image, depth_image = self.camreader.next_frame()
        # color_image = cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)  
        # print(color_image)
        #

        self.has_detected = False
        rgb_image = color_image.copy()
        
        start_time = time.time()
        detection = self.detect.infer(data=rgb_image)
        end_time = time.time() - start_time
        #print("Time for Detection:",end_time)

        bbox = detection["bbox"].squeeze()
        org_bbox = copy.deepcopy(bbox)
        if len(org_bbox) == 4:
            assert len(org_bbox) == 4, f"org_bbox must have 4 elements, but got {len(org_bbox)}: {org_bbox}"
            
            bbox_patch = self.get_bbox_patch(bbox=bbox)
            bbox_contour = np.array(bbox_patch, dtype=np.int32).reshape((-1, 1, 2))

            start_time = time.time()
            full_pcd = self.pointcloud.get_point_cloud(color_frame=color_image.copy(), depth_frame=depth_image.copy())
            pcd = self.pointcloud.crop_pcd(roi=bbox_contour, image_data=color_image.copy(), depth_data=depth_image.copy(),show_flag=False)

            thresh_pcd, center_point = self.pointcloud.filter_point_cloud(copy.deepcopy(pcd),display=False) 
            end_time = time.time() - start_time
            #print("Time for Pose Estimation:",end_time)

            if isinstance(center_point, np.ndarray):
                self.center_point = center_point
                self.has_detected = True

            mesh_translated,xy_z_diff = self.pointcloud.translate_pcd(ref_pcd = copy.deepcopy(self.mesh), object_pcd=thresh_pcd) 

            #bbox display
            org_bbox = [int(coord) for coord in org_bbox]
            cv2.rectangle(rgb_image, (org_bbox[0], org_bbox[1]), (org_bbox[2], org_bbox[3]),(0,255,0),3)    
            if self.display:
                cv2.imshow(self.window_name, rgb_image)
                key = cv2.waitKey(2) & 0xff

                mesh_translated.paint_uniform_color([0, 0, 1])
                full_pcd.scale(15000000, center=xy_z_diff)
                # ref_pcd.scale(1/1000,ref_pcd.get_center())
                draw_pcd = full_pcd + mesh_translated
                    
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
                
        return self.center_point, cv2.cvtColor(rgb_image.copy(),cv2.COLOR_BGR2RGB) 
    

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
                    #print("Centroid point",median)
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