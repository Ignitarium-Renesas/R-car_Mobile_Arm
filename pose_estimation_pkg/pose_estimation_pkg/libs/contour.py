import cv2
import numpy as np
import time  # Import time module to track elapsed time
import os
# from detection import Detection
from datetime import datetime
import torch

from pose_estimation_pkg.libs.utils.utils import get_slope, compute_angle, apply_rotation, get_bounding_rectangle, get_slope_from_cntr
from pose_estimation_pkg.libs.segmentation import Segmentation




class Segment:
    def __init__(self,package_path,display=False):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.display = display
        self.package_path = package_path 
        self.initialize_models()

    def initialize_models(self,):
        self.dirname = os.getcwd()

        self.save_dir = "Data/"
        os.makedirs(self.save_dir, exist_ok=True)

        self.pcd_dir = os.path.join(self.save_dir, "pcd_dir")
        os.makedirs(self.pcd_dir, exist_ok=True)

        self.image_dir = os.path.join(self.save_dir, "img_dir")
        os.makedirs(self.image_dir, exist_ok=True)

        self.segement = Segmentation(package_path=self.package_path,device=self.device)
        # self.segement.set_object()


    def get_rect(self, color_image):

        # while True:
        # color_image = cv2.imread(imgpath)
        color_image = cv2.cvtColor(color_image.copy(),cv2.COLOR_RGB2BGR) 
        # org_image = color_image.copy()

        results = self.get_segment(color_image=color_image)
        image, contours = results[0], results[1]
        # det_image = segmentation[0] if segmentation is not None else color_image
        if self.display:
            cv2.imshow("Capture Image", image)
            key = cv2.waitKey(0) & 0xff

        if len(contours) != 0:
            rect_points = get_bounding_rectangle(contours[0])
            x1, y1, x2,y2 = rect_points
            cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 10)
            cv2.imwrite(f"x_0.jpg", image) 
            # TODO: Later change it for multiple objects
            return rect_points, contours[0]
        
        return None, None    


    def get_segment(self,color_image):
        annotations = self.segement.infer(data=color_image)
        return annotations


    def save_image(self,img_crop,img_seg):
        current_date = datetime.now().date()
        img_save_idx = len(os.listdir(self.image_dir))
        save_path = os.path.join(self.image_dir, f"{current_date}_{img_save_idx}_seg.jpg")
        cv2.imwrite(save_path, img_crop)

        save_path = os.path.join(self.image_dir, f"{current_date}_{img_save_idx}_outline.jpg")
        cv2.imwrite(save_path, img_seg)
        print("[INFO] write image as JPEG")



if __name__ == "__main__":
    app = Segment()
    img1path = "Data/img_dir/2025-01-28_4.jpg"
    img2path = ""

    det_1 = app.infer(imgpath=img1path)
    det_2 = app.infer(imgpath=img2path)

