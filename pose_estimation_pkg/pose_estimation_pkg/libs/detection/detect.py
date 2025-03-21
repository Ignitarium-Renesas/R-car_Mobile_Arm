# -*- coding: utf-8 -*-
# --------------------------------------------------------------------------
#                         Copyright © by
#           Ignitarium Technology Solutions Pvt. Ltd.
#                         All rights reserved.
#  This file contains confidential information that is proprietary to
#  Ignitarium Technology Solutions Pvt. Ltd. Distribution,
#  disclosure or reproduction of this file in part or whole is strictly
#  prohibited without prior written consent from Ignitarium.
# --------------------------------------------------------------------------
#  Filename    : detect.py
# --------------------------------------------------------------------------
#  Description : 2D Detection inference
# --------------------------------------------------------------------------

"""inference code for object detection"""

# imports
from pose_estimation_pkg.libs.utils.utils import ansi_colors
# from utils.utils import ansi_colors



class Detection:
    """
    Detection class performs 2D object detection with given model type

    This class takes care of creating instance of given model type and calling
    inference with given data.

    Attributes:
        detection_type (str): The type of model to use for detection
        detection_params (dict): detection related params.

    Methods:
        infer: Call inference with choosen model type and pass given data.
    """

    def __init__(self, detection_type, device):
        """
        Initialize the Detection object
        """
        self.detection_type = detection_type
        self.device = device
        self.colors = ansi_colors()

    def set_object(self, object_name,weight_path):
        self.object_name = object_name
        if self.detection_type == "yolo":
            from .yolov8.yolo_infer import YOLOInference

            self.model = YOLOInference(device=self.device)
        else:
            raise ValueError(
                self.colors["RED"]
                + f"{self.detection_type} is not a valid model type for object detection."
                + " Check help (-h) for more details"
                + self.colors["RESET"]
            )
        self.model.set_object(object_name=object_name,weight_path=weight_path)

    def train(self):
        self.model.train()

    def infer(self, data):
        """
        Call object detection inference with selected model.
        Attributes:
            data (numpy): Input image in numpy format.
        Returns:
            results (array): Bounding boxes array in xyxy format.
        """
        results = self.model.infer(data=data)
        return results
