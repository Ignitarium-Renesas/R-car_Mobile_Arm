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
#  Filename    : segment.py
# --------------------------------------------------------------------------
#  Description : 2D Segmentation inference
# --------------------------------------------------------------------------

"""inference code for instance segmentation"""

# imports
# from utils import ansi_colors
from pose_estimation_pkg.libs.segmentation.yolo_seg_infer import Infer


class Segmentation:
    """
    Segmentation performs instance segmentation with given model and
    on given data.
    Attributes:
        work_dir (str): The path of working directory (pose_app).
        model_type (str): The model choice for segmentation.
        model_choice (str): The model choice for sam. One of vit_b, vit_h, vit_l.
        device (torch object): The hardware device (CPU/CUDA) for running inference.
    """

    def __init__(self, package_path, device, model_choice="yolo"):
        """
        Initialize the Segmentation object.
        """
        self.model_choice = model_choice
        self.package_path = package_path 
        self.device = device
        self.checkpoint_path = f"{self.package_path}/libs/weights/segmentation/yolo/yolo_seg_weights.pt"
        # self.colors = ansi_colors()
        self.set_model()

    def set_model(self,yolo_type="pt",platform = None):

        self.output_dir = f"results/"
        if self.model_choice == "yolo":
            # self.state = state
            self.model = Infer(model_path=self.checkpoint_path, output_dir=self.output_dir)


    def infer(self, data):
        """
        Call instance segmentation class passing input image and bounding boxes.
        Attributes
            data (numpy): Input image in numpy format.
            boxes (array): Bounding boxes for given image.
        Returns:
            annotations (dict): Dictionary containing contours, area and tight bounding box
        """
        annotations = self.model.infer(image=data)
        return annotations


if __name__ == "__main__":
    import argparse
    import cv2
    from ignutils.json_utils import read_json
    from ignutils.draw_utils import draw_polylines
    import torch

    parser = argparse.ArgumentParser(
        description="SAM Segementation Test"
    )
    parser.add_argument(
        "-img_path",
        "--image_path",
        type=str,
        default=None,
        help="path to image",
    )
    parser.add_argument(
        "-j",
        "--json_path",
        type=str,
        default=None,
        help="path to json with bbox coordinates",
    )
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="display results",
    )
    args = parser.parse_args()
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    image = cv2.imread(args.image_path)
    json_dict = read_json(args.json_path)
    segment = Segmentation(model_choice="vit_b", device=device)
    segment.set_object(object_name="seg_test")
    annotations = segment.infer(image, json_dict["bbox"])
    for annotation in annotations["segmentation"]:
        image = draw_polylines(image=image, points=annotation)
    if args.display:
        cv2.imshow("segmentation result", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()