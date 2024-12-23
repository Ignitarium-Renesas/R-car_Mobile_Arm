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
#  Filename    : yolo_infer.py
# --------------------------------------------------------------------------
#  Description : 2D Detection inference
# --------------------------------------------------------------------------

"""inference code for object detection"""

# imports
from ultralytics import YOLO
import argparse
import torch
import cv2
import os


class YOLOInference:
    """
    YOLOInference class performs YOLOv8 object detection on input data.

    This class takes care of loading the model, processing input data,
    and generating detection results.

    Attributes:
        model_path (str): The path to the YOLOv8 model weights file.
        device (torch object): The hardware device (CPU/CUDA) for running inference.

    Methods:
        infer: Perform inference on given data and check validity for box size
    """

    def __init__(self, device):
        """
        Initialize the YOLOInference object.
        """
        self.device = device

    def set_object(self, object_name, root_path, weight_path=None):
        if weight_path is None:
            weight_path = f"{root_path}/libs/weights/detection/yolo/{object_name}/yolo_weights.pt"
        assert os.path.exists(weight_path), (
            f"Weights are not available for {object_name}"
            + " Follow README.md/Yolo training steps to train and try again"
        )
        self.model = YOLO(weight_path)  # load model and weights for given model_path
        print("[INFO] Yolo weights loaded successfully")

    def infer(self, data):
        """
        Perform object detection on input image
        Attributes:
            data (numpy): Input image in numpy format.
        Returns:
            boxes (array): Bounding boxes array in xyxy format.
        """
        annotations = {}
        results = self.model.predict(
            source=data, iou=0.75, conf=0.5, device=self.device
        )  # check Yolov8 documentation for more relevent parameters
        ##
        # for result in results:
        #     result.show()
        boxes = results[0].boxes.xyxy.cpu().numpy()  # (x1, y1, x2, y2)
        annotations["bbox"] = boxes
        annotations["bbox_conf"] = results[0].boxes.conf
        return annotations


def get_arguments():
    """
    Take user arguments for customized option

    Returns:
        args: object of argparse class after collecting arguments
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-n",
        "--object_name",
        type=str,
        default="cap_screw",
        help="Name of object for detection",
    )
    parser.add_argument(
        "-i", "--test_image", type=str, required=True, help="Input image path"
    )
    arguments = parser.parse_args()
    return arguments


if __name__ == "__main__":
    args = get_arguments()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    yolo_obj = YOLOInference(device=device)
    yolo_obj.set_object(
        object_name=args.object_name,
        weight_path=f"../../weights/detection/yolo/{args.object_name}/yolo_weights.pt",
    )
    image = cv2.imread(args.test_image)
    assert image is not None, "Could not read image. Please check path again"
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    annotations = yolo_obj.infer(data=image)

    # saving output
    for box in annotations["bbox"]:
        start_point = (int(box[0]), int(box[1]))
        end_point = (int(box[2]), int(box[3]))
        color = (0, 255, 0)  # Green color for bounding boxes
        thickness = 2
        image = cv2.rectangle(image, start_point, end_point, color, thickness)

    image_name = os.path.splitext(os.path.basename(args.test_image))[0]
    save_path = f"../../outputs/{args.object_name}/{image_name}"
    folder_name = os.path.dirname(save_path)
    image_name = f"{image_name}_detect.png"
    os.makedirs(folder_name, exist_ok=True)
    save_path = os.path.join(folder_name, image_name)
    cv2.imwrite(save_path, cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    print(annotations["bbox"])
