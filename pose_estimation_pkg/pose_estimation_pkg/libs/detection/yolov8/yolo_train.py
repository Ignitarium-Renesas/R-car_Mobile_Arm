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
from .yolo_conversion import Labelme2YOLO
from ultralytics import YOLO
import argparse
import shutil
import torch
import os


class YOLOTraining:
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

    def __init__(self, device, yolo_model):
        """
        Initialize the YOLOInference object.
        """
        self.device = device
        self.yolo_model = yolo_model

    def set_object(self, object_name):
        self.object_name = object_name
        self.weight_path = f"weights/detection/yolo/{object_name}/yolo_weights.pt"
        self.data_path = f"dataset/{object_name}/data.yaml"
        print("[INFO] creating yolo format dataset")
        # convert train_data
        train_converter = Labelme2YOLO(
            json_dir=f"dataset/{object_name}/train/",
            output_dir=f"dataset/{object_name}/yolo_train/",
        )
        train_converter.convert()
        # convert val_data
        val_converter = Labelme2YOLO(
            json_dir=f"dataset/{object_name}/val/",
            output_dir=f"dataset/{object_name}/yolo_val/",
        )
        val_converter.convert()
        # convert test_data
        test_converter = Labelme2YOLO(
            json_dir=f"dataset/{object_name}/test/",
            output_dir=f"dataset/{object_name}/yolo_test/",
        )
        test_converter.convert()
        test_converter.save_dataset_yaml(object_name=self.object_name)

    def train(self, epochs):
        if os.path.exists("runs"):
            shutil.rmtree("runs")
        self.model = YOLO(f"{self.yolo_model}.pt")
        results = self.model.train(
            data=self.data_path,
            epochs=epochs,
            batch=4,
            project=os.path.join("runs", self.object_name),
            patience=100,
            imgsz=640,
        )
        # moving weights
        os.makedirs(f"weights/detection/yolo/{self.object_name}", exist_ok=True)
        shutil.move(
            f"runs/{self.object_name}/train/weights/best.pt",
            f"weights/detection/yolo/{self.object_name}/yolo_weights.pt",
        )
        return results


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
        "-e", "--epochs", default=100, type=int, help="Number of epochs to train"
    )
    arguments = parser.parse_args()
    return arguments


if __name__ == "__main__":
    args = get_arguments()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    yolo_obj = YOLOTraining(device=device)
    yolo_obj.set_object(
        object_name=args.object_name,
    )
    yolo_obj.train(epochs=args.epochs)
