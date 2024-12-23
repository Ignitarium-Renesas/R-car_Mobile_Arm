from utils.json_utils import read_json,write_json
# from pose_estimation_pkg.libs.utils.json_utils import read_json,write_json



import json
import numpy as np
import cv2
import pyrealsense2 as rs
import os

from enum import IntEnum

class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5



class CamReader:
    def __init__(self,cam_name = "D405",config_file="cam_settings.json") -> None:
        self.config_path = config_file
        self.cam_name = cam_name
        self.load_json()
        self.camera_intrisic = self.get_camera_intrinsic()
        self.load_cam()
        self.cam_config = self.save_cam_config(cam_name=self.cam_name)


    def load_json(self):
        if os.path.isfile(self.config_path):
            self.config = read_json(self.config_path)
        else:
            self.config = {
                "serial": "",
                "color_format": "RS2_FORMAT_RGB8",
                "color_resolution": [640,480],
                "depth_format": "RS2_FORMAT_Z16",
                "depth_resolution": [640,480],
                "fps": 5,
                "visual_preset": ""
            }
            write_json(jsonpath=self.config_path, jsondata=self.config)
            

    def load_cam(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.config["color_resolution"][0], self.config["color_resolution"][1], 
                             rs.format.rgb8, self.config["fps"])
        config.enable_stream(rs.stream.depth, self.config["depth_resolution"][0], self.config["depth_resolution"][1],
                             rs.format.z16, self.config["fps"])
        self.profile = self.pipeline.start(config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        # Using preset HighAccuracy for recording
        self.depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_scale = self.depth_sensor.get_depth_scale()
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 3  # 3 meter
        self.clipping_distance = self.clipping_distance_in_meters / self.depth_scale


    def next_frame(self):
        # Wait for a frame to get camera intrinsics
        align_to = rs.stream.color
        align = rs.align(align_to)
        frames = self.pipeline.wait_for_frames()
        

        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame().get_data()
        color_frame = aligned_frames.get_color_frame().get_data()
        # color_frame = frames.get_color_frame().get_data()
        # depth_frame = frames.get_depth_frame().get_data()
        color_frame = np.ascontiguousarray(color_frame)
        depth_frame = np.ascontiguousarray(depth_frame)
        return color_frame, depth_frame

      
    def get_camera_intrinsic(self,):
        # Set up the RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.config["color_resolution"][0], self.config["color_resolution"][1], rs.format.rgb8, self.config["fps"])

        # Start streaming
        pipeline.start(config)

        # Wait for a frame to get camera intrinsics
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            #print("Failed to capture color frame")
            pipeline.stop()
            exit(1)

        # Get intrinsics from the color frame
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        #print("Camera Intrinsics:", intrinsics)

        # Extract the intrinsic parameters
        width = intrinsics.width
        height = intrinsics.height
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy

        #print(fx,fy,cx,cy)
        # Create Open3D pinhole camera intrinsic
        # o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        # print("Open3D Intrinsic Matrix:\n", o3d_intrinsic.intrinsic_matrix)

        # Stop streaming
        config.disable_all_streams()
        pipeline.stop()
        return fx,fy,cx,cy
    

    def save_cam_config(self,cam_name,config_path="cam_config.json"):
        cam_config = {}
        if os.path.exists(config_path):
            cam_config = read_json(config_path)
            if cam_name in cam_config.keys():
                return cam_config
            
        config = {}
        config["camera_intrinsic"] = self.camera_intrisic
        config["depth_scale"] = self.depth_scale
        config["clipping_distance_in_meters"] = self.clipping_distance_in_meters
        cam_config[cam_name] = config
        
        write_json(jsonpath=config_path,jsondata=cam_config)
        return cam_config
