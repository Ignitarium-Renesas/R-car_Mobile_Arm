# Mobile Pick and Place
![RCAR Images](./images/rcar.png)
## Project Overview
The **Mobile Pick and Place** project involves a robotic arm mounted on an Autonomous Mobile Robot (AMR). The system performs pick-and-place operations autonomously by navigating to a target location, detecting and estimating the pose of objects using a YOLO-based object detection module, and executing robotic arm movements. The navigation stack and Robotics arm software is implemented using the ROS 2 Humble framework.
 
### System Workflow
1. **Navigation ( roverrobotics_ros2 )** : The AMR navigates from its current location to the designated pick and drop location using the Nav2 stack.
2. **Object Detection and 6D Pose Estimation( pose_estimation_pkg )**: A YOLO-based AI module identifies the object and uses it for estimating the 6D pose of the object relative to the camera by utilizing Hyco compiler.
3. **Pose Transformation**: The `rcar_communication` package transforms the pose from the camera frame to the base frame of the robotic arm.
4. **Pick-and-Place Operation**: Joint angles are calculated, and the robotic arm executes the pick-and-place task.
 
---
## Setup Instructions



## Prerequisites
- ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) installed on your system.
- Correct hardware connections for the AMR and robotic arm.
- A YOLO v8-based object detection model.
- Install [Node.js](https://nodejs.org/en/download/) for GUI.

## Steps to Set Up the Project

### 1. Create a ROS 2 Workspace and Clone Repository
```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/Ignitarium-Renesas/R-car_Mobile_Arm.git
```

### 2. Install Requirements
```sh
cd ~/ros2_ws
pip install -r src/pose_estimation_pkg/requirements.txt 
pip install pymycobot --upgrade 

cd ~/ros2_ws/Rcar_board_2/Project-Rover-Robot
npm i 
```

### 3. Build the Workspace
```sh
cd ~/ros2_ws
colcon build
```

## Run Application

### Step 1: Verify Connections
Ensure all hardware connections are correctly set up.

- Turn on the Wi-Fi Router.
- Turn on the robotic arm & MyAGV mobile base.

### Step 2: Connect to the MyAGV and MechArm Robotic Arm Controller

Connect to the Wi-Fi access point:
```sh
SSID: Robotics_5G  
Pass: IGN_Robo  
```
Log in to the MyAGV Mobile Base using SSH:
```sh
ssh er@192.168.0.222
# Password: Elephant
```

### Step 3: Launch the RealSense Camera Node
Open a terminal in the MyAGV system and launch the camera node:
```sh
ros2 launch realsense2_camera rs_launch.py
```

### Step 4: Run the Demo Application Node
Open another terminal on your laptop and run:
```sh
ros2 run rcar_demo task_node
```
For 6Dof, open a terminal and run
```bash
ros2 param set /camera_pose_srv 6dof True
```

### Step 5: Launch the GUI
Navigate to the `Project-Rover-Robot` folder and run:
```sh
node index.js
```
Open a browser and enter the following in the search bar:
```sh
http://localhost:5000/
```
Select the Connector and then click on the **Start Demo** button to start the Navigation and Picking.

