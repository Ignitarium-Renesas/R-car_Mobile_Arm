# ROS2 Package: rcar_communication

The `rcar_communication` package facilitates communication and control between the Renesas R-Car board, a robotic arm, and a mobile platform. It is designed for a demonstration setup to showcase an autonomous mobile pick-and-place operation using ROS 2.

## Features
- Communication between the R-Car board, robotic arm, and mobile platform.
- Execution of pick-and-place tasks autonomously.
- Support for search and pick operations using ROS 2 services.

## Prerequisites
- ROS 2 Humble or later.
- Network access to the robotic arm controller and mobile base.
- Pre-configured R-Car board with Docker and ROS 2 workspace (`rcar_ws`).

## Building the Package
1. Clone the repository:
   ```bash
   git clone git@gitlab.ignitarium.in:ign-ai/customer/renesas/rcar/r_car_board.git
   git clone git@gitlab.ignitarium.in:ign-ai/customer/renesas/rcar/myagv.git
   git clone git@gitlab.ignitarium.in:ign-ai/customer/renesas/rcar/pose_estimation_pkg.git
   
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Demo Setup and Execution

### Step 1: Bring Up the Robotic Arm Controller
1. Log in to the robotic arm controller via SSH:
   ```bash
   ssh er@192.168.0.225
   ```
   - Password: `Elephant`

2. Start the server:
   ```bash
   ./start_server.sh
   ```

### Step 2: Bring Up the Mobile Base
1. Log in to the mobile base via SSH:
   ```bash
   ssh er@192.168.0.222
   ```
   - Password: `Elephant`

2. Source the ROS 2 workspace:
   ```bash
   source colcon_ws/install/setup.bash
   ```

3. Launch the base and camera setup:
   ```bash
   ros2 launch myagv_odometry base_camera.launch.py
   ```

### Step 3: Bring Up the R-Car Board
1. Log in to the R-Car board via SSH.
2. Start Docker:
   ```bash
   docker start rcar
   docker exec -it rcar bash
   ```

3. Source the ROS 2 workspace:
   ```bash
   source rcar_ws/install/setup.bash
   ```

4. Launch the navigation stack:
   ```bash
   ros2 launch myagv_navigation navigation.launch.py
   ```

### Step 4: Run the Pick-and-Place Demo
1. Open another terminal on the R-Car board.

2. Call the pick-and-place service:
   ```bash
   ros2 service call /run_demo mecharm_interfaces/srv/PickObject {}
   ```

### Step 5: Run the Search-and-Pick Demo
1. Open another terminal on the R-Car board.

2. Call the search-and-pick service:
   ```bash
   ros2 service call /search_demo mecharm_interfaces/srv/PickObject {}
   ```

