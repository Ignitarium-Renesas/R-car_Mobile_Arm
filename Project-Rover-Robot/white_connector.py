import subprocess

def set_white_connector():
    try:
        subprocess.run(["ros2", "param", "set", "/camera_pose_srv","class_id", "0"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    set_white_connector()
