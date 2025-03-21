import subprocess

def set_white_connector():
    try:
        subprocess.run(["ros2", "service", "call", "/release_joint","std_srvs/srv/Trigger"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    set_white_connector()
