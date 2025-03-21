import subprocess

def set_start_demo():
    try:
        subprocess.run(["ros2", "param", "set", "/rcar_demo_node", "start_demo", "True"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    set_start_demo()


"""import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import subprocess
import time

def set_start_demo():
    try:
        subprocess.run(["ros2", "param", "set", "/rcar_demo_node", "start_demo", "True"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

def wait_for_param_to_turn_false():
    rclpy.init()
    node = Node("param_monitor")

    # Declare the parameter (ensures it can be accessed)
    node.declare_parameter("start_demo", True)
    
    while rclpy.ok():
        param = node.get_parameter("start_demo")  # Correctly retrieve the parameter
        
        if not param.value:  # If it's False, exit the loop
            print("start_demo is now False. Exiting.")
            node.destroy_node()
            rclpy.shutdown()
            return False  
        
        time.sleep(1.0)  # Wait before checking again

if __name__ == "__main__":
    set_start_demo()
    result = wait_for_param_to_turn_false()
    print("Returned:", result)"""

