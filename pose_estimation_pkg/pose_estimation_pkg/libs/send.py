from realsense import CamReader  


import cv2
import socket
import struct
import pickle
import time
from threading import Thread
from queue import Queue


class Producer:
    def __init__(self, hostname='127.0.0.1', port=65432,threading=True) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.img_counter = 0
        self.threading = True
        self.que = Queue(maxsize=1)
        self.stop_que = Queue(maxsize=1)
        self.camreader = CamReader()

        
        if self.threading:
            self.thread = Thread(target=self._send)
            self.thread.daemon = True
            self.thread.start()

    def _send(self):
        start_time = time.time()
        x = 1 # displays the frame rate every 1 second
        counter = 0
        while True:
            color_frame, depth_frame = self.camreader.next_frame()
            counter += 1         
            if (time.time() - start_time) > x :
                print("Read FPS: ", counter / (time.time() - start_time))
                counter = 0
                start_time = time.time()
            #break condition
            if not self.stop_que.empty():
                break
            if not self.que.empty():
                _ = self.que.get()
            self.que.put((color_frame, depth_frame))
            # time.sleep(0.1)
            
    def send(self,):
        color_frame, depth_frame = self.que.get()
        color_frame = cv2.cvtColor(color_frame,cv2.COLOR_RGB2BGR)       

        # Compress RGB frame
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, color_frame_encoded = cv2.imencode('.jpg', color_frame, encode_param)

        # Prepare data as a dictionary
        data = {
            "rgb": color_frame_encoded,
            "depth": depth_frame,  # Send depth as raw bytes
            "rgb_shape": color_frame.shape,  # Store shape for decoding
            "depth_shape": depth_frame.shape  # Store shape for decoding
        }

        # Serialize data using pickle
        # serialized_data = pickle.dumps(data, protocol=pickle.HIGHEST_PROTOCOL)
        data = pickle.dumps(data, 0)
        size = len(data)
        # size = len(serialized_data)

        # Send the size and data
        self.socket.sendall(struct.pack(">L", size) + data)
        self.img_counter += 1
        print(f"[INFO] Sent frame {self.img_counter} with size: {size} bytes")

    def stop(self):
        print("[INFO] Stopping thread...")
        self.stop_que.put("Stop")
        if self.threading and self.thread.is_alive():
            self.thread.join()

    def close(self):
        print("[INFO] Closing socket...")
        self.stop()
        self.socket.shutdown(socket.SHUT_RDWR)
        self.socket.close()


if __name__ == "__main__":
    producer = Producer()
    try:
        while True:
            # Fetch RGB and depth frames
            #skip_frame_count = 20
            #current_count = 0
            #while current_count < skip_frame_count:
                #color_frame, depth_frame = camreader.next_frame()
                #current_count += 1

            # color_frame, depth_frame = camreader.next_frame()
            

            # Send both frames together
            start_time = time.time()
            producer.send()
            print("[INFO] RGB and Depth data sent.")
            end_time = time.time() - start_time
            print("send time:", end_time)

    except KeyboardInterrupt:
        print("[INFO] Stopping producer...")
    finally:
        producer.close()


