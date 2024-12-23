#Reference : https://gist.github.com/kittinan/e7ecefddda5616eab2765fdb2affed1b


import socket
import struct
import pickle
import numpy as np
import cv2
from threading import Thread,Lock
from queue import Queue



class Consumer:
    def __init__(self, hostname='127.0.0.1', port=65432,threading=True):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((hostname, port))
        print(f"[INFO] Socket bound to {hostname}:{port}")
        self.socket.listen(1)
        print("[INFO] Socket is now listening for connections...")
        self.conn, self.addr = self.socket.accept()
        print(f"[INFO] Connection established with {self.addr}")
        self.threading = threading
        self.que= Queue(maxsize=1)
        self.stop_que= Queue(maxsize=1)

        self.data = b""
        if self.threading:
            self.thread = Thread(target=self.receive)
            self.thread.daemon = True
            self.thread.start()
    
    def next_frame(self):
        if not self.threading:
            return self.decode_image()
        else:
            return self.que.get()
    
    def receive(self):
        while True and self.threading:
            rgb, depth = self.decode_image()            
            #break condition
            if not self.stop_que.empty():
                break
            if not self.que.empty():
                _ = self.que.get()
            self.que.put((rgb, depth))

    def decode_image(self):
        payload_size = struct.calcsize(">L")
        #print("payload_size: {}".format(payload_size))
        while len(self.data) < payload_size:
            #print("Recv: {}".format(len(self.data)))
            self.data += self.conn.recv(4096)
        #print("Done Recv: {}".format(len(self.data)))
        packed_msg_size = self.data[:payload_size]
        self.data = self.data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        #print("msg_size: {}".format(msg_size))
        while len(self.data) < msg_size:
            self.data += self.conn.recv(4096)
        frame_data = self.data[:msg_size]
        self.data = self.data[msg_size:]  
        frame_dict=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        rgb = frame_dict["rgb"]
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        depth = frame_dict["depth"]
        
        return rgb, depth
            

    def stop(self):
        #print("[INFO] Stopping thread...")
        self.stop_que.put("Stop")
        if self.threading and self.thread.is_alive():
            self.thread.join()
    
    def close(self):
        #print("[INFO] Closing connection...")
        self.stop()
        self.conn.close()
        self.socket.close()


if __name__ == "__main__":
    from detection import Detection
    from utils.utils import select_device

    object_name="white_connector"
    weight_path = f"weights/detection/yolo/{object_name}/yolo_weights.pt"
    device = select_device("auto")

    consumer = Consumer()
    detect = Detection(detection_type="yolo",device=device)
    detect.set_object(object_name=object_name,weight_path=weight_path,path=None)
    try:
        while True:
            data = consumer.next_frame()
            if data is None:
                break

            color_image, depth_image = data
            color_image = cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)  
            detection = detect.infer(data=color_image)
            bbox = detection["bbox"].squeeze()
            if len(bbox) == 4:
                assert len(bbox) == 4, f"bbox must have 4 elements, but got {len(bbox)}: {bbox}"

                bbox = [int(coord) for coord in bbox]
                cv2.rectangle(color_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]),(0,255,0),3)    
            cv2.imshow("detection", color_image)
            key = cv2.waitKey(2) & 0xff
            


    except KeyboardInterrupt:
        print("[INFO] Exiting...")

    finally:
        consumer.close()
        # cv2.destroyAllWindows()
