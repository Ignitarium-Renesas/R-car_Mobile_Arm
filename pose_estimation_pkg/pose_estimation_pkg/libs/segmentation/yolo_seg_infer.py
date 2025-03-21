import os
import cv2
import argparse
from ultralytics import YOLO
import numpy as np 

class Infer:
    def __init__(self, model_path, output_dir):
        self.model = YOLO(model_path, task="segment")
        self.output_dir = output_dir
        self.vis = False
        self.class_colors = {}  # Dictionary to store consistent class colors

    def infer(self, image, save=False, hide_labels=True):
        self.batch = True if isinstance(image, list) and len(image) > 1 else False
        results = self.model.predict(
            source=image, save=save, project=self.output_dir, hide_labels=False, verbose=True, conf=0.5
        )
        return self.draw_segmentation(image=image, results=results)

    def get_class_color(self, class_id):
        """Get consistent class colors for visualization."""
        if class_id not in self.class_colors:
            self.class_colors[class_id] = tuple(np.random.randint(0, 256, 3).tolist())
        return self.class_colors[class_id]

    def draw_segmentation(self, image, results):
        """Draw segmentation masks, contours, bounding boxes, and labels."""
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        contours_list = []
        # idx = 0
        for result in results:
            masks = result.masks  # Segmentation masks
            boxes = result.boxes.xyxy  # Bounding boxes (x1, y1, x2, y2)
            classes = result.boxes.cls  # Class indices
            scores = result.boxes.conf  # Confidence scores
            if masks is not None:
                for mask_pnts, mask, box, cls, score in zip(masks, masks.data, boxes, classes, scores):
                    mask = mask.cpu().numpy()  # Convert to NumPy array
                    mask = (mask * 255).astype(np.uint8)  # Scale from 0-1 to 0-255
                    mask = cv2.resize(mask, (image.shape[1], image.shape[0]))  # Resize to image size

                    # Get class color (consistent per class)
                    color = self.get_class_color(int(cls))

                    # Convert mask to BGR format for proper overlay
                    colored_mask = np.zeros_like(image, dtype=np.uint8)
                    for i in range(3):  
                        colored_mask[:, :, i] = mask * (color[i] / 255.0)

                    # Overlay mask with transparency
                    alpha = 0.5
                    image = cv2.addWeighted(image, 1, colored_mask, alpha, 0)

                    # Convert segmentation points to integer
                    points = np.array(mask_pnts.xy, dtype=np.int32).reshape((-1, 1, 2))
                    contours_list.append(points)

                    # Draw contours on the image
                    cv2.drawContours(image, [points], -1, color, 2)

                    # Draw bounding box
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label text
                    label = f"{self.model.names[int(cls)]}: {score:.2f}"
                    cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # print(label)
                    # cv2.imwrite(f"z_{idx}.jpg", image) 
                    # idx += 1
                    
        return image, contours_list
