import cv2
from huggingface_hub import hf_hub_download
import requests
import torch
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from datetime import datetime
from ultralytics import YOLO
from transformers import pipeline, AutoImageProcessor
from PIL import Image

"""
Global Variables
"""

DETECTION_MODEL_PATH = "models/object_detection/weights.pt"
ESP32_CAMERA_URL     = ""

# resolves warnings about using a slower image processor
processor = AutoImageProcessor.from_pretrained(
    "depth-anything/Depth-Anything-V2-Base-hf",
    use_fast=True
)

TARGET_STRAWBERRY = None

"""
Perception Module
Handles object detection and depth estimation using pre-trained and transfer learned models.
"""
class Perception:
    
    def __init__(self):
        # YOLO Model Initialization
        self.detection_model = YOLO(DETECTION_MODEL_PATH)
        if self.detection_model is None:
            raise ValueError("The Ripeness Detection Model was not initialized properly.")
        # Depth Anything V2 Model Initialization
        self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Base-hf", image_processor=processor)
        if self.depth_model is None:
            raise ValueError("The Depth Estimation Model was not initialized properly.")
    
    # Fetch a single JPEG from an ESP32 Camera
    def fetch_image(self, url: str) -> np.ndarray:
        if url == "" or url is None or url.isspace():
            raise ValueError("Invalid URL provided for image fetching.")

        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status() # raise HTTP error if needed
            img_array = np.frombuffer(response.content, np.uint8)
        except Exception as e:
            print(f"Failed to fetch image from {url}: {e}")
            return None
    
        return cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    # Inference the YOLO model on an image
    def detect_strawberry(self, img: np.ndarray) -> list:
        if not isinstance(img, np.ndarray):
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            raise ValueError("Empty image provided for detection.")
        
        def save_image(img: np.ndarray) -> str:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}.jpg"
            cv2.imwrite(filename, img)
            return filename
        
        results = self.detection_model(img)
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.detection_model.names[cls_id]
                conf  = float(box.conf[0])
                xyxy  = box.xyxy[0].cpu().numpy().astype(int)
                print(f"image - {label} ({conf:.2f}) at {xyxy}")
        save_image(img)
        return results
    
    # Inference the Depth Anything V2 model on an image
    def get_depth_estimation(self, img: str, strawberry_detections: list) -> list:
        output = self.depth_model(img)
        depth_map = np.array(output["depth"])

        ret = []

        for r in strawberry_detections:
            for box in r.boxes:
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                depth_value = depth_map[center_y, center_x]
                ret.append({
                    "cx": center_x,
                    "cy": center_y,
                    "depth": depth_value
                })

        return ret

    # Determines new target strawberries' coordinate based on new frame
    # algorithm: min. 3D Euclidean Distance
    def nearest_neighbour(self, strawberries: list) -> list:
        if TARGET_STRAWBERRY is None:
            raise ValueError("No target strawberry was chosen.")
        if type(TARGET_STRAWBERRY) != list:
            raise TypeError("The strawberries must be a list of strawberries")
        if len(TARGET_STRAWBERRY) != 3:
            raise ValueError("Invalid target position coordinates")
        
        min_3d_euclidean_dist = float('inf')
        ret = [-1, -1, -1]
    
        for cx, cy, depth in strawberries:
            euclidean_dist = np.linalg.norm(np.array([cx, cy, depth])-np.array(TARGET_STRAWBERRY))
            if euclidean_dist < min_3d_euclidean_dist:
                min_3d_euclidean_dist = euclidean_dist
                ret = [cx, cy, depth]
        
        return ret

    # Plots strawberry plant and ripe strawberries
    def plot_strawberry_plant(self, img: str, detections: list) -> None:
        _, ax = plt.subplots()
        ax.imshow(img)

        for d in detections:
            for box in d.boxes:
                xywh = box.xywh
                cx, cy, w, h = xywh[0].cpu().numpy()
                x = cx - w / 2
                y = cy - h / 2

                rect = patches.Rectangle((x, y), w, h, linewidth=1, facecolor='none')
                ax.add_patch(rect)

        plt.show()
    
if __name__ == "__main__":
    perception = Perception()
    img = perception.fetch_image(ESP32_CAMERA_URL)

    if img is not None:
        detection_results = perception.detect_strawberry(img)
        results = perception.get_depth_estimation(img, detection_results)
        print("Depth estimation completed.")

        for res in results:
            print(f"Detection at x: {res['cx']}, y: {res['cy']} has depth: {res['depth']}")
        
        if not TARGET_STRAWBERRY and results:
            cx, cy, depth = random.choice(results)
            TARGET_STRAWBERRY = (cx, cy, depth)
        elif results:
            TARGET_STRAWBERRY = tuple(perception.nearest_neighbour(results))
        else:
            raise ValueError("No strawberries detected!")
