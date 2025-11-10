import cv2
from huggingface_hub import hf_hub_download
import requests
import torch
import numpy as np
import matplotlib.pyplot as plt

from datetime import datetime
from ultralytics import YOLO
from transformers import pipeline

DETECTION_MODEL_PATH = "models/object_detection/weights.pt"
ESP32_CAMERA_URL     = ""

"""
Perception Module
Handles object detection and depth estimation using pre-trained and transfer learned models.
"""
class Perception:
    
    def __init__(self):
        # YOLO Model Initialization
        self.detection_model = YOLO(DETECTION_MODEL_PATH)
        # Depth Anything V2 Model Initialization
        self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Base-hf")
    
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
    def get_depth_estimation(self, img):
        depth = self.depth_model.infer_image(img)
        return depth

if __name__ == "__main__":
    perception = Perception()
    img = perception.fetch_image(ESP32_CAMERA_URL)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()

    if img is not None:
        detection_results = perception.detect_strawberry(img)
        depth_map = perception.get_depth_estimation(img)
        plt.imshow(depth_map, cmap='plasma')
        plt.colorbar()
        plt.show()
        print("Depth estimation completed.")
    
