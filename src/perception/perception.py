import os
import cv2
import time
import random
import requests

import numpy as np

from dataclasses import dataclass
from datetime import datetime
from ultralytics import YOLO
from transformers import pipeline, AutoImageProcessor
from gradio_client import Client, handle_file
from PIL import Image

"""
Global Variables
"""

DETECTION_MODEL_PATH = "models/object_detection/weights.pt"
ESP32_CAMERA_URL     = "http://192.168.2.39/capture"

# resolves warnings about using a slower image processor
processor = AutoImageProcessor.from_pretrained(
    "depth-anything/Depth-Anything-V2-Base-hf",
    use_fast=True
)

@dataclass
class TargetStrawberry:
    x: float
    y: float
    depth: float

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
        self.segementation_model = Client("akhaliq/sam3")
        if self.segementation_model is None:
            raise ValueError("The Segmentation Model was not initialized properly.")
    
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
            filename = f"detections/image_{timestamp}.jpg"
            cv2.imwrite(filename, img)
            return filename
        
        detections = self.detection_model(img)
        ret = []
        for r in detections:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.detection_model.names[cls_id]
                conf  = float(box.conf[0])
                xyxy  = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                ret.append((cx, cy, label, conf, (x1, y1, x2, y2)))
                print(f"image - {label} ({conf:.2f}) at {xyxy}")
                
            if r.boxes.cls.numel() > 0: # only save image if there is a strawberry detected
                save_image(detections[0].plot()) 
                
        return ret

    # Get the target stem coordinate given the center of the target strawberry
    def get_target_stem_coordinate(self, img: np.ndarray, cx: np.int64, cy: np.int64) -> tuple:
        if not isinstance(img, np.ndarray):
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            raise ValueError("Empty image provided for stem coordinate extraction.")
        if not isinstance(cx, np.int64) or not isinstance(cy, np.int64):
            raise TypeError("Center coordinates must be integers.")
        if cx < 0 or cy < 0 or cx >= img.shape[1] or cy >= img.shape[0]:
            raise ValueError("Center coordinates are out of image bounds.")
        
        stem_coords = self.segment_image(img)
        if not stem_coords:
            raise ValueError("No stem coordinates were found.")
        
        min_dist = float('inf')
        target_stem_coord = (-1, -1)
        for coord in stem_coords:
            # a little offset to approximate the stem base
            dist = np.linalg.norm(np.array([cx, cy+5]) - np.array(coord))
            if dist < min_dist:
                min_dist = dist
                target_stem_coord = coord
    
        return target_stem_coord

    # HF SAM3 API call for segmenting the stem
    def segment_image(self, img: np.ndarray) -> None:
        if not isinstance(img, np.ndarray):
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            raise ValueError("Empty image provided for segmentation.")
        
        cv2.imwrite("/tmp/curr_view.jpg", img)
        if not os.path.exists("/tmp/curr_view.jpg"):
            raise FileNotFoundError("Temporary image file was not created successfully.")
        
        result = self.segementation_model.predict(
            image=handle_file("/tmp/curr_view.jpg"),
            text="Strawberry Stem",
            threshold=0.6,
            mask_threshold=0.5,
            api_name="/segment"
        )

        os.remove("/tmp/curr_view.jpg")

        res = result[0]
        locs = []
        for annot in res['annotations']:
            stem_img = cv2.imread(annot['image']) # masked as red
            b, g, r = cv2.split(stem_img) # split into color channels

            mask = (r > 150) & (g < 80) & (b < 80) # get red mask

            # bunch of coordinates where mask is true and just randomly pick one
            ys, xs = np.where(mask)
            coords = list(zip(xs, ys))
            rand_coord = random.choice(coords)
            xs, ys = rand_coord
            locs.append((xs, ys))
        
        return locs
        
    # Inference the Depth Anything V2 model on an image
    def get_depth_estimation(self, img: np.ndarray) -> list:
        if not isinstance(img, np.ndarray):
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            raise ValueError("Empty image provided for depth estimation.")
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        pil_img = Image.fromarray(img_rgb)

        output = self.depth_model(pil_img)
        depth_map = np.array(output["depth"])

        return depth_map
    
    # Determines new target strawberries' coordinate based on new frame
    # algorithm: min. 3D Euclidean Distance
    def nearest_neighbour(self, strawberries: list) -> None:
        if strawberries is None or len(strawberries) == 0:
            raise ValueError("No strawberries provided for nearest neighbour calculation.")
        
        min_3d_euclidean_dist = float('inf')
    
        for strawberry in strawberries: 
            cx = strawberry['cx']
            cy = strawberry['cy']
            depth = strawberry['depth']
            euclidean_dist = np.linalg.norm(np.array([cx, cy, depth])-np.array(TARGET_STRAWBERRY))
            if euclidean_dist < min_3d_euclidean_dist:
                min_3d_euclidean_dist = euclidean_dist
                TARGET_STRAWBERRY = TargetStrawberry(cx, cy, depth)
        
        return

if __name__ == "__main__":
    perception = Perception()
    while True:
        # img = perception.fetch_image(ESP32_CAMERA_URL)
        img = cv2.imread("test_img.jpg", cv2.IMREAD_COLOR)
        
        if img is not None:
            detections = perception.detect_strawberry(img)
            if detections:
                # pick a random ripe strawberry as target
                target_xy = random.choice(detections)
                t_cx, t_cy, _, _, _ = target_xy

                stem_coords = perception.get_target_stem_coordinate(img, t_cx, t_cy)
                t_x, t_y = stem_coords
                depth_map = perception.get_depth_estimation(img)
                t_depth = depth_map[t_y, t_x]
                TARGET_STRAWBERRY = TargetStrawberry(
                    x=t_x,
                    y=t_y,
                    depth=t_depth,
                )

                print(f"Target Strawberry Coordinates (x, y, depth): ({TARGET_STRAWBERRY.x}, {TARGET_STRAWBERRY.y}, {TARGET_STRAWBERRY.depth})")
            
        time.sleep(10) # 10 seconds