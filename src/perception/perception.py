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
from logutils import init, get_logger

"""
Global Variables
"""

init()
logger = get_logger("Perception")

DETECTION_MODEL_PATH = "perception/models/object_detection/weights.pt"
ESP32_CAMERA_URL     = "http://192.168.2.39/capture"

# resolves warnings about using a slower image processor
processor = AutoImageProcessor.from_pretrained(
    "depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf",
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
        logger.info("Initializing perception module models")
        # YOLO Model Initialization
        logger.debug("Loading detection model from path: %s", DETECTION_MODEL_PATH)
        try:
            self.detection_model = YOLO(DETECTION_MODEL_PATH)
        except Exception:
            logger.exception("Exception while initializing detection model from %s", DETECTION_MODEL_PATH)
            raise
        if self.detection_model is None:
            logger.error("Failed to initialize ripeness detection model")
            raise ValueError("The Ripeness Detection Model was not initialized properly.")
        # Depth Anything V2 Model Initialization
        logger.debug("Loading depth estimation model")
        try:
            self.depth_model = pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf", image_processor=processor)
        except Exception:
            logger.exception("Exception while initializing depth estimation model")
            raise
        if self.depth_model is None:
            logger.error("Failed to initialize depth estimation model")
            raise ValueError("The Depth Estimation Model was not initialized properly.")
        logger.debug("Connecting to segmentation model client")
        try:
            self.segementation_model = Client("prithivMLmods/SAM3-Demo")
        except Exception:
            logger.exception("Exception while initializing segmentation model client")
            raise
        if self.segementation_model is None:
            logger.error("Failed to initialize segmentation model")
            raise ValueError("The Segmentation Model was not initialized properly.")
        logger.info("Perception models initialized successfully")
    
    # an iteration of the perception module
    def run(self):
        logger.info("Running perception module")
        TARGET_STRAWBERRY = None
        try:
            img = perception.fetch_image(ESP32_CAMERA_URL)
            if img is None:
                logger.warning("Input image could not be read")
            else:
                detections = perception.detect_strawberry(img)
                if detections:
                    # pick a random ripe strawberry as target
                    target_xy = random.choice(detections)
                    t_cx, t_cy, _, _, _ = target_xy

                    stem_coords = perception.get_target_stem_coordinate(img, t_cx, t_cy)
                    t_x, t_y = stem_coords
                    depth_map = perception.get_depth_estimation(img)
                    if depth_map.size == 0:
                        logger.warning("Empty depth map returned; skipping depth query")
                    else:
                        t_depth = perception.get_depth_at_point(depth_map, t_x, t_y)
                        TARGET_STRAWBERRY = TargetStrawberry(
                            x=t_x,
                            y=t_y,
                            depth=t_depth,
                        )

                        perception.world_coords_transform()

                        logger.info(
                            "Target Strawberry Coordinates (x, y, depth): (%.4f, %.4f, %.4f)",
                            TARGET_STRAWBERRY.x,
                            TARGET_STRAWBERRY.y,
                            TARGET_STRAWBERRY.depth,
                        )
                else:
                    logger.info("No detections found in current frame")
        except Exception:
            logger.exception("Unhandled exception in perception main loop iteration")

        return TARGET_STRAWBERRY

    # Fetch a single JPEG from an ESP32 Camera
    def fetch_image(self, url: str) -> np.ndarray:
        logger.debug("Fetching image from URL: %s", url)
        if url == "" or url is None or url.isspace():
            logger.warning("Invalid URL provided for image fetching")
            raise ValueError("Invalid URL provided for image fetching.")

        try:
            response = requests.get(url, timeout=5)
            response.raise_for_status() # raise HTTP error if needed
            img_array = np.frombuffer(response.content, np.uint8)
        except Exception as e:
            logger.exception("Failed to fetch image from %s", url)
            return None
        image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if image is None:
            logger.warning("Fetched bytes could not be decoded into an image from %s", url)
            return None
        logger.info("Fetched image successfully from %s with shape %s", url, image.shape)
        return image

    # Inference the YOLO model on an image
    def detect_strawberry(self, img: np.ndarray) -> list:
        if not isinstance(img, np.ndarray):
            logger.error("Incorrect image input type for detection: %s", type(img))
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            logger.warning("Empty image provided for detection")
            raise ValueError("Empty image provided for detection.")
        logger.debug("Running strawberry detection on image with shape %s", img.shape)

        def save_image(img: np.ndarray) -> str:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"detections/image_{timestamp}.jpg"
            try:
                cv2.imwrite(filename, img)
                logger.info("Saved detection visualization to %s", filename)
            except Exception:
                logger.exception("Failed to save detection visualization to %s", filename)
            return filename

        try:
            detections = self.detection_model(img)
        except Exception:
            logger.exception("Exception during detection model inference")
            return []
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
                logger.debug("Detection: label=%s conf=%.2f bbox=%s center=(%s,%s)", label, conf, xyxy.tolist(), cx, cy)
                
            if r.boxes.cls.numel() > 0: # only save image if there is a strawberry detected
                try:
                    save_image(detections[0].plot())
                except Exception:
                    logger.exception("Failed to save plotted detection image")
            logger.info("Detected %d objects in current frame", len(ret))
                
        return ret

    # Get the target stem coordinate given the center of the target strawberry
    def get_target_stem_coordinate(self, img: np.ndarray, cx: np.int64, cy: np.int64) -> tuple:
        if not isinstance(img, np.ndarray):
            logger.error("Incorrect image input type for stem coordinate extraction: %s", type(img))
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            logger.warning("Empty image provided for stem coordinate extraction")
            raise ValueError("Empty image provided for stem coordinate extraction.")
        if not isinstance(cx, np.int64) or not isinstance(cy, np.int64):
            logger.error("Center coordinates must be np.int64. Received types: %s, %s", type(cx), type(cy))
            raise TypeError("Center coordinates must be integers.")
        if cx < 0 or cy < 0 or cx >= img.shape[1] or cy >= img.shape[0]:
            logger.warning("Center coordinates out of image bounds: (%s, %s)", cx, cy)
            raise ValueError("Center coordinates are out of image bounds.")
        logger.debug("Locating stem closest to target center (%s, %s)", cx, cy)
        
        stem_coords = self.segment_image(img)
        if not stem_coords:
            logger.warning("No stem coordinates found from segmentation")
            raise ValueError("No stem coordinates were found.")
        
        min_dist = float('inf')
        target_stem_coord = (-1, -1)
        for coord in stem_coords:
            # a little offset to approximate the stem base
            dist = np.linalg.norm(np.array([cx, cy+5]) - np.array(coord))
            if dist < min_dist:
                min_dist = dist
                target_stem_coord = coord
        logger.info("Selected target stem coordinate %s with distance %.2f", target_stem_coord, min_dist)
    
        return target_stem_coord

    # HF SAM3 API call for segmenting the stem
    def segment_image(self, img: np.ndarray) -> None:
        if not isinstance(img, np.ndarray):
            logger.error("Incorrect image input type for segmentation: %s", type(img))
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            logger.warning("Empty image provided for segmentation")
            raise ValueError("Empty image provided for segmentation.")
        logger.debug("Running segmentation for stem extraction")

        try:
            cv2.imwrite("/tmp/curr_view.jpg", img)
            if not os.path.exists("/tmp/curr_view.jpg"):
                logger.error("Failed to create temporary image file for segmentation")
                raise FileNotFoundError("Temporary image file was not created successfully.")

            result = self.segementation_model.predict(
                source_img=handle_file("/tmp/curr_view.jpg"),
                text_query="strawberry stem",
                conf_thresh=0.6,
                api_name="/run_image_segmentation"
            )
            logger.debug("Segmentation prediction completed")
        except Exception:
            logger.exception("Exception while running segmentation model")
            # cleanup if file exists
            try:
                if os.path.exists("/tmp/curr_view.jpg"):
                    os.remove("/tmp/curr_view.jpg")
            except Exception:
                logger.exception("Failed to remove temporary segmentation file")
            return []

        try:
            os.remove("/tmp/curr_view.jpg")
        except Exception:
            logger.exception("Failed to remove temporary segmentation file after prediction")
        locs = []
        for annot in result.get('annotations', []):
            stem_img = cv2.imread(annot['image']) # masked as red
            b, g, r = cv2.split(stem_img) # split into color channels

            mask = (r > 150) & (g < 80) & (b < 80) # get red mask

            # get the center of the masked stem region
            ys, xs = np.where(mask)
            center_x = int(np.mean(xs))
            center_y = int(np.mean(ys))
            locs.append((center_x, center_y))
        if not locs:
            logger.warning("Segmentation returned no stem locations")
        else:
            logger.info("Segmentation found %d stem location(s)", len(locs))
        
        return locs
        
    # Inference the Depth Anything V2 model on an image
    def get_depth_estimation(self, img: np.ndarray) -> list:
        if not isinstance(img, np.ndarray):
            logger.error("Incorrect image input type for depth estimation: %s", type(img))
            raise TypeError("Incorrect image input type. Expected np.ndarray.")
        if img is None or img.size == 0:
            logger.warning("Empty image provided for depth estimation")
            raise ValueError("Empty image provided for depth estimation.")
        logger.debug("Running depth estimation on image with shape %s", img.shape)
        
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        pil_img = Image.fromarray(img_rgb)

        try:
            output = self.depth_model(pil_img)
        except Exception:
            logger.exception("Exception during depth estimation inference")
            return np.array([])

        depth_map = np.array(output.get("predicted_depth", []))

        # save depth map as grayscale image for visualization
        depth_vis = np.array(output.get("depth", []))
        try:
            if depth_vis.size:
                cv2.imwrite("depth_map.jpg", depth_vis)
                logger.info("Depth map generated and saved to depth_map.jpg")
            else:
                logger.debug("Depth model returned empty visualization array")
        except Exception:
            logger.exception("Failed to save depth visualization image")

        return depth_map
    
    def get_depth_at_point(self, depth_map: np.ndarray, x: int, y: int) -> float:
        if not isinstance(depth_map, np.ndarray):
            logger.error("Incorrect depth map input type: %s", type(depth_map))
            raise TypeError("Incorrect depth map input type. Expected np.ndarray.")
        if depth_map is None or depth_map.size == 0:
            logger.warning("Empty depth map provided for depth retrieval")
            raise ValueError("Empty depth map provided for depth retrieval.")
        if x < 0 or y < 0 or x >= depth_map.shape[1] or y >= depth_map.shape[0]:
            logger.warning("Depth query coordinates out of bounds: (%s, %s)", x, y)
            raise ValueError("Coordinates are out of depth map bounds.")
        
        kernel_size = 5
        x_min = max(0, x - kernel_size // 2)
        x_max = min(depth_map.shape[1], x + kernel_size // 2 + 1)
        y_min = max(0, y - kernel_size // 2)
        y_max = min(depth_map.shape[0], y + kernel_size // 2 + 1)

        depth_region = depth_map[y_min:y_max, x_min:x_max]
        valid_depths = depth_region[depth_region > 0]
        if valid_depths.size == 0:
            logger.warning("No valid depth values found around point (%s, %s)", x, y)
            raise ValueError("No valid depth values found in the specified region.")

        depth_value = np.median(valid_depths)
        logger.debug("Depth at point (%s, %s): %.4f", x, y, depth_value)
        return depth_value
    
    def world_coords_transform(self) -> None:
        # ESP 32 Camera Intrinsics
        fx, fy = 574.63852299, 574.02980747  # Focal length in pixels along x-axis and y-axis
        cx_intrinsic, cy_intrinsic = 290.49412584, 276.20316581  # Principal point x-coordinate and y-coordinate in pixels

        t_x, t_y, t_depth = TARGET_STRAWBERRY.x, TARGET_STRAWBERRY.y, TARGET_STRAWBERRY.depth
        t_depth *= 1000  # convert m to mm
        logger.debug("Transforming target coordinates from image frame to camera frame")

        x = (t_x - cx_intrinsic) * t_depth / fx
        y = (t_y - cy_intrinsic) * t_depth / fy
        z = t_depth

        TARGET_STRAWBERRY.x = x
        TARGET_STRAWBERRY.y = y
        TARGET_STRAWBERRY.depth = z
        logger.info("Transformed target coordinates to (x=%.2f, y=%.2f, z=%.2f)", x, y, z)
    
    # Determines new target strawberries' coordinate based on new frame
    # algorithm: min. 3D Euclidean Distance
    def nearest_neighbour(self, strawberries: list) -> None:
        if strawberries is None or len(strawberries) == 0:
            logger.warning("No strawberries provided for nearest neighbour calculation")
            raise ValueError("No strawberries provided for nearest neighbour calculation.")
        logger.debug("Running nearest neighbour over %d strawberries", len(strawberries))
        
        min_3d_euclidean_dist = float('inf')
    
        for strawberry in strawberries: 
            cx = strawberry['cx']
            cy = strawberry['cy']
            depth = strawberry['depth']
            euclidean_dist = np.linalg.norm(np.array([cx, cy, depth])-np.array(TARGET_STRAWBERRY))
            if euclidean_dist < min_3d_euclidean_dist:
                min_3d_euclidean_dist = euclidean_dist
                TARGET_STRAWBERRY = TargetStrawberry(cx, cy, depth)
        logger.info("Nearest target updated with min 3D distance %.4f", min_3d_euclidean_dist)
        
        return

if __name__ == "__main__":
    perception = Perception()
    logger.info("Starting perception main loop")
    while True:
        try:
            img = perception.fetch_image(ESP32_CAMERA_URL)
            if img is None:
                logger.warning("Input image could not be read")
            else:
                detections = perception.detect_strawberry(img)
                if detections:
                    # pick a random ripe strawberry as target
                    target_xy = random.choice(detections)
                    t_cx, t_cy, _, _, _ = target_xy

                    stem_coords = perception.get_target_stem_coordinate(img, t_cx, t_cy)
                    t_x, t_y = stem_coords
                    depth_map = perception.get_depth_estimation(img)
                    if depth_map.size == 0:
                        logger.warning("Empty depth map returned; skipping depth query")
                    else:
                        t_depth = perception.get_depth_at_point(depth_map, t_x, t_y)
                        TARGET_STRAWBERRY = TargetStrawberry(
                            x=t_x,
                            y=t_y,
                            depth=t_depth,
                        )

                        perception.word_coords_transform()

                        logger.info(
                            "Target Strawberry Coordinates (x, y, depth): (%.4f, %.4f, %.4f)",
                            TARGET_STRAWBERRY.x,
                            TARGET_STRAWBERRY.y,
                            TARGET_STRAWBERRY.depth,
                        )
                else:
                    logger.info("No detections found in current frame")
        except Exception:
            logger.exception("Unhandled exception in perception main loop iteration")

        time.sleep(10) # 10 seconds