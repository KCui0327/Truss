import time
import cv2
import numpy as np
import requests
from ultralytics import YOLO
from datetime import datetime

MODEL_PATH = ""
LEFT_ESP32_CAMERA_URL  = ""
RIGHT_ESP32_CAMERA_URL = ""

model = YOLO(MODEL_PATH)

# Fetch a single JPEG from an ESP32 Camera
def fetch_image(url):
    try:
        response = requests.get(url, timeout=5)
        # raise HTTP error if needed
        response.raise_for_status()
        img_array = np.frombuffer(response.content, np.uint8)
        return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"Failed to fetch image from {url}: {e}")
        return None

# Inference the YOLO model on an image
def run_inference(img, cam_name):
    if img is None:
        print(f"[{cam_name}] No image to infer.")
        return
    
    results = model(img)

    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf  = float(box.conf[0])
            xyxy  = box.xyxy[0].cpu().numpy().astype(int)
            print(f"{cam_name} image - {label} ({conf:.2f}) at {xyxy}")
    save_image(img, cam_name)

def save_image(img, prefix):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.jpg"
    cv2.imwrite(filename, img)
    return filename

def main():
    while True:
        img_left  = fetch_image(LEFT_ESP32_CAMERA_URL)
        img_right = fetch_image(RIGHT_ESP32_CAMERA_URL)

        run_inference(img_left,  "LeftCam")
        run_inference(img_right,  "RightCam")

        # Delay before next image
        time.sleep()

if __name__ == "__main__":
    main()
