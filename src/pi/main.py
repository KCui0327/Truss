# src/pi/perception_main.py
from __future__ import annotations

import argparse
import time
from datetime import datetime

import numpy as np
import requests

def fetch_image(url: str):
    import cv2  # import only when needed
    try:
        response = requests.get(url, timeout=5)
        response.raise_for_status()
        img_array = np.frombuffer(response.content, np.uint8)
        return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"Failed to fetch image from {url}: {e}")
        return None

def save_image(img, prefix: str) -> str:
    import cv2
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.jpg"
    cv2.imwrite(filename, img)
    return filename

def run_inference(model, img, cam_name: str):
    if img is None:
        print(f"[{cam_name}] No image to infer.")
        return

    results = model(img)
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            print(f"{cam_name} image - {label} ({conf:.2f}) at {xyxy}")

    save_image(img, cam_name)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to YOLO weights, e.g. weights.pt")
    parser.add_argument("--left-url", required=True)
    parser.add_argument("--right-url", required=True)
    parser.add_argument("--period", type=float, default=1.0, help="Seconds between inference loops")
    args = parser.parse_args()

    from ultralytics import YOLO  # import only when needed
    model = YOLO(args.model)

    while True:
        img_left = fetch_image(args.left_url)
        img_right = fetch_image(args.right_url)

        run_inference(model, img_left, "LeftCam")
        run_inference(model, img_right, "RightCam")

        time.sleep(args.period)

if __name__ == "__main__":
    main()
