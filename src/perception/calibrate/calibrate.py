"""
This module provides a class for camera calibration using chessboard images.

Developed with reference and inspiration from OpenCV documentation:
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

Credits to:
- OpenCV contributors
"""

import numpy as np
import cv2 as cv
import glob

"""
Calibration Class to handle ESP32 Camera calibration using chessboard images.
"""
class Calibrate:
    def __init__(self, config) -> None:
        print("Initializing Calibrate class")

        self.chessboard_x_size = config.get('chessboard_x_size', 7)
        self.chessboard_y_size = config.get('chessboard_y_size', 7)
        self.criteria = (config.get('criteria'))

        self.img_width = config.get('image_width', 640)

        self.objp = np.zeros((self.chessboard_y_size * self.chessboard_x_size, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_x_size, 0:self.chessboard_y_size].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        self.calibration_images = []

        # Intrinsic and extrinsic camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None 
        self.rvecs = None
        self.tvecs = None
    
    def __resize_image(self, img, width) -> np.ndarray:
        og_width, og_height = img.shape[1], img.shape[0]
        aspect_ratio = width / og_width
        new_height = int(og_height * aspect_ratio)
        resized_img = cv.resize(img, (width, new_height), cv.INTER_LINEAR)
        return resized_img
    
    def calibrate_camera(self, img_path) -> None:
        imgs = glob.glob(img_path)

        if not imgs:
            raise ValueError("No images provided for calibration.")

        print("Starting camera calibration process...")
        for fname in imgs:
            img = cv.imread(fname)
            print(img.shape)
            if img is None:
                print(f"Warning: Could not read image {fname}. Skipping.")
                continue

            img = self.__resize_image(img, self.img_width)
            print(img.shape)
            cv.imshow(fname, img)
            cv.waitKey(0)

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCornersSB(
                image=gray,
                patternSize=(self.chessboard_x_size, self.chessboard_y_size),
                flags=cv.CALIB_CB_EXHAUSTIVE | cv.CALIB_CB_ACCURACY | cv.CALIB_CB_NORMALIZE_IMAGE,
            )

            print(f"Processing {fname}, found corners?: {ret}")

            if ret == True:
                self.objpoints.append(self.objp) # placeholder for 3d points in real world space

                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), self.criteria)
                self.imgpoints.append(corners2) # actual coordinates of corners in 2D

                cv.drawChessboardCorners(img, (self.chessboard_x_size, self.chessboard_y_size), corners2, ret)
                cv.imshow(fname, img)
                cv.waitKey(0)

        cv.destroyAllWindows()
        print("Camera calibration completed.")

        return

    def get_calibration_parameters(self) -> None:
        # rotation vectors -> rotation of chessboard relative to camera
        # translation vectors -> position of chessboard relative to camera

        # map 2D image points to 3D real world points then generate instrinsic and extrinsic parameters
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            (640, 480),
            None,
            None
        )

        if not ret:
            raise ValueError("Camera calibration failed.")
        print("Camera calibration successful.")

        return

    def undistort_image(self, image_path: str) -> np.ndarray:
        if self.camera_matrix is None or self.dist_coeffs is None:
            raise ValueError("Calibration parameters not set. Please run get_calibration_parameters() first.")
        
        img = cv.imread(image_path)
        if img is None:
            raise ValueError(f"Could not read image {image_path}.")
        
        print("Undistorting image...")
        img = self.__resize_image(img, self.img_width)
        
        h, w = img.shape[:2]
        new_camera_mtx, roi = cv.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h), 0.5, (w,h))

        dst = cv.undistort(img, self.camera_matrix, self.dist_coeffs, None, new_camera_mtx)

        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        print("Image undistortion completed.")
        return dst
