import config.config as cfg
import calibrate.calibrate as calibrate

def main():
    config = cfg.Config('config/config.yaml')
    print("Loaded configuration:", config.config)

    # Camera calibration
    print("Starting camera calibration...")
    calibrator = calibrate.Calibrate(config)
    calibrator.calibrate_camera(img_path='calibrate/imgs/*.jpg')
    calibrator.get_calibration_parameters()
    print("Camera matrix:\n", calibrator.camera_matrix)
    print("Distortion coefficients:\n", calibrator.dist_coeffs)
    print("Camera calibration completed.")

    print("Starting depth estimation...")
    while True:
        pass  # Placeholder for depth estimation loop
    


if __name__ == "__main__":
    main()