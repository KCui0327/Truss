"""

"""

import cv2

class Estimation:
    def __init__(self):
        pass

    def estimate(self, left_image, right_image):
        stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(left_image, right_image)

        # Display the disparity map
        cv2.imshow('Disparity Map', disparity)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        