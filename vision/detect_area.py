##
# @file detect_area.py
#
# @brief Detect the area and crop the ZED image from where the model will recognize blocks
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#

import cv2
import numpy as np
from PIL import Image

from params import *

class DetectArea: 
    """! Class that detect the area in wich detect blocks"""
    def __init__(self, input_img, output_img_path):
        """! Initialization of the class"""
        self.input_img = input_img
        self.output_img_path = output_img_path
    
    def create_mask(self):
        """! Create a mask to keep in view only the area in wich there are the blocks to avoid error on detection"""
        mask = np.zeros(self.input_img.shape[0:2], dtype=np.uint8)
        shown = np.array(TABLE)

        cv2.fillPoly(mask, [shown], 255)
        mask_background = cv2.inRange(mask, 1, 255)
        output = cv2.bitwise_and(self.input_img, self.input_img, mask=mask_background)

        cv2.imwrite(self.output_img_path, output)

if __name__ == '__main__':
    img = cv2.imread('zed_image.png')
    detectArea = DetectArea(input_img = img, output_img_path='detected_area.png')
    detectArea.create_mask()