##
# @file detect_blocks.py
#
# @brief Detect blocks in the image using YOLOv8 model trained in a custom dataset. There is the block class that rapresent a block with his characteristics
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#

from PIL import Image
from ultralytics import YOLO

from detect_area import DetectArea
from params import *
import pandas as pd

import cv2

class Block:
    """! Class that rapresent a block"""
    def __init__(self, category, category_id, confidence, xyxy, zed_img_cropped):
        self.category = category
        self.category_id = category_id
        self.confidence = confidence
        self.xyxy = xyxy
        
        self.image = Image.fromarray(zed_img_cropped)

        self.points_count = 0
        self.points = []


class DetectBlocks:
    """! Class that detect blocks"""
    def __init__(self, zed_img) -> None:
        """! Initializing the model, detect the area and create a mask to improve the recognition of the blocks
        @param The image in wich there are the block to recognize
        """
        self.model = YOLO('best.pt')
        self.zed_img = zed_img
        DetectArea(zed_img, ZED_IMG_CROPPED_PATH).create_mask()
        self.zed_img_cropped = cv2.imread(ZED_IMG_CROPPED_PATH)
        self.blocks = []

    def find_blocks(self):
        """! Detect blocks in the image using YOLOv8 model, save the result of the detection in the runs folder
        and print the result of the detection.
        """
        self.blocks.clear()
        results = self.model.predict(
            self.zed_img_cropped,
            save = True,
            save_crop = True,
            conf = MIN_LEVEL_CONFIDENCE,
            max_det = CATEGORIES
        )

        for r in results:
            for box in r.boxes:
                self.blocks.append(Block(
                    category = r.names[box.cls.item()],
                    category_id = box.cls,
                    confidence = box.conf,
                    xyxy = box.xyxy[0],
                    zed_img_cropped=self.zed_img_cropped
                ))

        print(f'Found {len(self.blocks)} objects!\n')
        df = pd.DataFrame(columns=['idx', 'Category', 'Category ID', 'Confidence', 'Bounding Box'])
        for i, block in enumerate(self.blocks):
            coord = ["%.2f" % coord for coord in block.xyxy.tolist()]
            df.loc[len(df.index)] = [i, block.category, int(block.category_id.tolist()[0]), block.confidence.tolist()[0], coord]
        
        print(df)
    
        return self.blocks