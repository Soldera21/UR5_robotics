##
# @file params.py
#
# @brief Parameters used in the vision scripts
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#

import os
import numpy as np

# Global Constants
## Area where the vision detect blocks
TABLE = [[825,549], [1301,552], [1570,913], [658, 921]]

## Level of confidence of the Yolo model to keep the assigned labels
MIN_LEVEL_CONFIDENCE = 0.3

## ROS nodes name
NODE_NAME = 'vision'

## ROS topic where to publish positions
PUB_TOPIC = 'lego_position'

## ROS topic from where the script get the pointcloud
POINTCLOUD_SUB_TOPIC = '/ur5/zed_node/point_cloud/cloud_registered'

## ROS topic from where the script get the ZED image
IMAGE_SUB_TOPIC = '/ur5/zed_node/left/image_rect_color'

## Path where the cropped image is saved (a mask is applied to the photo to reduce confusion) 
ZED_IMG_CROPPED_PATH = os.getcwd() + '/cropped_zed_image.png'

## Path where the original ZED image is saved
ZED_IMG_PATH = os.getcwd() + '/zed_image.png'

## Number of categories of blocks
CATEGORIES = 11

## Rotation matrix of the ZED camera
RY = np.matrix([[ 0.     , -0.49948,  0.86632],
                [-1.     ,  0.     ,  0.     ],
                [-0.     , -0.86632, -0.49948]])

## Zed position regarding to the base link frame
ZED_POSITION = np.array([-0.9 ,0.24 ,-0.35])

## Base link position regarding to the origin frame
BASE_LINK_POSITION = np.array([0.5,0.35,1.75])

## Height of the block regarding to the origin frame
BLOCK_COORD_Z = 0.875