##
# @file vision.py
#
# @brief Detect blocks from a photos caming from the ZED camera and find position of them.
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#

import rospy
import cv2
from cv_bridge import CvBridge
from math import atan2

from tf.transformations import quaternion_from_euler
import sensor_msgs.point_cloud2 as pcloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from planner_pkg.msg import legoGroup
from planner_pkg.msg import legoDetection
from geometry_msgs.msg import Pose

from params import *
from detect_blocks import DetectBlocks
from detect_blocks import Block

##\cond 
message_list = []
##\endcond

def find_orientation(y_max_point, x_min_point):
    """! Find the yaw of a block in Euler angles.
    @param y_max_point list The coordinates of the point of the block with the biggest y 
    @param x_min_point list The coordinates of the point of the block with the smallest x

    @return Double The yaw angle of the block
    """
    
    return atan2((y_max_point[1] - x_min_point[1]), (y_max_point[0] - x_min_point[0])) 

def find_center(y_max_point, y_min_point):
    """! Find the coordinates of the center of a block
    @param y_max_point list The coordinates of the point of the block with the biggest y 
    @param y_min_point list The coordinates of the point of the block with the smallest y

    @return List The coordinates of the center
    """
    return [(y_max_point[0] + y_min_point[0]) / 2, (y_max_point[1] + y_min_point[1]) / 2]


def build_pose(block): 
    """! Find three useful points for compute position and orientation of a block
    from all the points contained in it, moreover compute the coordinates of the center,
    find orientation, convert it to quaternions and in the end create the Pose object

    @param block Block The block object whose position and orientation you want to find

    @return The pose of a block
    """
    y_max_point = [0,0,0]   
    x_min_point = [0,0,0]   
    y_min_point = [0,0,0]   

    biggest_y = 0
    smallest_y = 100000
    smallest_x = 100000

    for point in block.points:
        if(point[2] > BLOCK_COORD_Z):  
            if(point[1] > biggest_y):
                biggest_y = point[1]
                y_max_point = np.copy(point)
            if(point[1] < smallest_y):
                smallest_y = point[1]
                y_min_point = np.copy(point)
            if(point[0] < smallest_x):
                smallest_x = point[0]
                x_min_point = np.copy(point)

    center = find_center(y_max_point, y_min_point)
    yaw = find_orientation(y_max_point, x_min_point)
    
    print()
    print(f'Category: {block.category}')
    print(f'Position: {["%.2f" % coord for coord in center]}')
    print('Rotation:' +  "{:10.2f}".format(yaw) + ' yaw' )


    initial_pose = Pose()
    initial_pose.position.x = center[0]
    initial_pose.position.y = center[1]
    initial_pose.position.z = BLOCK_COORD_Z

    quaternions = quaternion_from_euler(0, 0, yaw)
    
    initial_pose.orientation.x = quaternions[0]
    initial_pose.orientation.y = quaternions[1]
    initial_pose.orientation.z = quaternions[2]
    initial_pose.orientation.w = quaternions[3]

    return initial_pose

def receive_image(data):
    """! Recive image from ros and save it"""

    zed_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
    cv2.imwrite(ZED_IMG_PATH, zed_image)

def pointCloudCallBack():
    """! This function waits a message from a pointcloud and then reads the points from it and compute
    the position and the orientation of the blocks in the Gazebo scenario. In the end it populate the
    list with all messages for Ros 
    """
    
    zed_image = cv2.imread(ZED_IMG_PATH)
    while True:
        try:
            blocks = DetectBlocks(zed_img=zed_image).find_blocks()
            break
        except AttributeError:
            print('Error on getting image... Re-run the script')

    msg = rospy.wait_for_message(POINTCLOUD_SUB_TOPIC, PointCloud2)

    points=[]
    for block in blocks:
        block.points_count = 0
        for j in range(int(block.xyxy[1]), int(block.xyxy[3])):
            for i in range(int(block.xyxy[0]), int(block.xyxy[2])):
                points.append((i, j))
                block.points_count +=1
    
    point_counter = 1
    block_index = 0
    for data in pcloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=points):
        data = np.array(RY.dot([data[0], data[1], data[2]]) + ZED_POSITION + BASE_LINK_POSITION)
        if(block_index < len(blocks) - 1): 
            blocks[block_index].points.append(data[0])
            if(point_counter >= blocks[block_index].points_count):
                initial_pose = build_pose(blocks[block_index])
                message_list.append(legoDetection(blocks[block_index].category, initial_pose))
                
                block_index += 1
                point_counter = 1
            
            point_counter += 1
        


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    pos_pub = rospy.Publisher(PUB_TOPIC, legoGroup, queue_size = 11)

    print('************************')
    print('Vision module ready')
    print('************************')
    
    image_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, callback=receive_image, queue_size = 1)
    loop_rate = rospy.Rate(1.)
    print('Waiting for the image...')
    while True:
        loop_rate.sleep()
        break

    print('Image received')

    pointCloudCallBack()
    
    pos_pub.publish(legoGroup('Assignement', message_list))
    
    print()
    print('************************')
    print('Terminate vision module')
    print('************************')
