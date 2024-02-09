##
# @file vision.py
#
# @brief Detect legos from a photos caming from the ZED camera and find position of them.
#
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#

import rospy
import cv2
import time
from cv_bridge import CvBridge
from math import dist
from math import pi
from math import atan
from tf.transformations import quaternion_from_euler

import sensor_msgs.point_cloud2 as pcloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from planner_pkg.msg import legoGroup
from planner_pkg.msg import legoDetection
from geometry_msgs.msg import Pose

from params import *
from detect_legos import DetectLegos
from detect_legos import Lego

##\cond 
message_list = []
##\endcond

##\cond 
isMoving = True
##\endcond

def find_orientation(y_max_point, x_min_point, y_min_point):
    """! Find the orientation of a lego in Euler angles.
    @param y_max_point list The coordinates of the point of the lego with the biggest y 
    @param x_min_point list The coordinates of the point of the lego with the smallest x
    @param y_min_point list The coordinates of the point of the lego with the smallest y

    @return An array with the orientation expressed in Euler angles.
    """

    rotation = [0, 0, 0]
    if (x_min_point[0] - y_max_point[0]) != 0:
        alpha =  atan((y_max_point[1] - x_min_point[1]) / (y_max_point[0] - x_min_point[0])) 
    else:
        alpha = 0

    d12 = dist(y_max_point, x_min_point)
    d23 = dist(x_min_point, y_min_point)
    
    if(d12 > d23):
        alpha = alpha + (pi / 2)
    
    rotation[2] = alpha

    return rotation

def compute_positions(actual_detection, legos): 
    """! Find three useful points of a lego from all the points contained in it, then find orientation,
    convert it to quaternions and in the end create the Pose object

    @param actual_detection number The index of a lego
    @param legos list The list containing all the detected legos

    @return The pose of a lego
    """
    y_max_point = [0,0,0]   
    x_min_point = [0,0,0]   
    y_min_point = [0,0,0]   

    biggest_y = 0
    smallest_y = 100000
    smallest_x = 100000

    for point in legos[actual_detection].points:
        if(point[2] > LEGO_COORD_Z):  
            if(point[1] > biggest_y):
                biggest_y = point[1]
                y_max_point = np.copy(point)
            if(point[1] < smallest_y):
                smallest_y = point[1]
                y_min_point = np.copy(point)
            if(point[0] < smallest_x):
                smallest_x = point[0]
                x_min_point = np.copy(point)
    
    position = [(y_max_point[0] + y_min_point[0]) / 2,
                (y_max_point[1] + y_min_point[1]) / 2]

    rotation = find_orientation(y_max_point, x_min_point, y_min_point)
    print()
    print(f'Category: {legos[actual_detection].category}')
    print(f'Position: {["%.2f" % coord for coord in position]}')
    print(f'Rotation: {["%.2f" % angle for angle in rotation]}')

    initial_pose = Pose()
    initial_pose.position.x = position[0]
    initial_pose.position.y = position[1]
    initial_pose.position.z = LEGO_COORD_Z

    quaternions = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    
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
    """! This function wait a message from a pointcloud and then reads the points from it and compute
    the position and the orientation of the legos in the Gazebo scenario. In the end it populate the
    list with all messages for Ros 
    """
    
    zed_image = cv2.imread(ZED_IMG_PATH)
    while True:
        try:
            legos = DetectLegos(zed_img=zed_image).find_legos()
            break
        except AttributeError:
            print('Error on getting image... Retrying')

    msg = rospy.wait_for_message(POINTCLOUD_SUB_TOPIC, PointCloud2)

    points=[]

    for lego in legos:
        cont = 0
        for j in range(int(lego.xyxy[1]), int(lego.xyxy[3])):
            for i in range(int(lego.xyxy[0]), int(lego.xyxy[2])):
                points.append((i, j))
                cont += 1 
        lego.points_count = (cont)

    points_list = []
    for data in pcloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=points):
        points_list.append([data[0], data[1], data[2]])
    
    cont = 1
    actual_detection = 0
    
    for data in points_list:
        data_world = RY.dot(data) + ZED_POSITION + BASE_LINK_POSITION
        data_world = np.array(data_world)

        if(actual_detection < len(legos) - 1): 
            legos[actual_detection].points.append(data_world[0])
            if(cont >= legos[actual_detection].points_count):
                initial_pose = compute_positions(actual_detection, legos)

                message_list.append(legoDetection(legos[actual_detection].category, initial_pose))
                
                actual_detection += 1
                cont = 1
            
            cont += 1
    return len(legos)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    pos_pub = rospy.Publisher(PUB_TOPIC, legoGroup, queue_size = 11)

    print('************************')
    print('Vision module ready')
    print('************************')
    #movement_sub = rospy.Subscriber('/ur5/joint_states', Float64MultiArray, callback=lambda data: isMoving=len(data))
    #while(not isMoving or num_lego != 0):
    image_sub = rospy.Subscriber(IMAGE_SUB_TOPIC, Image, callback=receive_image, queue_size = 1)
    loop_rate = rospy.Rate(1.)
    print('Waiting for the image...')
    while True:
        loop_rate.sleep()
        break

    print('Image received')

    num_lego = pointCloudCallBack()
    
    pos_pub.publish(legoGroup('Assignement', message_list))

        #time.sleep(15)


    
    print()
    print('************************')
    print('Terminate vision module')
    print('************************')
