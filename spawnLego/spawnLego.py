##
# @file spawnLego.py
#
# @brief Spawn lego in random position and orientation
#
# @section module_authors Author(s)
# - Created by Marco Soldera
#


from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
import rospy
import random
import xml.etree.ElementTree as ET
from math import pi
import os

# Global constants

## Path of the models to add to the scene
models_path = os.path.dirname(os.path.abspath(__file__)) + "/models"

## Name of the models
models = ["X1-Y1-Z2", "X1-Y2-Z1", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z2", "X1-Y3-Z2", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2-FILLET", "X2-Y2-Z2", "X1-Y3-Z2-FILLET"] 

## Colors of the generated legos
colorList = ['Gazebo/Indigo', 'Gazebo/Gold', 'Gazebo/Orange','Gazebo/Red', 'Gazebo/Purple', 'Gazebo/Grass','Gazebo/White', 'Gazebo/Green', 'Gazebo/Yellow', 'Gazebo/Blue', 'Gazebo/Turquoise']

## Spacing factor for placing pieces
toll = 0.039

##\cond 
cont = 0
##\endcond 

##\cond 
spawned_lego = []
##\endcond

##\cond 
errors = 0
##\endcond


'''! Spawns the model in the given position
@param model (string): the name of the lego model
@param pos (struct): all the parameters for position and orientation of the lego
@param name (string, optional): the name of the model. Defaults to None.
@param ref_frame (string, optional): the reference frame. Defaults to 'world'

@return string : confirmation of the action
'''
def spawn_model(model, pos, name=None, ref_frame='world'):
	global cont

	if(name == None):
		name = model

	name = model + "_" + str(cont)

	model_xml = models_path + "/" + model + "/model.sdf"
	model_xml = open(model_xml, 'r').read() 
	color = colorList[cont]
	cont += 1

	if color is not None:
		model_xml = changeModelColor(model_xml, color)
	
	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

	return spawn_model_client(model_name=name, model_xml=model_xml, initial_pose=pos, reference_frame=ref_frame)


'''! Removes the model with 'modelName' from the Gazebo scene
@param model (string): name of the model to be deleted

@return bool : True if the deletion succeded
'''
def del_model(model):
	try:
		del_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		del_model_client(model)
		return True
	except:
		return False


'''! Generates a random number 
@param min (int): the minimum number
@param max (int): the maximum number

@return int : the random number generated
'''
def randNum(min, max):
	num = round(random.uniform(min, max), 2)
	return num


'''! Generates a random position and rotation in the spawning zone

@return Pose : the generated position for the brick
'''
def random_position():
	s = lego.split("-")

	r = int(s[1][1]) * toll

	while 1:
		x = randNum(0.05, 0.4)
		y = randNum(0.23, 0.72)
		if x < 0.35 or (y < 0.2 or y > 0.5):
			break
	z = 0.90

	initial_pose = Pose()
	initial_pose.position.x = x
	initial_pose.position.y = y
	initial_pose.position.z = z
	
	q = quaternion_from_euler(0, 0, randNum(0, pi/4))
	#q = quaternion_from_euler(0, 0, 0)

	initial_pose.orientation.x = q[0]
	initial_pose.orientation.y = q[1]
	initial_pose.orientation.z = q[2]
	initial_pose.orientation.w = q[3]

	return initial_pose


'''! Changes the color of model
@param (xml): xml of model
@param (string): color to apply

@return string: color
'''
def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')


'''! This function check if there is conflict in spawn with other legos
@param pos (array): positions of other legos
@param lego (string): new lego

@return bool : True if there is conflict
'''
def check_sovrapposizioni(pos, lego):
	global errors
	s = lego.split("-")

	r = int(s[1][1]) * toll

	if spawned_lego:
		for p in spawned_lego:
			if(p.position.x - pos.position.x)**2 + (p.position.y - pos.position.y)**2 < r**2:
				print("Conflitto!\n\n")
				errors += 1
				return True 
	spawned_lego.append(pos)
	print("Valido!")
	return False


if __name__ == "__main__":
	rospy.init_node("spawn")

	print('************************')
	print('Spawn module ready')
	print('************************')
	print()
	print()

	while len(spawned_lego) < len(models):
		cont = 0
		spawned_lego = []
		errors = 0
		i = 0

		print("Removing models:")
		for m in models:
			print("Trying to remove: " + m + "_" + str(i))
			del_model(m + "_" + str(i))
			i += 1
		print()
		print()
		
		for lego in models:
			while errors < 500:
				print(lego + ":")
				pos = random_position()
				print(pos)
				if not check_sovrapposizioni(pos, lego):
					print(str(spawn_model(lego, pos)) + "\n\n")
					break
			else:
				break
	
	print('************************')
	print('Terminate spawn module')
	print('************************')
