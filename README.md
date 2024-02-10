# FUNDAMENTALS OF ROBOTICS - UR5 PROJECT
## Marco Soldera (226651) - Marco Morandin (228160)

### Basic Setup

We **suggest** to follow the locosim README until this point https://github.com/mfocchi/locosim#python to setup Gazebo, Rviz and all related programs; for locosim setup then follow this README.

### Components installation for training
Run this commands to install all the required components to run the training
```
pip install ultralytics
```

```
pip install pathlib
```

```
pip install Counter
```

```
pip install scikit-learn
```

### Components installation for vision
Run this commands to install all the required components to run the vision part of the project
```
sudo apt-get install ros-ROS_VERSION-cv-bridge
```

```
pip3 install roslibpy
```

```
pip install opencv-python
```

```
pip install numpy
```

```
pip install ultralytics
```

```
pip install pillow
```

```
pip install pandas
```

### Repository Setup

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

Now you need to call the following line manually (next you will see that it will be done automatically in the .bashrc)

```
source /opt/ros/ROS_VERSION/setup.bash
```

```
cd ~/ros_ws/
```

```
 catkin_make
```

```
 cd ~/ros_ws/src/ 
```

Now you can clone the repository inside the ROS workspace you just created:

```
git clone https://github.com/Soldera21/UR5_robotics
```

And move it to ~/ros_ws/src/ folder:

```
cd ~/ros_ws/src/
mv UR5_robotics/* .
rmdir UR5_robotics
```

Once you have cloned it:

```
cd ~/ros_ws/
```

```
catkin_make install
```

Now you can continue to follow locosim guide https://github.com/mfocchi/locosim#configure-environment-variables.

### Run the project

Open a terminal and run:

```
python3 -i $LOCOSIM_DIR/robot_control/base_controllers/ur5_generic.py
```

Once the homing has completed in another terminal:

```
cd ~/ros_ws/src/spawnLego
```
```
python3 spawnLego.py
```

The spawning process can delete and replace bricks due to available positions that are colliding with already existing pieces; this procedure could take up to some seconds.

In another terminal run:

```
cd ~/ros_ws
```
```
rosrun planner_pkg planner_node
```

That contains planner and movement module all in one. Planner keeps listening for new positions from vision.

Open the last terminal and run the vision module:

```
cd ~/ros_ws/src/vision
```
```
python3 vision.py
```

Now the project is running and the pieces are being placed in their positions!

### Video demo

Here you can find a little demo of the movement of the robot:

<img src="https://github.com/Soldera21/UR5_robotics/blob/master/documentation/img/placement_ezgif.gif">

(This video is speeded up by 500% only to show how the trajectory looks like)
