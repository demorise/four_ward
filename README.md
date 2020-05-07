# 4Ward - Robot Guidance with MoveIt #

### ME369P  Ademola Oridate, Ed Hu, Doug Feicht
Final presentation [(here)](https://docs.google.com/presentation/d/17Ukz5PC6BCps5GtElDTN0TVy6ZJnzbunL3J4pHO2W8U/edit?usp=sharing)

This project explores Virtual Fixtures, a method of assisting or constraining robot motion, using python and ROS/moveIt for visualization. 

## Getting Started
Python code was developed to enable a joystick to be used to simulate robot motion in RViz. 
These instructions should help you install and run the project on your local machine for development and testing purposes. 

### Prerequisites
The code has been developed and tested with Ubuntu 14.04 and ROS Kinetic with MoveIt Kinetic, running on one or more X86-64 PCs using an Xbox One controller. ROS, moveIt and ros key packages should be installed per these instructions

- [ ] ROS Kinetic installation [wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [ ] ROS Network setup [wiki](http://wiki.ros.org/ROS/NetworkSetup)
- [ ] ROS Multiple Machines [tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
- [ ] Configuring and Using a Linux-Supported Joystick with ROS [wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
- [ ] MoveIt Kinetic installation [wiki](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
- [ ] Move Group Python Interface [tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) 

### Installing
To install the software, 
- [ ] Clone the following repositories into your catkin workspace 'src' folder:
```
git clone https://github.com/demorise/four_ward.git
git clone https://github.com/demorise/franka_ros.git
git clone https://github.com/demorise/panda_moveit_config.git
```
- [ ] Ensure MoveIt is installed
- [ ] Ensure MoveIt Visual Tools is installed [GitHub](https://github.com/ros-planning/moveit_visual_tools):
```
sudo apt-get install ros-kinetic-moveit-visual-tools
```
- [ ] Ensure MoveIt Resources is installed http://wiki.ros.org/moveit_resources
- [ ] Install pyassimp 3.3 to ensure mesh files are loaded correctly:  
```
$pip install pyassimp==3.3
```
- [ ] Open a terminal and navigate into your workspace directory, build the entire workspace ($catkin build) or build individual packages:
```
catkin build four_ward
catkin build franka_description
catkin build panda_moveit_config
source devel/setup.bash
```
- [ ] Open 5 terminal windows and execute these commands in order, each in their own window.

```
roslaunch panda_moveit_config demo.launch            # Launches RViz visualization tool
rosrun joy joy_node                                  # Launches joystick controller
rostopic echo joy                                    # confirm joystick is working
rosrun four_ward add_objects_and_markers.py          # loads workspace objects and markers     
rosrun four_ward jogger.py                           # robot motion code
rosrun four_ward trace.py                            # visualize trajectory of end-effector
```
## Using the Xbox One controller
![Xbox One joystick mapping](https://github.com/demorise/four_ward/blob/master/Xbox%20controller%20joystick%20mapping.png?raw=true)

The ROS joy_node obtains key data from the joystick and publishes rostopic 'joy' mesages (above) over the ROS network to a jogger node that converts the button and axes joystick outputs into cartesian and joint move commands. 
Three groups of controller buttons are mapped to robot drive functions as follows:
- D-pad "cross" (lower left): Drives robot parallel to the X and/or Y axis, one increment per key press.
- Right & Left fron "bumpers": Selects joint and sign for joint movement
- "YXBA" buttons: jog individual robot joints one increment based on front bumpers state


## Jogger Code Overview
[Jogger Code Overview](https://github.com/demorise/four_ward/blob/master/Jogger%20Code%20Overview.png?raw=true)

The jogger code receives joystick data and calls a **_moveIt_commander_** method to move the robot within the scene.
First, modules are imported and a planning group instantiated for the Franka Panda robot arm.
```python 
import rospy
import moveit_commander
group = moveit_commander.MoveGroupCommander('panda_arm')
```

A collision object is added to the planning scene (a box representing surgical table), and the rostopic "joy" is subscribed.
```python
#Add collision object to planning scene
scene = moveit_commander.PlanningSceneInterface()
scene.add_box(box_name, box_pose, size = (3, 1, 0.1))

#Subscribe to joystick ros topic: ‘joy’
rospy.Subscriber(‘joy’, Joy, callback)
```

Received messages will be handled by the subscriber such that the callback() function is called. Callback will receive the command, and add a corresponding move increment to the current poses to create a new pose goal.
```python
def callback (data)
   #data.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0]
   #data.buttons = [(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
   curr_cart_pose = get_current_pose().pose
   pose_goal = get_current_pose().pose

   pose_incr =0.075
   #jog in X axis
   if data.axes[6]==1:
      pose_goal.position.x = curr_cart_pose.position.x - pose_incr
   go_to_pose_goal(pose_goal, group)
```

The go_to_joint_state() and go_to_pose_goal() will move the robot to the updated goal
```python
def go_to_joint_state (joint_goal, group)
   group.go(joint_goal, wait=True)
   group.stop()

def go_to_pose_goal (pose_goal, group)
   group.set_pose_target(pose_goal)
   plan = group.go(wait=True)
   group.stop()
```

## Running the tests
Markers and colision objects are added to the sceen using rospy.publisher() to publish the details to the RViz visualiation node.
```python
#Publish Marker to RVIZ
marker =  Marker()
pub =  rospy.Publisher(marker_topic, Marker, queue_size = 10)
 
#Add collision object to planning scene
scene =  moveit_commander.PlanningSceneInterface()
scene.add_box(box_name, box_pose, size = (0.4, 0.4, 0.4))
```

### Cartesian Jogging
Use the D-pad keys to move the robot end effector parralel to the grid.
Use buttons Y and A to jog the end-effector in the z-axis

### Jog Individual Joints
Press the right bumper (front, upper) button while pressing one of the "YXBA" buttons or D-pad to move individual joints. Press the left bumper to move negative direciton.

### Virtual Fixture 
The jogger.py code includes routines to determine how close the end effector is to a desired path segment by computing the least perpendicular distance. If enabled, the vf_force_filter() command will evaluate the requested move as a force, and either update the jog postion by returning a unit vector times the jog incrment, or return zero if the move is denied. 

### Original Code
The Scripts folder in this repository contains
* __add_objects_and_markers.py__ (Ademola) loads 3D objects into the scene representing an application (surgery). Mesh files in STL format represent a surgical table, end effector probes and even a model of a patient. Prismatic markers are used to represent the table and a "forbidden zone" about the patient for intitial collision detection. 
* __jogger.py__ (Ademola) primary functions that receive a subset of joy commands and convert them into cartisian or joint jog commands. Other functions evaluate position requests using Virtual Fixture algorithms to guide or impede motion.
* __virtual_Fixture.py__ (Ed Hu) initial investigation into VF. The code:
   1. defines a plane (normal to [a,b,c] and determine distance to the current end effector position, creating a vector at the end effector normal to the plane.
   2. If too close, the request is overriden with a move that pushes away from the plane along the normal (after checking sign to ensure direction).
   3. Otherwise, return same position and proceed
[VF Distance Normal to Plane](https://github.com/demorise/four_ward/VF_plane.tiff) 
* __trace.py__ (Ademola) creates markers on the robot end-effector during jogging to visualize trajectory.

### Testing
The code was developed by Ademola and Ed Hu on ubuntu machines at their residences. Doug Feicht independently tested code and prepared this document.  

