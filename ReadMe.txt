NAO Shooting Game

The NAO Shooting Game project provides a package that uses most features of the NAO to play a game in which the NAO shoots at movable targets in its field of view by moving its
head completely to the left and then to the right. Afterwards, it will iteratively align itself with a target and shoot at it with the bow by adjusting the height of the bow.
However, the NAO will need your help to use the bow. because both its strength and mobility are restricted compared to those of a human! Following are two sets of instructions
and the list of dependencies. Firstly, one instruction set detailing how to manually help the NAO after it has raised the bow towards one target. Additionally, the instruction
set to execute the package.

HUMAN INTERACTION INSTRUCTIONS

- Load the bow by firstly introducing an arrow into the front of the bow.
- Then you will need to help him pull the string. For this we have equipped the NAO with a 3d printed piece attached to a blue piece of cloth to the robot's left arm
- Simply use the piece to pull the string back and store the energy.
- You can then let the NAO know you are done by touching its head. The NAO will then pull his left arm, effectively releasing the piece and firing the arrow.


CODE EXECUTION INSTRUCTIONS

- Build the code with catkin build
- Make sure to source ROS and the PROJECT package
- Start the nodes of the NAO with "roslaunch nao_bringup nao_full_py.launch"
- Start speech, tactile, and leds with
    - roslaunch nao_apps speech.launch
    - roslaunch nao_apps tactile.launch
    - roslaunch nao_apps leds.launch
- Start our service modules with "roslaunch PROJECT nao_service_modules.launch"
- Whenever you are ready, start the shooting game with "rosrun PROJECT main_engine.py"


PROJECT DEPENDENCIES

These are the packages used

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  naoqi_bridge_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

In addition, python libraries numpy, cv2, and the NAOqi API