# NAO Shooting Game

The NAO Shooting Game project provides a ROS package that uses most features of the humanoid robot NAO to play a game in which the NAO shoots at movable targets in its field with a toy bow. To do so, the robot will first scan the area to look for targets and, then successively aim at the center of each of them. 

### Stucture

- `images`: Contains reference images for target detection algorithms.
- `launch`: Contians the launch file to start all services (vision, control, shooting...)
- `scripts`: Implementations of the ROS services and nodes of the package.
- `srv`: Files describing the ROS service types used.


### Run the package

- Build the package with catkin build
- Make sure to source ROS and the nao-shooting-game
- Start the nodes of the NAO with "roslaunch nao_bringup nao_full_py.launch"
- Start speech, tactile, and leds with
    - roslaunch nao_apps speech.launch
    - roslaunch nao_apps tactile.launch
    - roslaunch nao_apps leds.launch
- Start our service modules with "roslaunch PROJECT nao_service_modules.launch"
- Whenever you are ready, start the shooting game with "rosrun PROJECT main_engine.py"


Developed by Tuna Gürbüz, Andrew Lai, Nick Zhou, and Pablo Alejandre as their final project for Humanoid Robotics Systems.
