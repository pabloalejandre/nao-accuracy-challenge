# NAO Accuracy Challenge

The NAO Accuracy Challenge project provides a ROS package that uses most features of the humanoid robot NAO to play a game in which the NAO shoots at movable targets in its proximity with a toy bow. To do so, the robot will first scan the area to look for targets and then successively aim at the center of each of the using computer vision and movement control algorithms  

### Stucture

- `images`: Contains reference images for target detection algorithms.
- `launch`: Contians the launch file to start all services (vision, control, shooting...)
- `scripts`: Implementations of the ROS services and nodes of the package.
- `srv`: Files describing the ROS service types used.


### Run the package

- Build the package with catkin build
- Make sure to source ROS and the nao_accuracy_challenge
- Start the nodes of the NAO with "roslaunch nao_bringup nao_full_py.launch"
- Start speech, tactile, and leds with
    - roslaunch nao_apps speech.launch
    - roslaunch nao_apps tactile.launch
    - roslaunch nao_apps leds.launch
- Start our service modules with "roslaunch nao_accuracy_challenge nao_service_modules.launch"
- Whenever you are ready, start the shooting game with "rosrun nao_accuracy_challenge main_engine.py"

### Contributors

The NAO Accuracy Challenge (or NAO Shooting Game) was developed by Tuna Gürbüz, Andrew Lai, Nick Zhou, and Pablo Alejandre (myself) as our final project for the course Humanoid Robotics Systems at TUM.
