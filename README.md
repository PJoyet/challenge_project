# ROS and experimental robotics 

## Part 3: project - SAR group

| par :         |         |                                                              |
| ---- | ---- | ---- |
|   Pierre JOYET   |   3407684   |   [pierre.joyet@etu.sorbonne-universite.fr](mailto:pierre.joyet@etu.sorbonne-universite.fr )  |
|   Marine CORNET   |   3531423   |   [marine.cornet@etu.sorbonne-universite.fr](mailto:marine.cornet@etu.sorbonne-universite.fr )   |

### Contenus

- **config**
  - robot_with_sensors.yaml
  - turtlebot3_burger.yaml

* **launch**
- **challenge1**
    - challenge1_task1.launch
    - challenge1_task2.launch
    - challenge1_task3.launch
  - **challenge2**
    - challenge2_task1.launch
    - challenge2_task2.launch
    - challenge2_task3.launch
  - **challenge3**
    - challenge3_task1.launch
    - challenge3_task2.launch
        Due to the randomly part of the simulation the robot may not be able to reach the entrance or the line at the exit of the corridor the argument task1:=true/false is available for starting the simulation in front of the corridor (a restart may be necessary)
    - challenge3_task3.launch
        Due to the randomly part of the simulation the robot may not be able to reach the entrance or the line at the exit of the corridor the argument task2:=true/false is available for starting the simulation at the end of task 2 (a restart may be necessary)
  - gazebo.launch
  - rviz.launch
* **msg**
  * cardinal_direction.msg
* **rviz**
  * configuration.rviz
* **scripts**
  * challenge1
    - line_following.py
        Node for controlling the robot to follow lines
  * challenge2
    - challende2_task2_world_control.py
    - challende2_task3_world_control.py
    - distance_servoing.py
        Node for distance control with a moving forward/backward wall by reading the north value of topic lds_distance 
    - emergency_stop.py
        Node for stopping the robot in front of wall
    - moving_obstacle_avoidance.py
        Node for distance control with a moving 8 direction wall by reading the 8 values of topic lds_distance 
  * challenge3
    - challende3a_world_control.py
    - challende3b_world_control.py
    - movement_corridor.py
        Node for control the robot inside a corridor by reading the 8 values of topic lds_distance
    - movement_corridor_part2.py
        Node for controlling the robot to follow lines after the node movement_corridor
  * lds_distance_publisher.py
      Node for reading scan topic and send mean for the 8 cardinal directions in the topic lds_distance
* **src**
  * challenge_project
    * CamColorDetection.py
      Module to calculate the real distance of color center in a image and the linear and angular speed from it
* **urdf**
  * common_propertie.xacro
  * turtlebot3_burger.gazebo.xacro
  * turtlebot3_burger.urdf.xacro
* **worlds**
  * challenge1.world
  * challenge2.world
  * challenge3.world
* CMakeList.txt
* package.xml
* README.md
* setup.py
* project.pdf
