# soma_ur5 package
This package contains the stuff for the UR5 setup in SOMA project.

get dependencies:
sudo apt-get install ros-indigo-leap-motion
#add more dependencies
get the forked versions of the packages in https://github.com/SIRSIIT :
* `roscd && cd ../src`
* `git clone git@github.com:SIRSIIT/universal_robot.git` for corrected kinematics
* `git clone https://github.com/CentroEPiaggio/pisa-iit-soft-hand.git && git checkout acc0f2106ba17b05687c41427560cd25b88ade49` for the soft hand to work :-/
* `git clone https://github.com/CentroEPiaggio/ros_control.git && git checkout c34f02453d80cb70beed1da80b9c627175c43137` for some reason
* `git clone git@github.com:SIRSIIT/gazebo_ros_pkgs.git` for some hackery

* `catkin_make`

to run do:

* `roslaunch soma_ur5 ur5_robot.launch simulated:=true hand:=true`

Other arguments are:
  `robot_ip , sensor_ip , use_rviz , gz_gui`
  
  Choose between these to command the robot:
  *   `roslaunch soma_ur5 [commander]`
  
  where `[commander]` can be:
   * `com_haptic.launch`
   * `com_interactive.launch`
   * `com_leap.launch`

To offset the sensor do:

* `rosservice call /Bias_sensor`

Visualisation is done with either the `use_rviz` argument or
* `rosrun rviz rviz`

Further configurations (such as the joint speed) are made through:
* `rqt`




