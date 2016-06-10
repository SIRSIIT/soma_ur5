# soma_ur5 package
This package contains the stuff for the UR5 setup in SOMA project.

get dependencies:
* `sudo apt-get install ros-indigo-leap-motion`

#add more dependencies
get the forked versions of the packages in https://github.com/SIRSIIT :
* `roscd && cd ../src`
* `git clone https://github.com/SIRSIIT/universal_robot.git` for corrected kinematics
* `git clone https://github.com/SIRSIIT/pisa-iit-soft-hand.git` for the soft hand to work :-/
* `git clone https://github.com/SIRSIIT/ros_control.git` for some reason
* `git clone https://github.com/SIRSIIT/gazebo_ros_pkgs.git` for some hackery

* `mkdir ~/code && cd ~/code && git clone https://github.com/qbrobotics/handadmin.git && git clone https://github.com/qbrobotics/qbAPI.git && cd qbAPI && git checkout 493b2591c5333d42ea5e7306caf203c42b56b292`
* `roscd soft_hand && cd ../hand-tools && rm -r handadmin qbAPI &&  ln -s ~/code/qbAPI && ln -s ~/code/handadmin`
* `cd ~/code/qbAPI/src && git checkout 72c0d68139060b95eeeb0f175ee44610f9ca2f71`

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




