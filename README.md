# soma_ur5 package
This package contains the stuff for the UR5 setup in SOMA project.

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





