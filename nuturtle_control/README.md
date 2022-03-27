# Nuturtle  Control
* `roslaunch nuturtle_control start_robot.launch cmd_src:=teleop robot:=nusim use_rviz:=true` to see the robot in rviz.
* To clarify the robot and obstacles in different color: red is generated from nusim with just odom and added noises. Green is where things theoritically should be at. Blue is calculated through SLAM.