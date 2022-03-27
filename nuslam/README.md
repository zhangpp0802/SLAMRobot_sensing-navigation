# NuSlam

* Red path: the real path of the simulated robot
* Blue path: the estimated path through SLAM, approximately matches the real path
* Green path: the estimated path through odometry only, does not match the real path

# Example Usage and results
1. Similating Slam
```
roslaunch nuslam slam.launch robot:=nusim
```
![Sample](images/slam.png)

2. Detection of landmarks
```
roslaunch nuslam landmark_detect.launch nusim:=true
```
![Sample2](images/landmark.png)

3. Data association simulation
```
roslaunch nuslam unknown_data_assoc.launch robot:=nusim
```
![Unknown data association simulation](images/data_as.png)

* you can also stimulate them with turtlebot with adding in the turtlebot name at robot.