# Reprojection ROS Package
This ROS node runs images through the Reprojection shader. 

First clone this repository on your machine
```
git clone https://github.com/DanielArnett/reprojection_ros.git
or 
git clone git@github.com:DanielArnett/reprojection_ros.git
```

Then build your catkin workspace.
```
cd <catkin_ws>
catkin_make
```

To run the dual-fisheye blending node run
```
roslaunch reprojection dual_fisheye_to_equi.launch
```

To dynamically reconfigure the shader's parameters, open a second terminal and run:

```
source <catkin_ws>/devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

You can now edit the params on the fly, tuning them while the node is running.
