# Reprojection ROS Package
This ROS node runs images through the Reprojection shader. 

First clone this repository on your machine
```
git clone https://github.com/DanielArnett/reprojection_ros.git
```

Also clone this repository, which reprojection depends on. 
```
git clone https://github.com/mohammedari/opengl_ros.git
```

Then build your catkin workspace.
```
source <catkin_ws>/devel/setup.bash
cd <catkin_ws>
catkin_make
```

To run the dual-fisheye blending node run
```
roslaunch reprojection dual_fisheye_to_equi.launch
```

This will open three image windows, one with the raw input, one with the left camera output, and one with the right camera input. It will also open the rqt_reconfigure window, where you may enter parameters for the camera. For now you should click on the left_equi and right_equi dropdown icons, and select both of the reprojection nodes. This will open all of the reprojection parameters for both cameras. At the top of each parameter list you'll se a Save and a Load icon. Click the load icon and navigate to the reprojection_ros/cfg directory. Here you'll find a saved .yaml config file for each camera which you can use as a starting point.


