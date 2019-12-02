# Reprojection ROS Package
This ROS node runs images through the Reprojection shader. 

To dynamically reconfigure the shader's parameters, open a second terminal and run:

```
source <catkin_ws>/devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

You can now edit the params on the fly, tuning them while the node is running.
