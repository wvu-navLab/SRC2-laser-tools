
# SRC2-laser_tools
tools for using the lidar from SRC2 in 3D
```
roslaunch scan_assemble.launch 
rosrun laser_tools scout_laser_tools.launch
rosservice call /scout_1/scan_to_cloud "minAngle:
  data: -0.395
maxAngle:
  data: 0.78
numAngleSteps:
  data: 10
numRepeat:
  data: 1" 

```

TODO: add launches for hauler & excavator
