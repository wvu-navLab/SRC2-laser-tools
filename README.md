
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

roslaunch laser_tools_src2 scout_laser_tools.launch 
rosrun range_to_base rangeToBaseService.py 
rosservice call /range_to_base_service "angle: 0.5"
range: 
  header: 
    seq: 0
    stamp: 
      secs: 270
      nsecs: 270000000
    frame_id: ''
  radiation_type: 0
  field_of_view: 0.0
  min_range: 0.0
  max_range: 0.0
  range: 9.8836774826
```
