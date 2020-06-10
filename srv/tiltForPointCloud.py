#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from laserScanToPointCloud2.srv import ScanToPointCloud2, ScanToPointCloud2Response
from laser_assembler.srv import AssembleScans2

def tiltLidar(mess):
    pub = rospy.Publisher('/scout_1/sensor_controller/command',Float64, queue_size=1)
    pub_cloud = rospy.Publisher('/scout_1/Cloud',PointCloud2, queue_size=1)
    startTime = rospy.get_rostime()
    Angle = -.3927
    for i in range(20):
        
        pub.publish(Angle);
        Angle=Angle + .0585
        rospy.sleep(.1)

    endTime = rospy.get_rostime();
    pub.publish(0.0);
  #  rospy.wait_for_service('laser_assembler')
    
    rospy.wait_for_service("assemble_scans2")

    assembler= rospy.ServiceProxy("assemble_scans2",AssembleScans2)
    

    resp = assembler(startTime, endTime)
    
    pub_cloud.publish(resp.cloud)
    return ScanToPointCloud2Response(resp.cloud)



rospy.init_node('lidar_pointCloud')

service=rospy.Service('laserScanToPointCloud2',ScanToPointCloud2,tiltLidar)

rospy.spin()

