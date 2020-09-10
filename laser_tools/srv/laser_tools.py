#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from laser_tools_src2.srv import ScanToPointCloud2, ScanToPointCloud2Response
from laser_assembler.srv import AssembleScans2
import roslib; roslib.load_manifest('laser_assembler')
def tiltLidar(mess):
    node='laser_tools'
    print rospy.get_param(node+"/cloud_topic_name")
    print rospy.get_param(node+"/sensor_control_topic_name")
    pub = rospy.Publisher(rospy.get_param(node+"/sensor_control_topic_name"),Float64, queue_size=1)
    pub_cloud = rospy.Publisher(rospy.get_param(node+"/cloud_topic_name"),PointCloud2, queue_size=1)
    
    minAngle = float(mess.minAngle)
    maxAngle = float(mess.maxAngle)
    steps = mess.numAngleSteps
    repeat = mess.numRepeat
    angleIncrement = (maxAngle-minAngle)/float(steps);
    print angleIncrement
    for j in range(repeat):
        Angle = minAngle
        for i in range(steps):
            pub.publish(Angle);
            rospy.sleep(2.0)
            if(j==0 and i ==0):
                startTime = rospy.get_rostime()
            print Angle
            print steps
            Angle=Angle + angleIncrement

    endTime = rospy.get_rostime();
    pub.publish(0.0);
  
    rospy.wait_for_service("assemble_scans2")
    assembler= rospy.ServiceProxy("assemble_scans2",AssembleScans2)
    

    resp = assembler(startTime, endTime)
    
    pub_cloud.publish(resp.cloud)
    return ScanToPointCloud2Response(resp.cloud)


node='laser_tools'
rospy.init_node(node)
robot = rospy.get_param(node+"/robot_name")
rospy.get_param(node+"/cloud_topic_name")

service=rospy.Service('scan_to_cloud',ScanToPointCloud2,tiltLidar)

rospy.spin()

