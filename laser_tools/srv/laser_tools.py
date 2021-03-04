#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from laser_tools_src2.srv import ScanToPointCloud2, ScanToPointCloud2Response
from laser_assembler.srv import AssembleScans2
import roslib; roslib.load_manifest('laser_assembler')

def tiltLidar(mess):
    node='laser_tools'
    print(rospy.get_param(node+"/cloud_topic_name"))
#    print rospy.get_param(node+"/sensor_control_topic_name")

    pub_pitch = rospy.Publisher(rospy.get_param(node+"/sensor_pitch_control_topic_name"),Float64, queue_size=1)
    pub_yaw = rospy.Publisher(rospy.get_param(node+"/sensor_yaw_control_topic_name"),Float64, queue_size=1)

    pub_cloud = rospy.Publisher(rospy.get_param(node+"/cloud_topic_name"),PointCloud2, queue_size=1)

    minPitchAngle = float(mess.minPitchAngle)
    maxPitchAngle = float(mess.maxPitchAngle)
    minYawAngle = float(mess.minYawAngle)
    maxYawAngle = float(mess.maxYawAngle)
    pitchSteps = mess.numPitchSteps
    yawSteps = mess.numYawSteps
    repeat = mess.numRepeat

    pitchIncrement = (maxPitchAngle-minPitchAngle)/float(pitchSteps)
    yawIncrement = (maxYawAngle-minYawAngle)/float(yawSteps)

    sleep = mess.sleepScan

    for j in range(repeat):

        pitch = minPitchAngle
        for i in range(pitchSteps):
            pub_pitch.publish(pitch)
            rospy.sleep(sleep)

            pitch = pitch +pitchIncrement

            yaw = minYawAngle
            for k in range(yawSteps):
                pub_yaw.publish(yaw)
                rospy.sleep(sleep)

                yaw = yaw +yawIncrement
                if(j==0 and i==0 and k==0):
                    startTime = rospy.get_rostime()




    rospy.sleep(sleep)s
    endTime = rospy.get_rostime();
    pub_pitch.publish(0.0);
    pub_yaw.publish(0.0);

    print("finish scan loop, before assemble_scans2")
    rospy.wait_for_service("assemble_scans2")
    assembler= rospy.ServiceProxy("assemble_scans2",AssembleScans2)
    print("after scan assembler call")

    resp = assembler(startTime, endTime)

    pub_cloud.publish(resp.cloud)
    return ScanToPointCloud2Response(resp.cloud)


node='laser_tools'
rospy.init_node(node)
robot = rospy.get_param(node+"/robot_name")
rospy.get_param(node+"/cloud_topic_name")

service=rospy.Service('scan_to_cloud',ScanToPointCloud2,tiltLidar)

rospy.spin()
