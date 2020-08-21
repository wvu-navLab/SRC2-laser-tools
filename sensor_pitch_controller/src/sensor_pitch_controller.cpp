/*!
 * \sensor_pitch_controller.cpp
 * \brief wheel_control (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 01, 2020
 */

#include "sensor_pitch_controller/sensor_pitch_controller.h"

SensorPitchController::SensorPitchController(ros::NodeHandle & nh)
: nh_(nh)
{
    subOdomTruth = nh_.subscribe("odometry/truth", 1000, &SensorPitchController::odometryTruthCallback, this);
    pubSensorJoint = nh_.advertise<motion_control::SensorJoint>("sensor_joint_angle", 1000);
}

void SensorPitchController::odometryTruthCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );

    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);

    // ROS_INFO("New current odometry.");
    // ROS_INFO_STREAM("Pitch "<< pitch_);
    }


void SensorPitchController::controlSensor()
{

    // See https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp
    // for a better implementation
    sensor_joint_cmd_  = Kp_*pitch_;

    // double j1 = Kp_*(sensor_joint_cmd_ - sensor_joint_current_);

    motion_control::SensorJoint j;

    j.j1 = sensor_joint_cmd_; 

    pubSensorJoint.publish(j);
    // ROS_INFO("Sensor joint angle published.");
    // ROS_INFO_STREAM("Commanded joint angle "<< sensor_joint_cmd_);
}

/*!
 * \brief Creates and runs the wheel_control node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_pitch_controller");
    ros::NodeHandle nh("");
    ros::Rate rate(100);

    ROS_INFO("Wheel Velocity Controller initializing...");
    SensorPitchController sensor_pitch_controller(nh);

    while(ros::ok()) 
    {
        sensor_pitch_controller.controlSensor();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
