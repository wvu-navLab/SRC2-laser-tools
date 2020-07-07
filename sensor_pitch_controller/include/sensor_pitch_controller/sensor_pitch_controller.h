/*!
 * \laser_pitch_controller.h
 * \brief laser_pitch_controller (...).
 *
 * Wheel control (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef SENSOR_PITCH_CONTROLLER_H
#define SENSOR_PITCH_CONTROLLER_H

// Include cpp important headers
#include <math.h>
#include <stdio.h> 

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <motion_control/SensorJoint.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

class SensorPitchController
{
private:
    // Node Handle
    ros::NodeHandle & nh_;
    // Publisher
    ros::Publisher pubSensorJoint;

    // Subscriber
    ros::Subscriber subOdomTruth;
    // ros::Subscriber subJointStates;

    // void doCalculations();
    // Callback function for subscribers.
    void odometryTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);

    double roll_, pitch_, yaw_;
 
    // double sensor_joint_current_;
    double sensor_joint_cmd_;

    // PID gains
    double Kp_ = 1;

public:
    SensorPitchController(ros::NodeHandle & nh);

    void controlSensor();

};



#endif // SENSOR_PITCH_CONTROLLER_H
