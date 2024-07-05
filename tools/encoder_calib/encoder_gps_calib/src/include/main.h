//
// Created by echo on 23-3-30.
//

#ifndef ENCODER_CALC_MAIN_H
#define ENCODER_CALC_MAIN_H

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Eigen>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include "deque"


class main_fuction {
public:
    main_fuction(){};
    ros::Publisher encoder_odom_pub ;
    ros::Publisher gps_odom_pub ;
    ros::Publisher encoder_path_pub ;
    ros::Publisher refined_encoder_path_pub ;
    ros::Publisher gps_path_pub ;
    double gps_cov_threshold=0.5;//use cov < 0.5
    double gps_interval_threshold=0.5; //add new point per 0.5m
    double wheel_track = 0.76;
    double wheel_radius = 0.22;
    double wheel_radius_r = 0.22;
    double wheel_radius_l = 0.22;
    double distance_x_prev = 0;
    double distance_y_prev = 0;
    ros::Time last_time;
    double yaw = 0;
    double curr_x =0;
    double curr_y =0;
    bool first_call = true;
};


#endif //ENCODER_CALC_MAIN_H
