#ifndef _BASE_CONTROLLER_H_
#define _BASE_CONTROLLER_H_


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <cmath>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/server.h>

#include <nox/BaseControllerConfig.h>


const double two_pi = 6.28319;

class BaseController
{
    public:
        BaseController();
        ~BaseController();

        void Handlespeed(const geometry_msgs::Vector3Stamped& speed);
        void PublishTF();

        void DynamicCallback(nox::BaseControllerConfig& config, uint32_t level);

    private:
        double radius;                              //Wheel radius, in m
        double wheelbase;                          //Wheelbase, in m
        double two_pi;
        double speed_act_left;
        double speed_act_right;
        double speed_req1;
        double speed_req2;
        double speed_dt;


        char base_link[];
        char odom[];
        char kinect[];
        char camera_link[];
        bool publish_tf;

        double linear_scale_positive;
        double linear_scale_negative
        double angular_scale_positive;
        double angular_scale_negative;

        ros::NodeHandle private_nh;
        ros::NodeHandle global_nh;

        ros::Rate rate;
        
        ros::Subscriber sub;
        ros::Publisher odom_pub;
        tf::TransformBroadcaster broadcaster;  

        ros::Time current_time;
        ros::Time speed_time;
};

#endif