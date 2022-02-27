#include <nox/base_controller.h>


BaseController:BaseController():
global_nh("/"),
speed_time(0.0)
{

  speed_act_left = 0.0;
  speed_act_right = 0.0;
  speed_req1 = 0.0;
  speed_req2 = 0.0;
  speed_dt = 0.0;


  sub = private_nh.subscribe("speed", 50, &BaseController::handle_speed, this);
  odom_pub = private_nh.advertise<nav_msgs::Odometry>("odom", 50);

  f = boost::bind(&BaseController::DynamicCallback,this, _1, _2);
  server.setCallback(f);
}

BaseController::~BaseController(){};

void BaseController::Handlespeed(const geometry_msgs::Vector3Stamped& speed)
{
  speed_act_left = trunc(speed.vector.x*100)/100;
  ROS_INFO("speed left : %f", speed_act_left);
  speed_act_right = trunc(speed.vector.y*100)/100;
  ROS_INFO("speed right : %f", speed_act_right);
  speed_dt = speed.vector.z;
  speed_time = speed.header.stamp;
}

void BaseController::PublishTF()
{

  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  while(n.ok()){
    ros::spinOnce();
    current_time = speed_time;
    dt = speed_dt;					//Time in s
    ROS_INFO("dt : %f", dt);
    dxy = (speed_act_left+speed_act_right)*dt/2;
    ROS_INFO("dxy : %f", dxy);
    dth = ((speed_act_right-speed_act_left)*dt)/wheelbase;

    if (dth > 0) dth *= angular_scale_positive;
    if (dth < 0) dth *= angular_scale_negative;
    if (dxy > 0) dxy *= linear_scale_positive;
    if (dxy < 0) dxy *= linear_scale_negative;

    dx = cos(dth) * dxy;
    dy = sin(dth) * dxy;

    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;

    theta = (theta >= two_pi) ? theta : theta - two_pi;
    theta = (theta <= -two_pi) ? theta : theta + two_pi;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::Quaternion empty_quat = tf::createQuaternionMsgFromYaw(0);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;
      geometry_msgs::TransformStamped k;
      
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;
      
      k.header.frame_id = kinect;
      k.child_frame_id = camera_link;
      k.transform.translation.x = 0.0;
      k.transform.translation.y = 0.0;
      k.transform.translation.z = 0.0;
      k.transform.rotation = empty_quat;
      k.header.stamp = current_time;

      broadcaster.sendTransform(t);
      broadcaster.sendTransform(k);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (speed_act_left == 0 && speed_act_right == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    vx = (dt == 0)?  0 : (speed_act_left+speed_act_right)/2;
    vth = (dt == 0)? 0 : (speed_act_right-speed_act_left)/wheelbase;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dth;

    odom_pub.publish(odom_msg);
    rate.sleep();

}

BaseController::DynamicCallback(nox::BaseControllerConfig& config, uint32_t level)
{
    radius = config.radius;
    wheelbase = config.wheelbase;
    rate = ros::Rate(config.rate);


    base_link = config.base_link_id;
    odom = config.odom_link_id;
    kinect = config.kinect_link_id;
    camera_link = config.camera_link_id;

    publish_tf = config.publish_tf;

    linear_scale_positive = config.linear_scale_positive;
    linear_scale_negative = config.linear_scale_negative;
    angular_scale_positive = config.angular_scale_negative;
    angular_scale_negative = config.angular_scale_negative;

}

