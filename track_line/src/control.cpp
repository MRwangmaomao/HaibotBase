#include "image_follow_line/image_follow_line.h"

control::control()
{
    track_dis_ = 0.3836;
    buffer_len = 10;
    vc_ = 0.3;

    kP = 1;
    kI = 0.1;
    kD = 0;


    odom_sub_ = n_.subscribe<nav_msgs::Odometry>("odom", 100, &control::OdomCallBack, this);
    odom_vth_sub_ = n_.subscribe<nav_msgs::Odometry>("odom_vth", 10, &control::OdomVthCallBack, this);
    speed_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void control::OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom)
{   
    double vx = odom->twist.twist.linear.x;
    double vth = odom->twist.twist.angular.z;

    double left_old = vx - vth * track_dis_ * 0.5;
    double right_old = vx + vth * track_dis_ * 0.5;
    
    if(left_v.size() > buffer_len)
    {
        left_v.pop();
        right_v.pop();
        left_v.push(left_old);
        right_v.push(right_old);
    }else{
        left_v.push(left_old);
        right_v.push(right_old);
    }
}

void control::OdomVthCallBack(const nav_msgs::Odometry::ConstPtr &odom)
{

}

void control::PublishSpeed(double left, double right)
{
    pub_speed_.linear.x = (left + right) * 0.5;
    pub_speed_.angular.z = (right - left) / track_dis_;
    speed_pub_.publish(pub_speed_);
}

void control::CheckErrorFront(double error, double vc, double kp, double ki, double kd){
    if(error_front.size() > buffer_len)
    {
        error_front.pop();
        error_front.push(error);
    }else{
        error_front.push(error);
    }
    //PID
    CountPID(error_front, vc, kp, ki, kd);
    //PublishSpeed
}

void control::CheckErrorBack(double error){
    if(error_back.size() > buffer_len)
    {
        error_back.pop();
        error_back.push(error);
    }else{
        error_back.push(error);

    }
}

void control::CountPID(queue<double> error, double vc, double kp, double ki, double kd)
{
    double left, right;
    double P = kp * error.back();
    double I = 0;
    double D = 0;
    vector<double> v_error;
    for (int i  = 0; i < error.size(); i++)
    {
        I = I + error.front();
        v_error.push_back(error.front());
        error.pop();
    }
    I = ki * I;
    D = kd * (v_error[0] - v_error[1]);

    double w = (P + I + D)/100.0;
    left = vc - w * track_dis_ * 0.5;
    right = vc + w * track_dis_ * 0.5;
    PublishSpeed(left, right);
}

