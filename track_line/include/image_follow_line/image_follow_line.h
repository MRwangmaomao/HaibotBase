#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace std;

class control{
    public:
        control();
        void OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom);//get encoder vL, vR
        void OdomVthCallBack(const nav_msgs::Odometry::ConstPtr &odom);//get IMU omega
        void PublishSpeed(double left, double right);
        void CountPID(queue<double> error, double vc, double kp, double ki, double kd);
        void CheckErrorFront(double error,  double vc, double kp, double ki, double kd);
        void CheckErrorBack(double error);
    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub_;
        ros::Subscriber odom_vth_sub_;
        ros::Publisher  speed_pub_; 
        

        geometry_msgs::Twist pub_speed_;
        double track_dis_;
        int buffer_len;

        queue<double> left_v;
        queue<double> right_v;

        double vc_;
        queue<double> error_front;
        queue<double> error_back;

        double kP,kI,kD;
};