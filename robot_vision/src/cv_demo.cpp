#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"cv_demo");
    cv::Mat image;
    cv::VideoCapture capture(0);
    if(!capture.isOpened()){
        ROS_ERROR("failed to open camera");
        ros::shutdown();
    }
    image = cv::imread("/home/chasing/Templates/ros_tmp/src/opencvtest/src/ubuntu.png");

    ROS_INFO_STREAM("opencv"<<image.cols<<"\t"<<image.rows);
    cv::imshow("image",image);
    cv::waitKey(0);
    ros::NodeHandle nh;
    ros::Publisher image_pub= nh.advertise<sensor_msgs::Image>("myopencv/image_raw",10);
    cv::Mat edges;

    ros::Rate rate(10);
    while(ros::ok())
    {

        capture>>edges;
        cv::imshow("image",edges);        
        cv::waitKey(2);
        image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", edges).toImageMsg());
        // cv::cvtColor(image,edges,CV_BGR2GRAY);
        cv::namedWindow("gray window");
        cv::imshow("gray window",image);
        cv::waitKey(2);

        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM_THROTTLE(1,"hello,opencv");
    }
}
