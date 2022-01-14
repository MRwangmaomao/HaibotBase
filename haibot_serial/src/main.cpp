#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>

#include <geometry_msgs/Twist.h>

#include "HaibotSerial.h"

using namespace std;


void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%f]", msg->linear.x);
}



// haibot_serial_node "/dev/ttyUSB0" 115200
int main(int argc, char** argv)//argc是命令行总的参数个数
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    if (argc != 3) {
        ROS_ERROR_STREAM("Please check port_id band_rate");
        return -1;
    }
    string portName = argv[1];
    int bandRate = std::atoi(argv[2]);

    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(portName);
    //设置串口通信的波特率
    sp.setBaudrate(bandRate);
    //串口设置timeout
    sp.setTimeout(to);

    ros::Subscriber velSub = n.subscribe("/cmd_vel", 100, CmdVelCallback);
    
    try {
        //打开串口
        sp.open();
    }
    catch (serial::IOException& e)//捕捉输入输出异常
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if (sp.isOpen()) {
        ROS_INFO_STREAM(portName << " is opened.");
    }
    else {
        return -1;
    }

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if (n != 0) {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);

            for (int i = 0; i < n; i++) {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    //关闭串口
    sp.close();
    ROS_INFO_STREAM("serial port is closed!!!");
    return 0;
}

