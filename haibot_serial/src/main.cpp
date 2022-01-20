#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/bind.hpp>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <haibot_serial/Shutdown.h>

#include "HaibotSerial.h"

using namespace std;

serial::Serial sp;

union UnionData {
    int16_t data;
    uint8_t mem[2];
};

string toHex(uint8_t number)
{
    char hexes[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    string hex = "";
    uint8_t nibble;
    do {
        nibble = number & 0x0f;
        number = number >> 4;
        hex = hexes[nibble] + hex;
    } while(number);
    if (hex.length() == 1) {
        hex = "0" + hex;
    }
    hex = "0x" + hex + "  ";
    return hex;
}

void DumpBuffer(uint8_t *buffer, int len)
{
    
    string bufferStr;
    for (int i = 0; i < len; i++) {
        bufferStr += toHex(buffer[i]);
    }
    std::cout << bufferStr << endl;
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    HaibotSerial serialObj = HaibotSerial();
    const unsigned int BUFFER_LENGTH = 13;
    uint8_t buffer[BUFFER_LENGTH] = { 0x20, 0x10, 0x00, 0x06, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    union UnionData speedLeft;
    speedLeft.data = static_cast<int16_t>(100 * (msg->linear.x + msg->angular.z * 0.1));
    union UnionData speedRight;
    speedRight.data = static_cast<int16_t>(-100 * (msg->linear.x - msg->angular.z * 0.1));
        
    serialObj.setSpeed(speedLeft.mem, speedRight.mem, buffer, BUFFER_LENGTH);
    //DumpBuffer(buffer, BUFFER_LENGTH);
    sp.write(buffer, BUFFER_LENGTH);
}

void ShutDownCallback(const haibot_serial::Shutdown::ConstPtr &msg)
{
    HaibotSerial serialObj = HaibotSerial();
    const unsigned int BUFFER_LENGTH = 8;
    uint8_t buffer[BUFFER_LENGTH] = { 0x20, 0x06, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00 };
    union UnionData delayTime;
    delayTime.data = static_cast<int16_t>(msg->delay_time);
    serialObj.shotDown(delayTime.mem, buffer);
    DumpBuffer(buffer, BUFFER_LENGTH);
    sp.write(buffer, BUFFER_LENGTH);
}

// haibot_serial_node "/dev/ttyUSB0" 115200
int main(int argc, char** argv)//argc是命令行总的参数个数
{
    ros::init(argc, argv, "serial_port");
    ros::NodeHandle n;

    if (argc != 3) {
        ROS_ERROR_STREAM("Please check port_id band_rate");
        return -1;
    }
    string portName = argv[1];
    int bandRate = std::atoi(argv[2]);

    //创建一个serial类
    //serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(portName);
    //设置串口通信的波特率
    sp.setBaudrate(bandRate);
    //串口设置timeout
    sp.setTimeout(to);

    ros::Subscriber velSub = n.subscribe("/cmd_vel", 100, CmdVelCallback);
    ros::Subscriber shutDownSub = n.subscribe("/shutdown", 10, ShutDownCallback);
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

    uint8_t voltageReqBuffer[8] = { 0x20, 0x03, 0x00, 0x32, 0x00, 0x01, 0x23, 0x74 };
    uint8_t imuReqBuffer[8] = { 0x20, 0x03, 0x00, 0x38, 0x00, 0x09, 0x02, 0xB0 };
    uint8_t voltageCount = 10;
    uint8_t counterIter = 0;
    ros::Rate loop_rate(50);
    while (ros::ok()) {
        counterIter++;
        if (counterIter >= voltageCount) {
            counterIter =0;
            sp.write(voltageReqBuffer, 8);
            uint8_t voltageffer[1024] = { 0 };
            sp.read(voltageffer, 20);
        }
        //获取缓冲区内的字节数
        //size_t n = sp.available();
        //if (n != 0) {
        //    
        //    //读出数据
        //    n = sp.read(buffer, n);

        //    for (int i = 0; i < n; i++) {
                //16进制的方式打印到屏幕
        //        std::cout << std::hex << (buffer[i] & 0xff) << " ";
        //    }
        //    std::cout << std::endl;
            //把数据发送回去
        sp.write(imuReqBuffer, 8);
        uint8_t imubuffer[1024] = { 0 };
        sp.read(imubuffer, 20);
       // }
        loop_rate.sleep();
        ros::spinOnce();
    }

    //关闭串口
    sp.close();
    ROS_INFO_STREAM("serial port is closed!!!");
    return 0;
}

