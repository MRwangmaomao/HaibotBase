#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/bind.hpp>
#include <thread>
#include <chrono>

#include <geometry_msgs/Twist.h>
#include <haibot_serial/Shutdown.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

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
    uint8_t buffer_get[200];
    union UnionData speedLeft;
    speedLeft.data = static_cast<int16_t>(100 * (msg->linear.x + msg->angular.z * 0.1));
    union UnionData speedRight;
    speedRight.data = static_cast<int16_t>(-100 * (msg->linear.x - msg->angular.z * 0.1));
    //ROS_INFO_STREAM("x_linear_spead is:  " << msg->linear.x << ",       z_angular is:  " << msg->angular.z);     
    bool success_post = false;
    serialObj.setSpeed(speedLeft.mem, speedRight.mem, buffer, BUFFER_LENGTH);
    int counter_i = 0;
    while (!success_post) {
        counter_i++;
        sp.write(buffer, BUFFER_LENGTH);
        usleep(15000);
        size_t n = sp.available();
        if (n != 0) {
            success_post = true;
            n = sp.read(buffer_get, n);
            // DumpBuffer(buffer_get, n);
        } else {
            std::cout << "Data not get! Try Again! " << std::endl;
            sp.write(buffer, BUFFER_LENGTH);
            usleep(15000);
        }
        if (counter_i > 3) {
            break;
        }
    }
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
    HaibotSerial serialObj = HaibotSerial();
    int loop_cycle_ms = 1;
    int loop_control_hz = 1000 / loop_cycle_ms;
    int loop_imu_pub_ms = 100;
    int loop_odom_wheel_pub_ms = 200;
    int loop_voltage_pub_ms = 3000;
    float voltage_val = 0.0;

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

    ros::Subscriber velSub = n.subscribe("/cmd_vel", 50, CmdVelCallback);
    ros::Subscriber shutDownSub = n.subscribe("/shutdown", 10, ShutDownCallback);
    ros::Publisher voltage_pub = n.advertise<sensor_msgs::BatteryState>("/battery", 10);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 30);
    ros::Publisher odom_wheel_pub = n.advertise<nav_msgs::Odometry>("/odom_wheel", 30);

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

    int counter_i = 0;
    ros::Rate loop_rate(loop_control_hz);
    while (ros::ok()) {
        counter_i++;
        if (counter_i%(loop_voltage_pub_ms/loop_cycle_ms) == 0) {
            // publish voltage info
            uint8_t buffer_pub_vol[8] = {0x20, 0x03, 0x00, 0x32, 0x00, 0x01, 0x23, 0x74};
            sp.write(buffer_pub_vol, 8);
            usleep(10000);
            size_t n = sp.available();
            if (n != 0) {
                uint8_t buffer_get_vol[100] = {0};
                n = sp.read(buffer_get_vol, n);
                serialObj.getVoltage(voltage_val, buffer_get_vol);
                if (voltage_val < 100) {
                    sensor_msgs::BatteryState vol_msg;
                    vol_msg.header.stamp = ros::Time::now();
                    vol_msg.charge = 24;
                    vol_msg.cell_voltage = {3.7, 3.7};
                    vol_msg.header.frame_id = "base_link";
                    vol_msg.location = "China";
                    vol_msg.power_supply_status = 1;
                    vol_msg.power_supply_health = 1;
                    vol_msg.capacity = 29.0;
                    vol_msg.design_capacity = 24.0;
                    vol_msg.voltage = voltage_val;
                    vol_msg.present = true;
                    vol_msg.percentage= vol_msg.design_capacity / vol_msg.capacity;
                    // ROS_INFO_STREAM("Battery voltage is:  " << voltage_val);
                    // DumpBuffer(buffer_get_vol, n);
                    voltage_pub.publish(vol_msg);
                }
            }
        }

        if (counter_i%(loop_odom_wheel_pub_ms/loop_cycle_ms) == 0) {
            uint8_t buffer_pub_odom_wheel[8] = {0x20, 0x03, 0x00, 0x34, 0x00, 0x02, 0x83, 0x74};
            sp.write(buffer_pub_odom_wheel, 8);
            usleep(10000);
            size_t n = sp.available();
            if (n != 0) {
                uint8_t buffer_get_odom_wheel[100] = {0};
                n = sp.read(buffer_get_odom_wheel, n);
                // DumpBuffer(buffer_get_odom_wheel, n);
                nav_msgs::Odometry odom_wheel_msg;
                odom_wheel_msg.header.stamp = ros::Time::now();
                odom_wheel_msg.header.frame_id = "odom";
                odom_wheel_msg.twist.twist.linear.x = 0;
                odom_wheel_msg.twist.twist.linear.y = 0;
                odom_wheel_msg.twist.twist.linear.z = 0;
                odom_wheel_msg.twist.twist.angular.x = 0;
                odom_wheel_msg.twist.twist.angular.y = 0;
                odom_wheel_msg.twist.twist.angular.z = 0;
                odom_wheel_msg.pose.pose.position.x = 0;
                odom_wheel_msg.pose.pose.position.y = 0;
                odom_wheel_msg.pose.pose.position.z = 0;
                odom_wheel_msg.pose.pose.orientation.x = 0;
                odom_wheel_msg.pose.pose.orientation.y = 0;
                odom_wheel_msg.pose.pose.orientation.z = 0;
                odom_wheel_msg.pose.pose.orientation.w = 0;
                odom_wheel_pub.publish(odom_wheel_msg);
            }
        }

        if (counter_i%(loop_imu_pub_ms/loop_cycle_ms) == 0) {
            // publish imu info
            uint8_t buffer_pub_imu[8] = {0x20, 0x03, 0x00, 0x38, 0x00, 0x09, 0x02, 0xB0};
            sp.write(buffer_pub_imu, 8);
            usleep(10000);
            size_t n = sp.available();
            if (n != 0) {
                uint8_t buffer_get_imu[1024];
                float imu_buffer[9];
                n = sp.read(buffer_get_imu, n);
                if (serialObj.getImu(imu_buffer, buffer_get_imu)) {
                    sensor_msgs::Imu imu_msg;
                    // ROS_INFO_STREAM(" ax: " << imu_buffer[0] << ",  ay: " << imu_buffer[1] << ",  az: " << imu_buffer[2]
                    //     << ",  gx: " << imu_buffer[3] << ",  gy: " << imu_buffer[4] << ",  gz: " << imu_buffer[5]
                    //     << ",  pitch: " << imu_buffer[6] << ",  roll: " << imu_buffer[7] << ",  yaw: " << imu_buffer[8]);
                    // DumpBuffer(buffer_get_imu, n);
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = "imu_link";
                    imu_msg.linear_acceleration.x = imu_buffer[0];
                    imu_msg.linear_acceleration.y = imu_buffer[1];
                    imu_msg.linear_acceleration.z = imu_buffer[2];
                    
                    imu_msg.angular_velocity.x = imu_buffer[3];
                    imu_msg.angular_velocity.y = imu_buffer[4];
                    imu_msg.angular_velocity.z = imu_buffer[5];

                    imu_msg.orientation.w = imu_buffer[6];
                    imu_msg.orientation.x = imu_buffer[7];
                    imu_msg.orientation.y = imu_buffer[8];
                    imu_msg.orientation.z = imu_buffer[8];
                    imu_pub.publish(imu_msg);
                }
            }
        }

        if (counter_i >= 30000) {
            counter_i == 0;
        }
    
        loop_rate.sleep();
        ros::spinOnce();
    }

    //关闭串口
    sp.close();
    ROS_INFO_STREAM("serial port is closed!!!");
    return 0;
}

