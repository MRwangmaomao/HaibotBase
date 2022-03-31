# Haibot

## 1 使用说明
硬件准备：Haibot移动机器人平台

机器人建图导航快速操作流程：
### step1
接通电源，按下开关

### step2
打开控制台，输入如下命令，可以打开机器人底盘驱动，雷达驱动，相机驱动。
```bash
roslaunch haibot_tools haibot_bringup.launch
```
机器人可以发送雷达点云消息、相机2D图像、深度图像、3D点云消息、移动底盘IMU消息、电池消息、轮子里程计消息等。可以向机器人发送速度消息、关机指令等。
### step3
控制机器人进行cartographer激光建图
```bash
roslaunch cartographer_slam_core haibot_cartographer.launch
```
### step4
控制机器人进行移动
```bash
roslaunch nav move_base.launch
```

其他常用命令：

键盘遥控：
```bash
rosrun haibot_tools keyboard.py
```

机器人画圆
```bash
rosrun haibot_tools circle_run.py
```

## 2 使用PC连接机器人
确保机器人和PC在一个局域网络内，查看机器人的IP地址，比如我们的机器人的IP为192.168.1.105。

在电脑PC端打开xshell或者电脑ubuntu系统的控制台，使用如下命令连接机器人：
```bash
ssh robot@192.168.1.105
```

然后就可以参照上面的步骤来控制机器人启动并完成建图与导航。

## 3 深度学习demo
使用usb相机，接到机器人的usb口上。使用如下命令启动深度学习视频分析任务。
```bash
roslaunch ros_deep_learning detectnet.ros1.launch
```

## 4 循迹denmo
使用usb相机，接到机器人的usb口上。使用如下命令启动小车循迹任务。
```bash
roscd track_line/script
sh bringup.sh
```


## 5 二维码识别demo
使用usb相机，接到机器人的usb口上。使用如下命令启动二维码识别任务。
```bash
roslaunch apriltag_ros detector.launch
```
