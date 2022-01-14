//
// Created by wpr on 2021/12/6.
//

#ifndef HAIBOT_SERIAL_HAIBOTSERIAL_H
#define HAIBOT_SERIAL_HAIBOTSERIAL_H

class HaibotSerial
{
public:
    HaibotSerial();
    ~HaibotSerial();
    void getVoltage(float &voltage, unsigned char *ptr);
    void getImu(float (&imu)[9], unsigned char *ptr);
    void setSpeed(unsigned char left_speed, unsigned char right_speed, unsigned char *ptr);
    void shotDown(unsigned char delay_s, unsigned char *ptr);
private:
    unsigned short crc16BitByBit(unsigned char *ptr, unsigned short len);

};

#endif //HAIBOT_SERIAL_HAIBOTSERIAL_H
