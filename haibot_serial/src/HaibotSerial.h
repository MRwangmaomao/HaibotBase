//
// Created by wpr on 2021/12/6.
//

#ifndef HAIBOT_SERIAL_HAIBOTSERIAL_H
#define HAIBOT_SERIAL_HAIBOTSERIAL_H

typedef unsigned char uint8_t;

class HaibotSerial
{
public:
    HaibotSerial();
    ~HaibotSerial();
    void getVoltage(float &voltage, uint8_t *ptr);
    void getImu(float (&imu)[9], uint8_t *ptr);
    void setSpeed(uint8_t leftSpeed[2], uint8_t rightSpeed[2], uint8_t *ptr, int ptrLen);
    void shotDown(uint8_t delay_s[2], uint8_t *ptr);
private:
    unsigned short crc16BitByBit(uint8_t *ptr, unsigned short len);

};

#endif //HAIBOT_SERIAL_HAIBOTSERIAL_H
