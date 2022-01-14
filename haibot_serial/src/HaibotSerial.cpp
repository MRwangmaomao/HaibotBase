//
// Created by wpr on 2021/12/6.
//

#include "HaibotSerial.h"

HaibotSerial::HaibotSerial()
{

}

HaibotSerial::~HaibotSerial()
{

}

void HaibotSerial::getVoltage(float &voltage, unsigned char *ptr)
{
    unsigned short crc_res = crc16BitByBit(ptr, 5);
    if(ptr[5] == crc_res%256 && ptr[6] == crc_res/256) {
        unsigned int tmp = ptr[3];
        voltage = (float)(tmp * 256 + ptr[4]) / 100;
    } else {
        voltage = 9999;
    }
}

void HaibotSerial::getImu(float (&imu)[9], unsigned char *ptr)
{
    unsigned short crc_res = crc16BitByBit(ptr, 21);
    if(ptr[21] == crc_res%256 && ptr[22] == crc_res/256) {
        unsigned int tmp = ptr[3];
        imu[0] = (float)(tmp * 256 + ptr[4]) / 100;
        tmp = ptr[5];
        imu[2] = (float)(tmp * 256 + ptr[6]) / 100;
        tmp = ptr[7];
        imu[3] = (float)(tmp * 256 + ptr[8]) / 100;
        tmp = ptr[7];
        imu[4] = (float)(tmp * 256 + ptr[10]) / 100;
        tmp = ptr[11];
        imu[5] = (float)(tmp * 256 + ptr[12]) / 100;
        tmp = ptr[13];
        imu[6] = (float)(tmp * 256 + ptr[14]) / 100;
        tmp = ptr[15];
        imu[7] = (float)(tmp * 256 + ptr[16]) / 100;
        tmp = ptr[17];
        imu[8] = (float)(tmp * 256 + ptr[18]) / 100;
        tmp = ptr[19];
        imu[9] = (float)(tmp * 256 + ptr[20]) / 100;
    } else {
        for (float & i : imu) {
            i = 9999;
        }
    }
}


void HaibotSerial::setSpeed(unsigned char left_speed, unsigned char right_speed, unsigned char *ptr)
{
    ptr[8] = left_speed;
    ptr[10] = right_speed;
    unsigned short crc_res = crc16BitByBit(ptr, 11);
    ptr[11] = crc_res%256;
    ptr[12] = crc_res/256;
}

void HaibotSerial::shotDown(unsigned char delay_s, unsigned char *ptr)
{
    ptr[5] = delay_s;
    unsigned short crc_res = crc16BitByBit(ptr, 6);
    ptr[6] = crc_res%256;
    ptr[7] = crc_res/256;
}

unsigned short HaibotSerial::crc16BitByBit(unsigned char *ptr, unsigned short len)
{
    const unsigned short polynom = 0xA001;
    unsigned char i;
    unsigned short crc = 0xffff;

    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= polynom;
            }
            else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return crc;
}