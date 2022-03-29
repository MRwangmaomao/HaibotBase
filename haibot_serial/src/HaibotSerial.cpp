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

void HaibotSerial::getVoltage(float &voltage, uint8_t *ptr)
{
    unsigned short crc_res = crc16BitByBit(ptr, 5);
    if(ptr[5] == crc_res%256 && ptr[6] == crc_res/256) {
        unsigned int tmp = ptr[3];
        voltage = (float)(tmp * 256 + ptr[4]) / 100;
    } else {
        voltage = 9999;
    }
}

bool HaibotSerial::getImu(float (&imu)[9], uint8_t *ptr)
{
    unsigned short crc_res = crc16BitByBit(ptr, 21);
    if(ptr[21] == crc_res%256 && ptr[22] == crc_res/256) {
        char tmp = (char)ptr[3];
        
        imu[0] = (float)((int)(((unsigned int)ptr[3]<<8) | ptr[4])) / 32768 * 9.8 / 2; // ax
        imu[1] = (float)((int)(((unsigned int)ptr[5]<<8) | ptr[6])) / 32768 * 9.8 / 2; // ay
        imu[2] = (float)((int)(((unsigned int)ptr[7]<<8) | ptr[8])) / 32768 * 9.8 / 2; // az

        imu[3] = (float)((int)(((unsigned int)ptr[9]<<8) | ptr[10])) / 32768 / 2000;  // gx
        imu[4] = (float)((int)(((unsigned int)ptr[11]<<8) | ptr[12])) / 32768 / 2000; // gy
        imu[5] = (float)((int)(((unsigned int)ptr[13]<<8) | ptr[14])) / 32768 / 2000; // gz

        // angle
        tmp = ptr[15];
        imu[6] = (float)(tmp * 256 + ptr[16]) / 100;
        if (imu[6] > 565) {
            imu[6] = imu[6] - 656;
        }
        tmp = ptr[17];
        imu[7] = (float)(tmp * 256 + ptr[18]) / 100;
        if (imu[7] > 565) {
            imu[7] = imu[7] - 656;
        }
        tmp = ptr[19];
        imu[8] = (float)(tmp * 256 + ptr[20]) / 100;
        if (imu[8] > 180) {
            imu[8] = imu[8] - 656;
        }
    } else {
        return false;
    }
    return true;
}


void HaibotSerial::setSpeed(uint8_t leftSpeed[2], uint8_t rightSpeed[2], uint8_t *ptr, int ptrLen)
{
    ptr[7] = leftSpeed[1];
    ptr[8] = leftSpeed[0];
    ptr[9] = rightSpeed[1];
    ptr[10] = rightSpeed[0];
    unsigned short crc_res = crc16BitByBit(ptr, 11);
    ptr[11] = crc_res % 256;
    ptr[12] = crc_res / 256;
}

void HaibotSerial::shotDown(uint8_t delay_s[2], uint8_t *ptr)
{
    ptr[4] = delay_s[1];
    ptr[5] = delay_s[0];
    unsigned short crc_res = crc16BitByBit(ptr, 6);
    ptr[6] = crc_res%256;
    ptr[7] = crc_res/256;
}

unsigned short HaibotSerial::crc16BitByBit(uint8_t *ptr, unsigned short len)
{
    const unsigned short polynom = 0xA001;
    uint8_t i;
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
