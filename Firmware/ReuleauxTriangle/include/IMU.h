#pragma once

#include <Arduino.h>
#include <Kalman.h>
#include "I2CDev.h"


class IMU {

public:
    IMU() {
        // kalman mpu6050 init
        Wire.begin(2, 15, uint32_t(4000000));  // Set I2C frequency to 400kHz
        i2cData[0] = 7;                         // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
        i2cData[1] = 0x00;                      // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
        i2cData[2] = 0x00;                      // Set Gyro Full Scale Range to ±250deg/s
        i2cData[3] = 0x00;                      // Set Accelerometer Full Scale Range to ±2g

        while (IMUi2cWrite(0x19, i2cData, 4, false))
            ;  // Write to all four registers at once
        while (IMUi2cWrite(0x6B, 0x01, true))
            ;  // PLL with X axis gyroscope reference and disable sleep mode
        while (IMUi2cRead(0x75, i2cData, 1))
            ;
        delay(100);  // Wait for sensor to stabilize
        /* Set kalman and gyro starting angle */
        while (IMUi2cRead(0x3B, i2cData, 6))
            ;
        accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
        accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
        accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
        double pitch = acc2rotation(accX, accY);
        kalmanZ.setAngle(pitch);
        gyroZangle = pitch;

        Serial.println("kalman mpu6050 init");
    }

    void readIMUData() {
        // 读取MPU6050数据
        while (IMUi2cRead(0x3B, i2cData, 14))
            ;
        accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
        accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
        accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
        //    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
        gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
        gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
        gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    }

    double accX;
    double accY;
    double gyroZ;
    double kalAngleZ;
    Kalman kalmanZ;
    double gyroZangle;
// Angle calculate using the gyro only
double compAngleZ;
public:
    /* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
    double acc2rotation(double x, double y) {
        double tmp_kalAngleZ = (atan(x / y) / 1.570796 * 90);
        if (y < 0) {
            return (tmp_kalAngleZ + 180);
        } else if (x < 0) {
            //将当前值与前值比较，当前差值大于100则认为异常
            if (!isnan(kalAngleZ) && (tmp_kalAngleZ + 360 - kalAngleZ) > 100) {
//            Serial.print("X<0"); Serial.print("\t");
//            Serial.print(tmp_kalAngleZ); Serial.print("\t");
//            Serial.print(kalAngleZ); Serial.print("\t");
//            Serial.print("\r\n");
                if (tmp_kalAngleZ < 0 && kalAngleZ < 0)  //按键右边角
                    return tmp_kalAngleZ;
                else  //按键边异常处理
                    return tmp_kalAngleZ;
            } else
                return (tmp_kalAngleZ + 360);
        } else {
            return tmp_kalAngleZ;
        }
    }
public:
    uint8_t i2cData[14];  // Buffer for I2C data
private:
    double accZ;

    // Calculated angle using a complementary filter
    // Calculated angle using a Kalman filter

    double gyroX, gyroY;
};