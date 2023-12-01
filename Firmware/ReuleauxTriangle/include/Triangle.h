#pragma once

#include "Esp32.h"
#include "IMU.h"
#include "BLDC.h"

static uint32_t timer;

class Triangle {

public:
    void mianloop () {
        bldc.motor.loopFOC();  //foc循环用来控制电机运动

        imu.readIMUData(); // 读取陀螺仪数据

        double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
        timer = micros();

        // 卡尔曼滤波处理后的角度变化值
        double pitch = imu.acc2rotation(imu.accX, imu.accY);
        // 转化为陀螺仪角速度
        double gyroZrate = imu.gyroZ / 131.0;  // Convert to deg/s

        // 卡尔曼滤波后的角度
        imu.kalAngleZ = imu.kalmanZ.getAngle(pitch, gyroZrate + gyroZ_OFF, dt);
        // 陀螺仪的变化角度
        imu.gyroZangle += (gyroZrate + gyroZ_OFF) * dt;

        // 当陀螺仪漂移过大时，重置陀螺仪角度
        if (imu.gyroZangle < -180 || imu.gyroZangle > 180)
            imu.gyroZangle = imu.kalAngleZ;

        // 计算摆动角度
        float pendulum_angle = constrainAngle(fmod(imu.kalAngleZ, 120) - target_angle);

        // 如果预期摆渡角度大于阈值
        if (abs(pendulum_angle) > swing_up_angle)
        {   // 为电机设置swing_up_voltage以便向上摆动
            bldc.motor.controller = MotionControlType::torque;
            target_voltage = -_sign(gyroZrate) * swing_up_voltage;
            bldc.motor.move(target_voltage);
        }
    }

    // function constraining the angle in between -60~60
    float constrainAngle(float x) {
        float a = 0;
        if (x < 0) {
            a = 120 + x;
            if (a < abs(x))
                return a;
        }
        return x;
    }
public:
    Esp32 esp32;
    IMU imu;
    BLDC bldc;

private:
    float gyroZ_OFF = -0.19;
    float target_angle = 89.5;     //平衡角度 例如TA89.3 设置平衡角度89.3
    float swing_up_angle = 18;     //摇摆角度 离平衡角度还有几度时候，切换到自平衡控制
    float target_voltage = 0;      //目标电压
    float swing_up_voltage = 1.5;  //摇摆电压 左右摇摆的电压，越大越快到平衡态，但是过大会翻过头
};
