#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class BLDC {

public:
    BLDC() {
        //供电电压设置 [V]
        driver.voltage_power_supply = 12;
        driver.init();
        //连接电机和driver对象
        motor.linkDriver(&driver);
        //FOC模型选择
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
        //运动控制模式设置
        motor.controller = MotionControlType::torque;
        //速度PI环设置
        motor.PID_velocity.P = v_p_1;
        motor.PID_velocity.I = v_i_1;

        //最大电机限制电机
        motor.voltage_limit = 12;
        //速度低通滤波时间常数
        motor.LPF_velocity.Tf = 0.01;
        //设置最大速度限制
        motor.velocity_limit = 40;
        motor.useMonitoring(Serial);
        //初始化电机
        motor.init();

        //初始化 FOC
        Preferences prefs;     // 声明Preferences对象
        prefs.begin("motor");  // 打开命名空间mynamespace
        if (motor.initFOC())  //如果初始化成功，写入offset
        {
            Serial.println(motor.zero_electric_angle);
            prefs.putFloat("offset", motor.zero_electric_angle);
        }
        prefs.end();  // 关闭当前命名空间

        Serial.println(F("Motor ready."));
        Serial.println(F("Set the target velocity using serial terminal:"));
    }

    BLDCMotor motor = BLDCMotor(7);
private:
    BLDCDriver3PWM driver = BLDCDriver3PWM(12, 14, 27); // U V W

private:
    float v_i_1 = 15;              //非稳态速度环I
    float v_p_1 = 0.25;            //非稳态速度环P
    float v_i_2 = 10;              //稳态速度环I
    float v_p_2 = 0.1;             //稳态速度环P
};
