// #include <iostream>
// #include <thread>
// #include <atomic>
// #include <sys/time.h>
// #include "eu_planet.h"//motors

#include "ymbot_joint_eu.h"

using namespace std;

YmbotJointEu::YmbotJointEu() {
    dev_index = 0;
    motor_id = 0;
    motor_mode = 1;
    flag_enable = false;

    rated_torque = 0.01;

    present_position = 0.0;
    target_position = 0.0;
    record_position = 0.0;


    present_velocity = 0.0;
    target_velocity = 0.0;

    present_current = 0.0;
    target_current = 0.0;

    joint_offset_angle = 0.0;
    joint_offset_radian = 0.0;
    present_joint_radian = 0.0;
    target_joint_radian = 0.0;

    joint_limit_max = 0.0;
    joint_limit_min = 0.0;
}

bool YmbotJointEu::motor_initialization_CSP() {
    if (PLANET_SUCCESS == planet_setEnabled(dev_index, motor_id, true)) {
        cout << "Motor " << motor_id << " enabled successfully" << endl;
        motor_mode = 5;
        planet_setMode(dev_index, motor_id, motor_mode);
        planet_setTargetCurrent(dev_index, motor_id, 500);
        planet_setTargetVelocity(dev_index, motor_id, 10);

        if (PLANET_SUCCESS != planet_getPosition(dev_index, motor_id, &present_position)) {
            cout << "Motor " << motor_id << " get position failed" << endl;
            return false;
        };

        record_position = present_position;
        planet_quick_setTargetPosition(dev_index, motor_id, present_position + 0.1);
        this_thread::sleep_for(chrono::milliseconds(20));

        if (PLANET_SUCCESS != planet_getEnabled(dev_index, motor_id, &flag_enable)) {
            cout << "Motor " << motor_id << " get (enabled flag) failed" << endl;
            return false;
        };

        if (flag_enable) {
            cout << "Motor " << motor_id << " initialize successfully" << endl;  
            // target_current = 1600;
            // target_velocity = 60; 
            target_current = 2000; //1600;  //测试版
            target_velocity = 79; //70;
            planet_setTargetCurrent(dev_index, motor_id, target_current); 
            planet_setTargetVelocity(dev_index, motor_id, target_velocity); 
            //wang junlei edit@2025.9.19 肩肘部关节PI值设置，电流环KI置0时重新上电后电机参数可能重新变成初始值：(位置环P 128，速度环P 2048、I 128，电流环P 800 I 70)
            if (motor_id == 31 || motor_id == 32 || motor_id == 33 || motor_id == 34 || motor_id == 41 || motor_id == 42 || motor_id == 43 || motor_id == 44) 
            {
                planet_setIOfVelocityLoop(dev_index, motor_id, 10); //速度环KI置10,过低会超调，过高稳态误差会相对较大
                planet_setPOfCurrentLoop(dev_index, motor_id, 500); //电流环KP置500,低于400会响应较慢
                planet_setIOfCurrentLoop(dev_index, motor_id, 0); //电流环KI置0,防止积分稳态效果导致电流环震颤电机抖动
            }

            cout << "Motor " << motor_id << " current: " << target_current << " velocity: " << target_velocity << endl;
            return true;
        }
        else {
            return false;
        }
    }
    else {
        cout << "Motor " << motor_id << " enabled failed" << endl;
        cout << "Motor " << motor_id << " initialize failed" << endl;
        return false;
    }
}


bool YmbotJointEu::motor_initialization_PV() {
    if (PLANET_SUCCESS == planet_setEnabled(dev_index, motor_id, true)) {
        cout << "Motor " << motor_id << " enabled successfully" << endl;
        motor_mode = 3;
        planet_setMode(dev_index, motor_id, motor_mode);
        return true;
    }
    else {
        cout << "Motor " << motor_id << " enabled failed" << endl;
        return false;
    }
}


float YmbotJointEu::comput_current(double torque) {
    return torque / rated_torque * 1000;
}

void YmbotJointEu::set_zero_current() {
    planet_setTargetCurrent(dev_index, motor_id, 0);
}

bool YmbotJointEu::motor_disabled() {
    if (PLANET_SUCCESS == planet_setEnabled(dev_index, motor_id, false)) {
        cout << "Motor " << motor_id << " disabled successfully" << endl;
        return true;
    }
    else {
        cout << "Motor " << motor_id << " disabled failed" << endl;
        return false;
    }
}

