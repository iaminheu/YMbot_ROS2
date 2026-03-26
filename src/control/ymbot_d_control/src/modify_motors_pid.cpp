#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "ymbot_hardware_driver/ymbot_joint_eu.h"

// int id_array[] = {35, 36, 37, 45, 46, 47}; // 躯干和脖子6个自由度

int id_array[] = {31, 32, 33, 34, 41, 42, 43, 44}; // 躯干和脖子6个自由度

// int id_array[] = {31, 32, 33, 34}; // 躯干和脖子6个自由度
// int id_array[] = {41, 42, 43, 44}; // 躯干和脖子6个自由度

using namespace std;

int MotorFunction() {
  /**************************************motors****************************************/
  size_t n_motors = sizeof(id_array) / sizeof(id_array[0]);
  vector<YmbotJointEu> motor(n_motors);
  vector<float> joints_offset_angle(n_motors);

  int channel = 0;
  for (int devIndex = 0; devIndex < 3; devIndex++) {
    if (PLANET_SUCCESS != planet_initDLL(planet_DeviceType_Canable, devIndex,
                                         channel, planet_Baudrate_1000)) {
      cout << "Canable " << devIndex
           << " communication initialization failed !!!" << endl;
      return 0;
    }
  }

  cout << n_motors << endl;

  for (size_t i = 0; i < n_motors; i++) {
    if (id_array[i] > 10 && id_array[i] < 30) {
      motor[i].dev_index = 1;   // 换接线前是1  2025.2.12
    } else if (id_array[i] > 30 && id_array[i] < 40) {
      motor[i].dev_index = 2;   // 换接线前是0  2025.2.12
    } else if (id_array[i] > 40 && id_array[i] < 50) {
      motor[i].dev_index = 0;   // 换接线前是2  2025.2.12
    } else {
      cout << "There is an id number in the id_array that does not match the "
              "actual motor."
           << endl;
      return 0;
    }
    motor[i].motor_id = id_array[i];
    // motor[i].joint_offset_angle = joints_offset_angle[i];
    // motor[i].joint_offset_radian = motor[i].joint_offset_angle / 180.0 * M_PI;
  }

  // get position
  float position = 0;
  unsigned int p_position = 0;
  unsigned int p_velocity = 0;
  unsigned int i_velocity = 0;
  unsigned int p_current = 0;
  unsigned int i_current = 0;

  for (size_t i = 0; i < n_motors; i++) {
      // get
      planet_getPosition(motor[i].dev_index, motor[i].motor_id, &position);
      cout << "motor id: " << motor[i].motor_id << "\t position: " << position << endl; 

      planet_getPOfPositionLoop(motor[i].dev_index, motor[i].motor_id, &p_position);
      cout << "motor id: " << motor[i].motor_id << "\t p of position loop: " << p_position << endl; 

      planet_getPOfVelocityLoop(motor[i].dev_index, motor[i].motor_id, &p_velocity);
      cout << "motor id: " << motor[i].motor_id << "\t p of velocity loop: " << p_velocity << endl; 

      planet_getIOfVelocityLoop(motor[i].dev_index, motor[i].motor_id, &i_velocity);
      cout << "motor id: " << motor[i].motor_id << "\t i of velocity loop: " << i_velocity << endl; 

      planet_getPOfCurrentLoop(motor[i].dev_index, motor[i].motor_id, &p_current);
      cout << "motor id: " << motor[i].motor_id << "\t p of current loop: " << p_current << endl; 

      planet_getIOfCurrentLoop(motor[i].dev_index, motor[i].motor_id, &i_current);
      cout << "motor id: " << motor[i].motor_id << "\t i of current loop: " << i_current << endl; 

      // set
      // p_position = 64;
      // planet_setPOfPositionLoop(motor[i].dev_index, motor[i].motor_id, p_position);

      // p_velocity = 2000;
      // planet_setPOfVelocityLoop(motor[i].dev_index, motor[i].motor_id, p_velocity);

      // i_velocity = 100;
      // planet_setIOfVelocityLoop(motor[i].dev_index, motor[i].motor_id, i_velocity);


      //  need be add after change the any PID paramters
      // planet_saveParas(motor[i].dev_index, motor[i].motor_id);

      cout << endl;
  }


  planet_freeDLL(0);
  planet_freeDLL(1);
  planet_freeDLL(2);

  return 0;
}



int main(int /*argc*/, char ** /*argv*/) {

  MotorFunction();

  return 0;
}