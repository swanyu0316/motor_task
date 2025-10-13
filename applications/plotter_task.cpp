#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"

extern sp::DBus remote;
extern sp::RM_Motor motor_3508_1;
extern sp::RM_Motor motor_3508_2;
extern sp::RM_Motor motor_3508_3;
extern sp::RM_Motor motor_3508_4;
extern sp::SuperCap supercap;
extern sp::PM02 pm02;

extern float P_in;
extern float P_actual;
extern float K_tau;
extern float dbg_omega_RL;
extern float dbg_current4;

sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
#define TEST_MOTOR_ID 1

  while (true) {
#if TEST_MOTOR_ID == 1
    //plotter.plot(remote.ch_lh, motor_3508_1.speed);
    plotter.plot(P_in, P_actual, K_tau);
    //plotter.plot(dbg_omega_RL, motor_3508_4.speed, dbg_current4);
#elif TEST_MOTOR_ID == 2
    plotter.plot(remote.ch_lh, motor_3508_2.speed);
#elif TEST_MOTOR_ID == 3
    plotter.plot(remote.ch_lh, motor_3508_3.speed);
#elif TEST_MOTOR_ID == 4
    plotter.plot(remote.ch_lh, motor_3508_4.speed);
#endif
    osDelay(1);
  }
}
