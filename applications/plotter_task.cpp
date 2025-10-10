#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;
extern sp::RM_Motor motor_3508_1;
extern sp::RM_Motor motor_3508_2;
extern sp::RM_Motor motor_3508_3;
extern sp::RM_Motor motor_3508_4;

sp::Plotter plotter(&huart3);

extern "C" void plotter_task()
{
#define TEST_MOTOR_ID 1

  while (true) {
#if TEST_MOTOR_ID == 1
    plotter.plot(remote.ch_lh, motor_3508_1.speed);
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