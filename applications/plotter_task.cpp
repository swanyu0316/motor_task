#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

extern sp::DBus remote;
extern sp::RM_Motor motor_6020;
sp::Plotter plotter(&huart1);

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(remote.ch_lh, motor_6020.speed);
    osDelay(1);  // 100Hz
  }
}