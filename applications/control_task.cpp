#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

//实例化一个遥控器
sp::DBus remote(&huart3);

//实例化can1
sp::CAN can1(&hcan1);

//实例化四个id分别为1、2、3、4的电压环（6020_V）6020电机
sp::RM_Motor motor_6020_1(1, sp::RM_Motors::GM6020_V);
sp::RM_Motor motor_6020_2(2, sp::RM_Motors::GM6020_V);
sp::RM_Motor motor_6020_3(3, sp::RM_Motors::GM6020_V);
sp::RM_Motor motor_6020_4(4, sp::RM_Motors::GM6020_V);

extern "C" void control_task()
{
  //接收遥控器消息
  remote.request();

  //can1初始化配置
  can1.config();
  can1.start();

  //ps：其他大多数电机cmd的均为扭矩值
  while (true) {
    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        //电容控制策略
        break;

      case sp::DBusSwitchMode::MID:

        // 摇杆输入归一化（-1 ~ +1）
        float lx = remote.ch_lh / 660.0f;  // 左摇杆左右
        float ly = remote.ch_lv / 660.0f;  // 左摇杆前后
        float rx = remote.ch_rh / 660.0f;  // 右摇杆左右
        float ry = remote.ch_rv / 660.0f;  // 右摇杆前后

        // 将摇杆输入转化为速度指令
        float vy = ly * 5.0f;  // 前后速度指令
        float vx = lx * 5.0f;  // 左右速度指令
        float wz = 0.0f;       // 旋转角速度（暂时为 0）

        // 右摇杆控制旋转
        if (ry > 200)
          wz = -2.0f;  // 向左转
        else if (ry < -200)
          wz = 2.0f;  // 向右转

        // 左摇杆前后控制前后速度
        motor_6020_1.cmd(vy);
        motor_6020_2.cmd(vy);
        motor_6020_3.cmd(vy);
        motor_6020_4.cmd(vy);

        // 计算电压命令（先简单共用一个）

        float voltage_cmd = vx;

        // 限幅，防止过大
        if (voltage_cmd > 10.0f) voltage_cmd = 10.0f;
        if (voltage_cmd < -10.0f) voltage_cmd = -10.0f;

        break;

        //约定右down挡时全部电机失能
      case sp::DBusSwitchMode::DOWN:
        motor_6020_1.cmd(0.0f);
        motor_6020_2.cmd(0.0f);
        motor_6020_3.cmd(0.0f);
        motor_6020_4.cmd(0.0f);
        break;
      default:
        break;
    }
    motor_6020_1.write(can1.tx_data);
    motor_6020_2.write(can1.tx_data);
    motor_6020_3.write(can1.tx_data);
    motor_6020_4.write(can1.tx_data);
    can1.send(motor_6020_1.tx_id);
    can1.send(motor_6020_2.tx_id);
    can1.send(motor_6020_3.tx_id);
    can1.send(motor_6020_4.tx_id);

    osDelay(10);
  }
}

//UART接收中断回调函数
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

//UART错误中断回调函数
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}

// CAN接收中断回调函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_6020_1.rx_id)
        motor_6020_1.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_6020_2.rx_id)
        motor_6020_2.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_6020_3.rx_id)
        motor_6020_3.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_6020_4.rx_id)
        motor_6020_4.read(can1.rx_data, stamp_ms);
    }
  }
}
