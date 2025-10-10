#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

//实例化一个遥控器
sp::DBus remote(&huart3);

//实例化can1
sp::CAN can1(&hcan1);

//实例化四个id分别为1、2、3、4的电压环（3508_V）3508电机
sp::RM_Motor motor_3508_1(1, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_2(2, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_3(3, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_4(4, sp::RM_Motors::M3508);

// dt: 控制周期，1ms  Kp: 比例系数（反应速度） Ki: 积分系数（消除静差） Kd: 微分系数（抑制震荡） 输出上限 输出下限 alpha: 滤波系数 是否角度环（false表示线性速度环）是否动态更新
// PID
sp::PID motor1_pid(0.001f, 20.0f, 0.5f, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor2_pid(0.001f, 20.0f, 0.5f, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor3_pid(0.001f, 20.0f, 0.5f, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor4_pid(0.001f, 20.0f, 0.5f, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);

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

      case sp::DBusSwitchMode::MID: {
        // 摇杆输入归一化（-1 ~ +1）
        float lx = remote.ch_lh / 660.0f;  // 左摇杆左右
        float ly = remote.ch_lv / 660.0f;  // 左摇杆前后
        float rx = remote.ch_rh / 660.0f;  // 右摇杆左右
        float ry = remote.ch_rv / 660.0f;  // 右摇杆前后

        float vx = 0.0f;  // 前后速度指令
        float vy = 0.0f;  // 左右速度指令
        float wz = 0.0f;  // 旋转角速度
        const float wz_fixed = 2.0f;
        const int threshold = 200;

        // 只要右摇杆偏离中心，底盘就以固定角速度旋转

        if (rx > threshold || rx < -threshold || ry > threshold || ry < -threshold) {
          if (rx < 0 || ry < 0)
            wz = -wz_fixed;  // 向左旋转
          else
            wz = wz_fixed;  // 向右旋转
        }
        else {
          wz = 0;
        }

        // 只要左摇杆偏离中心，底盘就直线运动

        if (abs(remote.ch_lh) > 50 || abs(remote.ch_lv) > 50) {
          vx = ly * 5.0f;
          vy = lx * 5.0f;
        }
        else {
          vx = 0;
          vy = 0;
        }

        // 直行PID
        motor1_pid.calc(vx, motor_3508_1.speed);
        motor2_pid.calc(vx, motor_3508_2.speed);
        motor3_pid.calc(vx, motor_3508_3.speed);
        motor4_pid.calc(vx, motor_3508_4.speed);

        // 将PID输出结果作为扭矩命令
        motor_3508_1.cmd(motor1_pid.out);
        motor_3508_2.cmd(motor2_pid.out);
        motor_3508_3.cmd(motor3_pid.out);
        motor_3508_4.cmd(motor4_pid.out);

        // 旋转PID

        const float r = 0.077f;                  // 轮半径 m
        const float L_plus_W = 0.165f + 0.185f;  // 纵向+横向到中心 m
        const float gear_ratio = 14.9f;          // 电机减速比

        float omega_FL = (1.0f / r) * (vx - vy - L_plus_W * wz) * gear_ratio;
        float omega_FR = (1.0f / r) * (vx + vy + L_plus_W * wz) * gear_ratio;
        float omega_RL = (1.0f / r) * (vx + vy - L_plus_W * wz) * gear_ratio;
        float omega_RR = (1.0f / r) * (vx - vy + L_plus_W * wz) * gear_ratio;

        motor1_pid.calc(omega_FL, motor_3508_1.speed);
        motor2_pid.calc(omega_FR, motor_3508_2.speed);
        motor3_pid.calc(omega_RL, motor_3508_3.speed);
        motor4_pid.calc(omega_RR, motor_3508_4.speed);

        // 将PID输出结果作为扭矩命令

        motor_3508_1.cmd(motor1_pid.out);
        motor_3508_2.cmd(motor2_pid.out);
        motor_3508_3.cmd(motor3_pid.out);
        motor_3508_4.cmd(motor4_pid.out);

        float voltage_cmd = vx;

        // 限幅，防止过大
        if (voltage_cmd > 10.0f) voltage_cmd = 10.0f;
        if (voltage_cmd < -10.0f) voltage_cmd = -10.0f;

        break;
      }

        //约定右down挡时全部电机失能
      case sp::DBusSwitchMode::DOWN: {
        motor_3508_1.cmd(0.0f);
        motor_3508_2.cmd(0.0f);
        motor_3508_3.cmd(0.0f);
        motor_3508_4.cmd(0.0f);
        break;
      }
      default:
        break;
    }

    // 统一发送
    motor_3508_1.write(can1.tx_data);
    motor_3508_2.write(can1.tx_data);
    motor_3508_3.write(can1.tx_data);
    motor_3508_4.write(can1.tx_data);
    can1.send(motor_3508_1.tx_id);
    can1.send(motor_3508_2.tx_id);
    can1.send(motor_3508_3.tx_id);
    can1.send(motor_3508_4.tx_id);

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

      if (can1.rx_id == motor_3508_1.rx_id)
        motor_3508_1.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_3508_2.rx_id)
        motor_3508_2.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_3508_3.rx_id)
        motor_3508_3.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_3508_4.rx_id)
        motor_3508_4.read(can1.rx_data, stamp_ms);
    }
  }
}
