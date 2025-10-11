#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/pid/pid.hpp"

// 实例化PM02初始化  C板通信端口，若使用达妙改为 &huart1, false
sp::PM02 pm02(&huart6);

//实例化一个遥控器
sp::DBus remote(&huart3);

//实例化can2
sp::CAN can2(&hcan2);

//实例化supercap
sp::SuperCap supercap(sp::SuperCapMode::AUTOMODE);

//实例化四个id分别为1、2、3、4的3508电机
sp::RM_Motor motor_3508_1(1, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_2(2, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_3(3, sp::RM_Motors::M3508);
sp::RM_Motor motor_3508_4(4, sp::RM_Motors::M3508);

// dt: 控制周期，1ms  Kp: 比例系数（反应速度） Ki: 积分系数（消除静差） Kd: 微分系数（抑制震荡） 输出上限 输出下限 alpha: 滤波系数 是否角度环（false表示线性速度环）是否动态更新
// 速度环 PID

// PID 参数

float deadzone = 0.05f;

float kp_speed = 0.015f;
float ki_speed = 0.0f;
float kd_speed = 0.009f;

//PID
sp::PID motor1_pid_speed(kp_speed, ki_speed, kd_speed, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor2_pid_speed(kp_speed, ki_speed, kd_speed, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor3_pid_speed(kp_speed, ki_speed, kd_speed, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);
sp::PID motor4_pid_speed(kp_speed, ki_speed, kd_speed, 0.0f, 10000.0f, 5000.0f, 1.0f, false, true);

extern "C" void control_task()
{
  //接收遥控器消息
  remote.request();

  // 向裁判串口请求数据
  pm02.request();

  //can2初始化配置
  can2.config();
  can2.start();

  //ps：其他大多数电机cmd的均为扭矩值
  while (true) {
    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        //电容控制策略 capacitor 自动模式
        break;

      case sp::DBusSwitchMode::MID: {
        // 摇杆输入
        float lx = remote.ch_lh;  // 左摇杆左右
        float ly = remote.ch_lv;  // 左摇杆前后
        float rx = remote.ch_rh;  // 右摇杆左右
        float ry = remote.ch_rv;  // 右摇杆前后

        // 将摇杆输入转化为速度指令
        float vx = 0.0f;  // 前后速度指令
        float vy = 0.0f;  // 左右速度指令
        float wz = 0.0f;  // 旋转角速度
        const float wz_fixed = 2.0f;

        // 右摇杆偏离中心，底盘就以固定角速度旋转
        if (fabsf(rx) > deadzone || fabsf(ry) > deadzone) {
          if (rx < 0.0f || ry < 0.0f)
            wz = -wz_fixed;  // 向左旋转
          else
            wz = wz_fixed;  // 向右旋转
        }
        else {
          wz = 0.0f;
        }

        // 左摇杆偏离中心，底盘直线运动
        if (fabsf(lx) > deadzone || fabsf(ly) > deadzone) {
          vx = ly * 5.0f;
          vy = lx * 5.0f;
        }
        else {
          vx = 0.0f;
          vy = 0.0f;
        }

        //麥輪底盤解算
        const float r = 0.077f;                  // 轮半径 m
        const float L_plus_W = 0.165f + 0.185f;  // 纵向+横向到中心 m
        const float gear_ratio = 14.9f;          // 电机减速比

        float omega_FL = (1.0f / r) * (vx - vy - L_plus_W * wz) * gear_ratio;
        float omega_FR = (1.0f / r) * (vx + vy + L_plus_W * wz) * gear_ratio;
        float omega_RL = (1.0f / r) * (vx + vy - L_plus_W * wz) * gear_ratio;
        float omega_RR = (1.0f / r) * (vx - vy + L_plus_W * wz) * gear_ratio;

        // PID
        motor1_pid_speed.calc(omega_FL, motor_3508_1.speed);
        motor2_pid_speed.calc(omega_FR, motor_3508_2.speed);
        motor3_pid_speed.calc(omega_RR, motor_3508_3.speed);
        motor4_pid_speed.calc(omega_RL, motor_3508_4.speed);

        // 读取电容状态
        supercap.read(can2.rx_data, osKernelSysTick());
        float P_actual = supercap.power_out;

        // 期望转矩
        float tau1 = motor1_pid_speed.out;
        float tau2 = motor2_pid_speed.out;
        float tau3 = motor3_pid_speed.out;
        float tau4 = motor4_pid_speed.out;

        // 获取轮速
        float omega1 = motor_3508_1.speed;
        float omega2 = motor_3508_2.speed;
        float omega3 = motor_3508_3.speed;
        float omega4 = motor_3508_4.speed;

        // 预测功率计算参数
        const float K1 = 0.0005f;  // 转矩引起的能量损耗
        const float K2 = 0.0002f;  // 角速度引起的能量损耗
        const float K3 = 20.0f;    // 待机功耗

        // 预测功率计算
        float sum_tau_omega = tau1 * omega1 + tau2 * omega2 + tau3 * omega3 + tau4 * omega4;
        float sum_tau2 = tau1 * tau1 + tau2 * tau2 + tau3 * tau3 + tau4 * tau4;
        float sum_omega2 = omega1 * omega1 + omega2 * omega2 + omega3 * omega3 + omega4 * omega4;
        // P_in = Στ·ω + K1 Στ² + K2 Σω² + K3
        float P_in = sum_tau_omega + K1 * sum_tau2 + K2 * sum_omega2 + K3;

        // 获取功率上限
        float P_max = static_cast<float>(pm02.robot_status.chassis_power_limit);
        if (P_max <= 0.0f) P_max = 2000.0f;

        // 计算判别式
        float discriminant =
          sum_tau_omega * sum_tau_omega - 4.0f * K1 * sum_tau2 * (K2 * sum_omega2 + K3 - P_max);

        // 计算转矩缩小系数 K_tau
        float K_tau = 1.0f;
        if (discriminant > 0.0f && sum_tau2 > 1e-6f) {
          K_tau = (-sum_tau_omega + sqrtf(discriminant)) / (2.0f * K1 * sum_tau2);

          // 在0~1之间
          if (K_tau > 1.0f) K_tau = 1.0f;
          if (K_tau < 0.0f) K_tau = 0.0f;
        }

        // 超级电容控制实际功率
        float P_needed_from_cap = P_in - P_max;
        if (P_needed_from_cap > 0.0f) {
          float cap_supply = std::min(P_actual, P_needed_from_cap);  // 电容补充功率
          float P_effective = P_in - cap_supply;
          if (P_effective > P_max) {
            float K_supercap = P_max / P_effective;
            if (K_supercap < K_tau) K_tau = K_supercap;
          }
        }

        // 应用缩小系数到转矩
        tau1 *= K_tau;
        tau2 *= K_tau;
        tau3 *= K_tau;
        tau4 *= K_tau;

        const float RM3508_TORQUE_CONST = 0.3f;  // 扭矩常数0.3N·m/A

        // 转换为电流发送给电机
        float current1 = tau1 / RM3508_TORQUE_CONST;
        float current2 = tau2 / RM3508_TORQUE_CONST;
        float current3 = tau3 / RM3508_TORQUE_CONST;
        float current4 = tau4 / RM3508_TORQUE_CONST;

        motor_3508_1.cmd(current1);
        motor_3508_2.cmd(current2);
        motor_3508_3.cmd(current3);
        motor_3508_4.cmd(current4);

        // 如果在死区的话归零
        if (
          fabs(lx) < deadzone && fabs(ly) < deadzone && fabs(rx) < deadzone &&
          fabs(ry) < deadzone) {
          motor_3508_1.cmd(0);
          motor_3508_2.cmd(0);
          motor_3508_3.cmd(0);
          motor_3508_4.cmd(0);
        }

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
    motor_3508_1.write(can2.tx_data);
    motor_3508_2.write(can2.tx_data);
    motor_3508_3.write(can2.tx_data);
    motor_3508_4.write(can2.tx_data);
    can2.send(motor_3508_1.tx_id);
    can2.send(motor_3508_2.tx_id);
    can2.send(motor_3508_3.tx_id);
    can2.send(motor_3508_4.tx_id);

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
    if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor_3508_1.rx_id)
        motor_3508_1.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == motor_3508_2.rx_id)
        motor_3508_2.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == motor_3508_3.rx_id)
        motor_3508_3.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == motor_3508_4.rx_id)
        motor_3508_4.read(can2.rx_data, stamp_ms);
    }
  }
}