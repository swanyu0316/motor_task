#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/pid/pid.hpp"

// 实例化PM02初始化
sp::PM02 pm02(&huart6);

//实例化一个遥控器
sp::DBus remote(&huart3);

//实例化can2
sp::CAN can2(&hcan2);

//实例化supercap
sp::SuperCap supercap(sp::SuperCapMode::AUTOMODE);

//实例化四个id分别为1、2、3、4的3508电机
sp::RM_Motor motor_3508_1(1, sp::RM_Motors::M3508);  // FL (左前)
sp::RM_Motor motor_3508_2(2, sp::RM_Motors::M3508);  // FR (右前)
sp::RM_Motor motor_3508_3(3, sp::RM_Motors::M3508);  // RR (右后)
sp::RM_Motor motor_3508_4(4, sp::RM_Motors::M3508);  // RL (左后)

// dt: 控制周期，1ms  Kp: 比例系数（反应速度） Ki: 积分系数（消除静差） Kd: 微分系数（抑制震荡） 输出上限 输出下限 alpha: 滤波系数 是否角度环（false表示线性速度环）是否动态更新
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

float P_in = 0.0f;      // 预测输入功率
float P_actual = 0.0f;  // 实际输出功率（从电容读取）
float K_tau = 1.0f;     // 转矩缩小系数

extern "C" void control_task()
{
  //接收遥控器消息
  remote.request();

  // 向裁判串口请求数据
  pm02.request();

  //can2初始化配置
  can2.config();
  can2.start();

  while (true) {
    // 在 switch 外部定义电流变量，以便在循环末尾统一发送
    float current1 = 0.0f;
    float current2 = 0.0f;
    float current3 = 0.0f;
    float current4 = 0.0f;

    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP: {
        //电容控制策略 自动模式 未完待续

        break;
      }

      case sp::DBusSwitchMode::MID: {
        // 摇杆输入
        // 左摇杆用于平移 (vx, vy)
        float lv_input = remote.ch_lv;  // 左摇杆前后  vx (前进/后退)
        float lh_input = remote.ch_lh;  // 左摇杆左右  vy (左移/右移)

        // 右摇杆用于旋转 (wz)
        float rv_input = remote.ch_rv;  // 右摇杆前后 向左转
        float rh_input = remote.ch_rh;  // 右摇杆左右 向右转

        // 将摇杆输入转化为速度指令
        float vx = 0.0f;  // 前后速度指令
        float vy = 0.0f;  // 左右速度指令
        float wz = 0.0f;  // 旋转角速度
        const float wz_fixed = 2.0f;
        const float max_speed = 1.0f;  // 最大平移速度系数

        // 右摇杆偏离中心 地盘旋转
        if (fabsf(rv_input) > deadzone) {
          // 底盘向左转
          wz = wz_fixed;
        }
        else if (fabsf(rh_input) > deadzone) {
          // 底盘向右转
          wz = -wz_fixed;
        }
        else {
          wz = 0.0f;
        }

        // 左摇杆偏离中心，底盘直线运动
        if (fabsf(lv_input) > deadzone || fabsf(lh_input) > deadzone) {
          // 左摇杆前后控制 vx) > deadzone) {
          vx = lv_input * max_speed;
          vy = -lh_input * max_speed;
        }
        else if (wz == 0.0f) {
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
        P_actual = supercap.power_out;

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
        P_in = sum_tau_omega + K1 * sum_tau2 + K2 * sum_omega2 + K3;

        // 获取功率上限
        float P_max = static_cast<float>(pm02.robot_status.chassis_power_limit);
        if (P_max <= 0.0f) P_max = 2000.0f;

        // 计算判别式
        float discriminant =
          sum_tau_omega * sum_tau_omega - 4.0f * K1 * sum_tau2 * (K2 * sum_omega2 + K3 - P_max);

        // 计算转矩缩小系数 K_tau
        K_tau = 1.0f;
        if (discriminant > 0.0f && sum_tau2 > 1e-6f) {
          K_tau = (-sum_tau_omega + sqrtf(discriminant)) / (2.0f * K1 * sum_tau2);
          // 在0~1之间
          if (K_tau > 1.0f) K_tau = 1.0f;
          if (K_tau < 0.0f) K_tau = 0.0f;
        }

        // 超级电容控制实际功率
        supercap.write(
          can2.tx_data,                                      // uint8_t* CAN发送数据缓冲
          pm02.robot_status.chassis_power_limit,             // 底盘功率上限
          pm02.power_heat.buffer_energy,                     // 超级电容剩余能量
          pm02.robot_status.power_management_chassis_output  // 底盘实际输出
        );

        // 应用缩小系数到转矩
        tau1 *= K_tau;
        tau2 *= K_tau;
        tau3 *= K_tau;
        tau4 *= K_tau;

        const float RM3508_TORQUE_CONST = 0.3f;  // 扭矩常数0.3N·m/A

        // 转换为电流发送给电机
        current1 = tau1 / RM3508_TORQUE_CONST;
        current2 = tau2 / RM3508_TORQUE_CONST;
        current3 = tau3 / RM3508_TORQUE_CONST;
        current4 = tau4 / RM3508_TORQUE_CONST;

        // 如果在死区的话归零
        if (
          fabs(lv_input) < deadzone && fabs(lh_input) < deadzone && fabs(rv_input) < deadzone &&
          fabs(rh_input) < deadzone) {
          current1 = 0;
          current2 = 0;
          current3 = 0;
          current4 = 0;
        }

        // 电流限幅
        if (current1 > 10000.0f) current1 = 10000.0f;
        if (current1 < -10000.0f) current1 = -10000.0f;
        if (current2 > 10000.0f) current2 = 10000.0f;
        if (current2 < -10000.0f) current2 = -10000.0f;
        if (current3 > 10000.0f) current3 = 10000.0f;
        if (current3 < -10000.0f) current3 = -10000.0f;
        if (current4 > 10000.0f) current4 = 10000.0f;
        if (current4 < -10000.0f) current4 = -10000.0f;

        break;
      }

        //约定右down挡时全部电机失能
      case sp::DBusSwitchMode::DOWN: {
        current1 = 0.0f;
        current2 = 0.0f;
        current3 = 0.0f;
        current4 = 0.0f;
        break;
      }
      default:
        break;
    }
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
      else if (can2.rx_id == motor_3508_4.rx_id) {
        motor_3508_4.read(can2.rx_data, stamp_ms);
      }
    }
  }
}
