#ifndef MAIN_H
#define MAIN_H
#include "zf_common_headfile.h"
#include "img_process.h"

// 编码器引脚信息
// 请确保两轮编码器前进时回传值都为正数，若回传为负请isr.c中encoder_get_count前修改正负号
// 左轮编码器
// 左编码器引脚定义
#define ENCODER_LEFT (QTIMER2_ENCODER1)
#define ENCODER_LEFT_A (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_LEFT_B (QTIMER2_ENCODER1_CH2_C4)

// 右编码器引脚定义
#define ENCODER_RIGHT (QTIMER2_ENCODER2)
#define ENCODER_RIGHT_A (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_RIGHT_B (QTIMER2_ENCODER2_CH2_C25)

// 电机参数设置
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!
// 电机定义
#define MOTOR_LEFT_PWM (PWM2_MODULE2_CHB_C11)
#define MOTOR_LEFT_DIR (C10)
#define MOTOR_RIGHT_PWM (PWM2_MODULE3_CHB_D3)
#define MOTOR_RIGHT_DIR (D2)
// 左电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR_LEFT_FORWARD_DIR_LEVEL (GPIO_LOW)

// 右电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR_RIGHT_FORWARD_DIR_LEVEL (GPIO_LOW)

// 舵机参数设置
// 舵机引脚定义
#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30)                         // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ (50)                                          // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (105)                                        // 定义主板上舵机活动范围(左打方向的极限值) 角度 自行标定
#define SERVO_MOTOR_R_MAX (80)                                         // 定义主板上舵机活动范围(右打方向的极限值) 角度 自行标定
#define SERVO_DIR (SERVO_MOTOR_L_MAX > SERVO_MOTOR_R_MAX ? -1.f : 1.f) // 根据左右duty的大小自动决定舵机方向

// 无线
typedef enum
{
    stop = 0,
    go
} status;

extern uint8 stop_flag ;//停车信号 0停止 1运行


// 以下宏在初步测试例程时不可更改，后面若需更改舵机算法可以自行更改
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif
#endif
