/*
 * mode_airship.cpp
 *
 *  Created on: Jan 28, 2024
 *      Author: jacky
 */
#include "maincontroller.h"

#define SERVO_MID 1500	//旋翼舵机中位对应的脉宽值,单位：us,需要自己调整,一般中位是1500us
#define SERVO_PI_4  500 //旋翼倾转45度对应的脉宽变化,单位：us,需要自己调整
static float target_yaw=0.0f;
static float roll_thrust;                // 滚转输出, +/- 1.0
static float pitch_thrust;               // 俯仰输出, +/- 1.0
static float yaw_thrust;                 // 偏航输出, +/- 1.0
static float throttle_thrust;            // 油门输出, 0.0 - 1.0
static float _thrust_left=0.0f,_thrust_right=0.0f,_tilt_left=0.0f,_tilt_right=0.0f;//最终给到电机和舵机的输出
static float thrust_max;                 // 最大电机输出
static float thr_adj = 0.0f;             // 油门输出调整

bool mode_airship_init(void){
	// if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
	if (motors->get_armed() && ap->land_complete && !has_manual_throttle() &&
			(get_pilot_desired_throttle(get_channel_throttle(), 0.0f) > get_non_takeoff_throttle())) {
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	// set target altitude to zero for reporting
	pos_control->set_alt_target(0);
	set_manual_throttle(true);//设置为手动油门
	target_yaw=ahrs_yaw_deg();
	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode airship!\n");
	return true;
}

void mode_airship(void){
	// if not armed exit immediately
	if (!get_soft_armed()) {
		robot_state=STATE_STOP;
		Motor_Set_Value(1, PWM_ESC_MIN);//左旋翼
		Motor_Set_Value(5, PWM_ESC_MIN);//右旋翼
		Servo_Set_Value(1,SERVO_MID);	//左舵机，注意自己调整舵机中位！
		Servo_Set_Value(3,SERVO_MID);	//右舵机，注意自己调整舵机中位！
		return;
	}
	robot_state=STATE_FLYING;

	// get pilot's desired throttle
	throttle_thrust=get_channel_throttle();
	roll_thrust=get_channel_roll();
	pitch_thrust=get_channel_pitch();
	yaw_thrust=get_channel_yaw();

	//安全机制，限制控制输出
	if (throttle_thrust <= 0.01f) {
		throttle_thrust = 0.0f;
	}

	//油门和滚转的混空
	_thrust_left  = throttle_thrust + yaw_thrust * 0.5f;
	_thrust_right = throttle_thrust - yaw_thrust * 0.5f;

	//姿态控制优先级高于油门控制
	thrust_max = MAX(_thrust_right,_thrust_left);
	if (thrust_max > 1.0f) {
		thr_adj = 1.0f - thrust_max;
	}

	//调整平均油门
	_thrust_left  = constrain_float(_thrust_left  + thr_adj, 0.0f, 1.0f);
	_thrust_right = constrain_float(_thrust_right + thr_adj, 0.0f, 1.0f);

	//俯仰和偏航的混控
	_tilt_left  = pitch_thrust - roll_thrust;
	_tilt_right = pitch_thrust + roll_thrust;

	Motor_Set_Value(1, (uint16_t)((float)PWM_ESC_MIN+(float)(PWM_ESC_MAX-PWM_ESC_MIN)*_thrust_left));//左旋翼
	Motor_Set_Value(5, (uint16_t)((float)PWM_ESC_MIN+(float)(PWM_ESC_MAX-PWM_ESC_MIN)*_thrust_right));//右旋翼
	Servo_Set_Value(1,(uint16_t)(SERVO_MID+_tilt_left*SERVO_PI_4));	//左舵机，注意自己调整正负号！
	Servo_Set_Value(3,(uint16_t)(SERVO_MID-_tilt_right*SERVO_PI_4));//右舵机，注意自己调整正负号！

}
