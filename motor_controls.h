#ifndef __MOTOR_CONTROLS__H__
#define __MOTOR_CONTROLS__H__

#include "stdlib.h"
#include "stdint.h"
#include "Arduino.h"
#include "io_controls.h"
#include "common.h"
#include "serial_print.h"


#define NUMBER_OF_MOTORS               2
#define MOTOR_GEAR_RATIO               1000
#define MOTOR_ENCODER_REV_CYCLE        3



#define MOTOR_CONTROL_PWM_FREQ                 100
#define MOTOR_CONTROL_PWM_RES                    8



typedef enum
{
    MOTOR_CONTROLS_STOP,
    MOTOR_CONTROLS_MOVE_FORWARD,
    MOTOR_CONTROLS_MOVE_REVERSE,
}motor_rotation_ctrl_t;

typedef enum
{
    MOTOR_SPEED_STOP,
    MOTOR_SPEED_10,
    MOTOR_SPEED_20,
    MOTOR_SPEED_30,
    MOTOR_SPEED_40,
    MOTOR_SPEED_50,
    MOTOR_SPEED_MAX
}motor_speed_levels_t;

typedef struct motor_steering_control
{
    motor_rotation_ctrl_t motor_dir_ctrl;
    uint8_t forward_pin;
    uint8_t reverse_pin;
    uint8_t set_speed;
    uint8_t driving_pwm;
    uint8_t channel;
    int8_t speed_hysteresis;
}motor_steering_control_t;

typedef enum
{
    MOTOR_SEL_LEFT,
    MOTOR_SEL_RIGHT,

}motor_sel_t;

typedef enum
{
    CAR_CONTROL_MOVE_FORWARD,
    CAR_CONTROL_MOVE_REVERSE,
    CAR_CONTROL_MOVE_LEFT,
    CAR_CONTROL_MOVE_RIGHT,
}car_control_movement_t;


typedef struct
{
    uint16_t motor_encoder_prev_counter;
    uint16_t motor_encoder_counter;
    uint16_t speed;
}motor_controls_state_t;

bool motor_control_init();
void car_movement_control(car_control_movement_t , uint8_t speed);
void motor_control_move(motor_sel_t sel , motor_rotation_ctrl_t rot_ctrl , uint8_t speed);
void motor_control_move(motor_sel_t sel , motor_rotation_ctrl_t rot_ctrl , motor_speed_levels_t speed);

void motor_control_left(motor_rotation_ctrl_t rot_ctrl );
void motor_control_right(motor_rotation_ctrl_t rot_ctrl);

void motor_encoder_callback(uint8_t motor_sel);
motor_controls_state_t * motor_ctrl_read_sate();
void motor_control_timer_callback_f100ms();
void process_speed_control();


#endif