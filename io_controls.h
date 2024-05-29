#ifndef __IO_CONTROLS__H__
#define __IO_CONTROLS__H__

#include "Arduino.h"


#define MOTOR_LEFT_ENCODER_C1_PIN         17
#define MOTOR_LEFT_ENCODER_C2_PIN         5
#define MOTOR_CONTROL_LEFT_PIN_FWD        22
#define MOTOR_CONTROL_LEFT_PIN_REV        23



#define MOTOR_RIGHT_ENCODER_C1_PIN         16
#define MOTOR_RIGHT_ENCODER_C2_PIN         4
#define MOTOR_CONTROL_RIGHT_PIN_FWD        18
#define MOTOR_CONTROL_RIGHT_PIN_REV        19


// ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
// ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
#define MOTOR_CONTROL_LEFT_MOTOR_CHANNEL       2
#define MOTOR_CONTROL_RIGHT_MOTOR_CHANNEL      3


typedef void (*callback_encoder_fb) (uint8_t);



void init_io_controls();
void motor_encoder_init(callback_encoder_fb);

extern void motor_encoder_callback(uint8_t motor_sel);

#endif