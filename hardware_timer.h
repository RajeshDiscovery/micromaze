#ifndef __HARDWARE_TIMER__H__
#define __HARDWARE_TIMER__H__

#include "Arduino.h"
#include "common.h"
#include "esp32-hal.h"



void hw_timer_setup(uint16_t time_ms);
__attribute__((weak)) void hw_timer_callback();


/**
 * @brief outof scope.
 *
 */
extern void motor_control_timer_callback_f100ms();





#endif