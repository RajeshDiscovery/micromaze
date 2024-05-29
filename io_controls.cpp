
#include "io_controls.h"


void IRAM_ATTR motor_encoder_left_interrupt();
void IRAM_ATTR motor_encoder_right_interrupt();

// static callback_encoder_fb encoder_callback;

void init_io_controls()
{
    /**
     * Motor controls left.
     */
    pinMode(MOTOR_CONTROL_LEFT_PIN_FWD, OUTPUT);
    pinMode(MOTOR_CONTROL_LEFT_PIN_REV, OUTPUT);

    pinMode(MOTOR_CONTROL_RIGHT_PIN_FWD, OUTPUT);
    pinMode(MOTOR_CONTROL_RIGHT_PIN_REV, OUTPUT);

    pinMode(MOTOR_LEFT_ENCODER_C1_PIN, INPUT);
    pinMode(MOTOR_LEFT_ENCODER_C2_PIN, INPUT);

    pinMode(MOTOR_RIGHT_ENCODER_C1_PIN, INPUT);
    pinMode(MOTOR_RIGHT_ENCODER_C2_PIN, INPUT);
}


void motor_encoder_init(callback_encoder_fb)
{
    attachInterrupt(MOTOR_LEFT_ENCODER_C1_PIN, motor_encoder_left_interrupt, CHANGE);
    attachInterrupt(MOTOR_RIGHT_ENCODER_C1_PIN, motor_encoder_right_interrupt, CHANGE);
}


void IRAM_ATTR motor_encoder_left_interrupt()
{
    motor_encoder_callback(0);
}


void IRAM_ATTR motor_encoder_right_interrupt()
{
    motor_encoder_callback(1);
}

