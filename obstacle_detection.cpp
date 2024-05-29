#include "obstacle_detection.h"



void init_obstacle_detect_controls()
{
    pinMode(OBST_DETECT_RIGHT_EN_PIN, OUTPUT);
    digitalWrite(OBST_DETECT_RIGHT_EN_PIN, false);
    pinMode(OBST_DETECT_LEFT_EN_PIN, OUTPUT);
    digitalWrite(OBST_DETECT_LEFT_EN_PIN, false);
    pinMode(OBST_DETECT_FRONT_L_EN_PIN, OUTPUT);
    digitalWrite(OBST_DETECT_FRONT_L_EN_PIN, false);
    pinMode(OBST_DETECT_FRONT_R_EN_PIN, OUTPUT);
    digitalWrite(OBST_DETECT_FRONT_R_EN_PIN, false);
}

uint16_t obstacle_detect_controls_read(obstacle_detect_controls_pos_t control)
{
    uint16_t analog_read_val;
    switch (control)
    {
    case OBSTACLE_DETECTION_LEFT:
        /* code */
        analog_read_val = analogRead(OBST_DETECT_LEFT_SEN_PIN);
        break;
    case OBSTACLE_DETECTION_RIGHT:
        /* code */
        analog_read_val = analogRead(OBST_DETECT_RIGHT_SEN_PIN);
        break;
    case OBSTACLE_DETECTION_FRONT_LEFT:
        /* code */
        analog_read_val = analogRead(OBST_DETECT_FRONT_L_SEN_PIN);
        break;
    case OBSTACLE_DETECTION_FRONT_RIGHT:
        /* code */
        analog_read_val = analogRead(OBST_DETECT_FRONT_R_SEN_PIN);
        break;

    default:
        analog_read_val = 0;
        break;
    }
    return analog_read_val;
}


void obstacle_detect_controls_en(obstacle_detect_controls_pos_t control , bool state)
{
    switch (control)
    {
    case OBSTACLE_DETECTION_LEFT:
        digitalWrite(OBST_DETECT_LEFT_EN_PIN, state);
        break;
    case OBSTACLE_DETECTION_RIGHT:
        digitalWrite(OBST_DETECT_RIGHT_EN_PIN, state);
        break;
    case OBSTACLE_DETECTION_FRONT_LEFT:
        digitalWrite(OBST_DETECT_FRONT_L_EN_PIN, state);
        break;
    case OBSTACLE_DETECTION_FRONT_RIGHT:
        digitalWrite(OBST_DETECT_FRONT_R_EN_PIN, state);
        break;

    default:
        break;
    }
}