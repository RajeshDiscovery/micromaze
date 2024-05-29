#ifndef __OBSTACLE_DETECTION__H__
#define __OBSTACLE_DETECTION__H__

#include "Arduino.h"
#include "common.h"


#define  OBST_DETECT_FRONT_R_EN_PIN    33
#define  OBST_DETECT_FRONT_R_SEN_PIN   39 //Vin

#define  OBST_DETECT_FRONT_L_EN_PIN     25
#define  OBST_DETECT_FRONT_L_SEN_PIN    34

#define  OBST_DETECT_RIGHT_EN_PIN       32
#define  OBST_DETECT_RIGHT_SEN_PIN      36

#define  OBST_DETECT_LEFT_EN_PIN        26
#define  OBST_DETECT_LEFT_SEN_PIN       35

typedef enum
{
    OBSTACLE_DETECTION_LEFT,
    OBSTACLE_DETECTION_RIGHT,
    OBSTACLE_DETECTION_FRONT_LEFT,
    OBSTACLE_DETECTION_FRONT_RIGHT,
}obstacle_detect_controls_pos_t;

// obstacle_detect_controls_pos_t obstacle_detect_control


void init_obstacle_detect_controls();
uint16_t obstacle_detect_controls_read(obstacle_detect_controls_pos_t);
void obstacle_detect_controls_en(obstacle_detect_controls_pos_t , bool state);









#endif