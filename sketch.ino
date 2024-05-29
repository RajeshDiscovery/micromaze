#include "io_controls.h"
#include "motor_controls.h"
#include "serial_print.h"
#include <PID_v1.h>
#include "hardware_timer.h"
#include "obstacle_detection.h"

void setup()
{
    Serial.begin(9600);
    init_io_controls();
    motor_control_init();
    hw_timer_setup(100);
    motor_encoder_init(motor_encoder_callback);

    motor_control_move(motor_sel_t::MOTOR_SEL_LEFT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD, MOTOR_SPEED_40);
    motor_control_move(motor_sel_t::MOTOR_SEL_RIGHT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD, MOTOR_SPEED_40);
    // motor_control_move(motor_sel_t::MOTOR_SEL_RIGHT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD,100);

    init_obstacle_detect_controls();
    obstacle_detect_controls_en(OBSTACLE_DETECTION_LEFT, false);
    obstacle_detect_controls_en(OBSTACLE_DETECTION_RIGHT,false);
    obstacle_detect_controls_en(OBSTACLE_DETECTION_FRONT_LEFT ,false);
    obstacle_detect_controls_en(OBSTACLE_DETECTION_FRONT_RIGHT ,false);
}


// void init_obstacle_detect_controls();
// uint16_t obstacle_detect_controls_read();


volatile uint16_t timer_counter = 0;
    // uint8_t speed_pwm = 10;
    // bool dir = 0;
extern volatile uint32_t timer_counter_inside;


uint8_t pwm_speed = 0;
uint32_t start_time = 0 ;
motor_speed_levels_t motor_speed_levels = MOTOR_SPEED_STOP;


obstacle_detect_controls_pos_t obstacle_pos[4] = {
    OBSTACLE_DETECTION_LEFT,
    OBSTACLE_DETECTION_RIGHT,
    OBSTACLE_DETECTION_FRONT_LEFT,
    OBSTACLE_DETECTION_FRONT_RIGHT
};

void read_obstacle_distance()
{
    uint16_t analog_data;
    for(uint8_t indx=0; indx < 4; indx++)
    {
        obstacle_detect_controls_en(obstacle_pos[indx], true);
        delay(20);
        analog_data = obstacle_detect_controls_read(obstacle_pos[indx]);
        analog_data = obstacle_detect_controls_read(obstacle_pos[indx]);
        analog_data = obstacle_detect_controls_read(obstacle_pos[indx]);
        analog_data = obstacle_detect_controls_read(obstacle_pos[indx]);
        serialPrintf("analog[%d] =%d \r\n", indx , analog_data);
        obstacle_detect_controls_en(obstacle_pos[indx], false);

    }
}



void loop()
{

    motor_controls_state_t *motor_state =  motor_ctrl_read_sate();
    // serialPrintf("the counter of motor[0] = %d \r\n", motor_state[0].motor_encoder_counter);
    // serialPrintf("the counter of motor[1] = %d \r\n", motor_state[1].motor_encoder_counter);
    // serialPrintf("motor 0 speed  = %d \r\n", motor_state[0].speed);
    // serialPrintf("motor 1 speed  = %d \r\n", motor_state[1].speed);
    // serialPrintf("the print test");
    uint16_t analog_data;
    while (true)
    {
        // motor_control_move(motor_sel_t::MOTOR_SEL_LEFT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD,speed_pwm);
        // // speed_pwm += 10;
        // serialPrintf("the counter of motor[0] = %d \r\n", motor_state[0].motor_encoder_counter);
        // serialPrintf("the counter of motor[1] = %d \r\n", motor_state[1].motor_encoder_counter);

        // serialPrintf("pwm= ,%d , speed  = ,%d, \r\n", (uint16_t)motor_speed_levels , motor_state[0].speed);
        // serialPrintf("motor 1 speed  = %d \r\n", motor_state[1].speed);
        // serialPrintf("timer_counter =%d \r\n", timer_counter);
        // serialPrintf("timer_counter_inside =%d \r\n",timer_counter_inside);
        // Serial.println("Hello world ...");
        // analog_data = obstacle_detect_controls_read(OBSTACLE_DETECTION_LEFT);
        // serialPrintf("analog =%d \r\n",analog_data);
        read_obstacle_distance();
        delay(500);
        // process_speed_control();
        // if(dir)
        // {
        //     motor_control_move(motor_sel_t::MOTOR_SEL_RIGHT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD,100);
        //     dir = 0;
        // }
        // else
        // {
        //     dir =1;
        //     motor_control_move(motor_sel_t::MOTOR_SEL_RIGHT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_REVERSE,100);
        // }
        // if( get_time_diff_u32(start_time, millis()) > 10000)
        // {
        //     start_time = millis();
        //     pwm_speed += 10;
        //     motor_speed_levels = (motor_speed_levels_t)((int)motor_speed_levels + (int)MOTOR_SPEED_10);
        //     if(motor_speed_levels == MOTOR_SPEED_MAX)
        //     {
        //         motor_speed_levels = MOTOR_SPEED_STOP;
        //     }
        //     motor_control_move(motor_sel_t::MOTOR_SEL_LEFT,motor_rotation_ctrl_t::MOTOR_CONTROLS_MOVE_FORWARD,motor_speed_levels);
        // }
    }
    // {
    //     /* code */
    // }

}



// void hw_timer_callback()
// {
//     timer_counter++;
// }


