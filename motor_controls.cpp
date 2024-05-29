

#include "motor_controls.h"
#include <PID_v1.h>






//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=8, Ki=0.01, Kd=0.05;
PID motor_left(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const uint8_t left_motor_map[MOTOR_SPEED_MAX] = {
    0,
    50 ,
    80,
    100,
    150,
    220
};

const uint8_t right_motor_map[MOTOR_SPEED_MAX] = {
    0,
    50 ,
    80,
    100,
    150,
    220
};

const uint8_t motor_speed_table[MOTOR_SPEED_MAX] =
{
    0,
    10,
    20,
    30,
    40,
    50,
};

const uint8_t* motor_speed_to_pwm_map[NUMBER_OF_MOTORS] = {
    left_motor_map,
    right_motor_map
};




class motor_controls_pid
{
private:
    // Specify the links and initial tuning parameters
    double Kp[NUMBER_OF_MOTORS] = {2, 2}, Ki[NUMBER_OF_MOTORS] = {5, 5}, Kd[NUMBER_OF_MOTORS] = {1, 1};
    double Setpoint[NUMBER_OF_MOTORS], Input[NUMBER_OF_MOTORS], Output[NUMBER_OF_MOTORS];

    PID motor_pid[NUMBER_OF_MOTORS] = {
        PID(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT),
        PID(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT)
    };


public:
    motor_controls_pid(/* args */);
    double motor_controls_pid_compute(uint8_t motor_sel , double input );
    double motor_controls_pid_set_speed(uint8_t motor_sel , uint8_t speed);
    ~motor_controls_pid();
};


static motor_steering_control_t steering_control[NUMBER_OF_MOTORS];
volatile motor_controls_state_t motor_speed_status[NUMBER_OF_MOTORS];
motor_controls_pid motor_pid;


void motor_control_with_direction(motor_sel_t sel, motor_rotation_ctrl_t rot_ctrl);


bool motor_control_init()
{
    steering_control[0].channel = MOTOR_CONTROL_LEFT_MOTOR_CHANNEL;
    steering_control[0].forward_pin = MOTOR_CONTROL_LEFT_PIN_FWD;
    steering_control[0].reverse_pin = MOTOR_CONTROL_LEFT_PIN_REV;
    steering_control[0].motor_dir_ctrl = MOTOR_CONTROLS_STOP;
    steering_control[0].set_speed = 0;
    steering_control[0].speed_hysteresis = 1;

    steering_control[1].channel = MOTOR_CONTROL_RIGHT_MOTOR_CHANNEL;
    steering_control[1].forward_pin = MOTOR_CONTROL_RIGHT_PIN_FWD;
    steering_control[1].reverse_pin = MOTOR_CONTROL_RIGHT_PIN_REV;
    steering_control[1].motor_dir_ctrl = MOTOR_CONTROLS_STOP;
    steering_control[1].set_speed = 0;
    steering_control[1].speed_hysteresis = 1;

    ledcSetup(steering_control[0].channel, MOTOR_CONTROL_PWM_FREQ, MOTOR_CONTROL_PWM_RES);
    ledcSetup(steering_control[1].channel, MOTOR_CONTROL_PWM_FREQ, MOTOR_CONTROL_PWM_RES);
    return false;
}


void motor_control_move(motor_sel_t sel , motor_rotation_ctrl_t rot_ctrl , motor_speed_levels_t speed)
{
    motor_control_with_direction(sel, rot_ctrl);
    steering_control[sel].set_speed = motor_speed_table[speed];
    ledcWrite(steering_control[sel].channel, motor_speed_to_pwm_map[sel][speed]);
    steering_control[sel].driving_pwm =motor_speed_to_pwm_map[sel][speed];
}


    // if(sel == motor_sel_t ::MOTOR_SEL_LEFT)
    // {
    //     Setpoint = (double) speed;
    //     //serialPrintf("Setpointd =%f , setpointd=%d \r\n",Setpoint , speed);
    //     // motor_left.SetMode(AUTOMATIC);

    // }
    // motor_pid.motor_controls_pid_set_speed(sel,speed);
    // ledcWrite(steering_control[sel].channel, speed);
// }


void motor_control_move(motor_sel_t sel , motor_rotation_ctrl_t rot_ctrl , uint8_t speed)
{
    motor_control_with_direction(sel, rot_ctrl);
    steering_control[sel].set_speed = speed;
    if(sel == motor_sel_t ::MOTOR_SEL_LEFT)
    {
        Setpoint = (double) speed;
        //serialPrintf("Setpointd =%f , setpointd=%d \r\n",Setpoint , speed);
        // motor_left.SetMode(AUTOMATIC);
        ledcWrite(steering_control[sel].channel, speed);
    }
    // motor_pid.motor_controls_pid_set_speed(sel,speed);
    // ledcWrite(steering_control[sel].channel, speed);
}

void motor_control_with_direction(motor_sel_t sel, motor_rotation_ctrl_t rot_ctrl)
{
    if(steering_control[sel].motor_dir_ctrl != rot_ctrl)
    {
        ledcWrite(steering_control[sel].channel, 0);
        switch (rot_ctrl)
        {
            case MOTOR_CONTROLS_STOP:
            {
                if(steering_control[sel].motor_dir_ctrl == MOTOR_CONTROLS_MOVE_REVERSE)
                {
                    ledcDetachPin(steering_control[sel].reverse_pin);
                }

                if(steering_control[sel].motor_dir_ctrl == MOTOR_CONTROLS_MOVE_FORWARD)
                {
                    ledcDetachPin(steering_control[sel].forward_pin);
                }
                digitalWrite(steering_control[sel].reverse_pin, false);
                digitalWrite(steering_control[sel].forward_pin, false);
            }
            break;
            case MOTOR_CONTROLS_MOVE_FORWARD:
            {
                if(steering_control[sel].motor_dir_ctrl == MOTOR_CONTROLS_MOVE_REVERSE)
                {
                    //TODO interface has to provide for hal later
                    ledcDetachPin(steering_control[sel].reverse_pin);
                    ledcAttachPin(steering_control[sel].forward_pin, steering_control[sel].channel);
                    //serialPrintf("motor [%d] Frd-rev fwdpin =%d rev=%d\r\n",(uint8_t)sel,
                    // steering_control[sel].reverse_pin, steering_control[sel].forward_pin);
                }
                else
                {
                    // initial motor condition
                    ledcAttachPin(steering_control[sel].forward_pin, steering_control[sel].channel);
                    //serialPrintf("motor [%d] frd-else \r\n",(uint8_t)sel );
                }
            }
            break;
            case MOTOR_CONTROLS_MOVE_REVERSE:
            {
                if(steering_control[sel].motor_dir_ctrl == MOTOR_CONTROLS_MOVE_FORWARD)
                {
                    //TODO interface has to provide for hal later
                    ledcDetachPin(steering_control[sel].forward_pin);
                    ledcAttachPin(steering_control[sel].reverse_pin, steering_control[sel].channel);
                    //serialPrintf("motor [%d] rev-frd  rev=%d fwd=%d \r\n",(uint8_t)sel,steering_control[sel].reverse_pin,
                    // steering_control[sel].forward_pin);
                }
                else
                {
                    // initial motor condition
                    ledcAttachPin(steering_control[sel].reverse_pin, steering_control[sel].channel);
                    //serialPrintf("motor [%d] rev-else \r\n",(uint8_t)sel);
                }
            }
            break;

            default:
                break;
        }
        steering_control[sel].motor_dir_ctrl = rot_ctrl;
    }
}

void motor_control_left(motor_rotation_ctrl_t rot_ctrl , uint8_t degree)
{

}

#if 0
void motor_control_left(motor_rotation_ctrl_t rot_ctrl)
{
    if(steering_control.motor_left_control != rot_ctrl)
    {
        switch (rot_ctrl)
        {
            case MOTOR_CONTROLS_STOP:
            {
                if(steering_control.motor_left_control == MOTOR_CONTROLS_MOVE_REVERSE)
                {
                    ledcDetachPin(MOTOR_CONTROL_LEFT_PIN_REV);
                }

                if(steering_control.motor_left_control == MOTOR_CONTROLS_MOVE_FORWARD)
                {
                    ledcDetachPin(MOTOR_CONTROL_LEFT_PIN_FWD);
                }
                digitalWrite(MOTOR_CONTROL_LEFT_PIN_FWD, false);
                digitalWrite(MOTOR_CONTROL_LEFT_PIN_REV, false);
            }
            break;
            case MOTOR_CONTROLS_MOVE_FORWARD:
            {
                if(steering_control.motor_left_control == MOTOR_CONTROLS_MOVE_REVERSE)
                {
                    //TODO interface has to provide for hal later
                    ledcDetachPin(MOTOR_CONTROL_LEFT_PIN_REV);
                    ledcAttachPin(MOTOR_CONTROL_LEFT_PIN_FWD, MOTOR_CONTROL_LEFT_MOTOR_CHANNEL);
                }
                else
                {
                    // initial motor condition
                    ledcAttachPin(MOTOR_CONTROL_LEFT_PIN_FWD, MOTOR_CONTROL_LEFT_MOTOR_CHANNEL);
                }
            }
            break;
            case MOTOR_CONTROLS_MOVE_REVERSE:
            {
                if(steering_control.motor_left_control == MOTOR_CONTROLS_MOVE_FORWARD)
                {
                    //TODO interface has to provide for hal later
                    ledcDetachPin(MOTOR_CONTROL_LEFT_PIN_FWD);
                    ledcAttachPin(MOTOR_CONTROL_LEFT_PIN_REV, MOTOR_CONTROL_LEFT_MOTOR_CHANNEL);
                }
                else
                {
                    // initial motor condition
                    ledcAttachPin(MOTOR_CONTROL_LEFT_PIN_REV, MOTOR_CONTROL_LEFT_MOTOR_CHANNEL);
                }
            }
            break;

            default:
                break;
        }
        steering_control.motor_left_control = rot_ctrl;
    }

}
#endif



void motor_control_right(motor_rotation_ctrl_t rot_ctrl)
{
    switch (rot_ctrl)
    {
        case MOTOR_CONTROLS_MOVE_FORWARD:
            /* code */
            digitalWrite(MOTOR_CONTROL_RIGHT_PIN_FWD, true);
            digitalWrite(MOTOR_CONTROL_RIGHT_PIN_REV, false);
            break;

        case MOTOR_CONTROLS_MOVE_REVERSE:
            /* code */
            digitalWrite(MOTOR_CONTROL_RIGHT_PIN_FWD, false);
            digitalWrite(MOTOR_CONTROL_RIGHT_PIN_REV, true);
            break;

        default:
            break;
    }
}

void car_movement_control(car_control_movement_t , uint8_t speed)
{

}

void motor_encoder_callback(uint8_t motor_sel)
{

    motor_speed_status[motor_sel].motor_encoder_counter++;

#if false
    motor_sel_t motor_selection = (motor_sel_t) motor_sel;
    switch (motor_selection)
    {
        case MOTOR_SEL_LEFT:
            /* code */
            motor_speed_status[motor_sel].motor_encoder_counter++;
            break;

        case MOTOR_SEL_RIGHT:
            /* code */
            motor_speed_status[motor_sel].motor_encoder_counter++;
            break;

        default:
            break;
    }

#endif
}



motor_controls_state_t * motor_ctrl_read_sate()
{
    return (motor_controls_state_t*)motor_speed_status;
}


// #define NUMBER_OF_MOTORS               2
// #define MOTOR_GEAR_RATIO               1000
// #define MOTOR_ENCODER_REV_CYCLE        3



void motor_control_track_set_point()
{
    int8_t error_speed;
    uint8_t abs_error;

    uint8_t pwm_error_value;
    for(uint8_t sel = 0; sel < NUMBER_OF_MOTORS; sel++)
    {
        error_speed = (motor_speed_status[sel].speed - steering_control[sel].set_speed);
        abs_error = abs(error_speed);
        if(abs_error < 5)
        {
            pwm_error_value =1 ;
        }
        else if((abs_error >= 5 ) && (abs_error < 10 ))
        {
            pwm_error_value = 2;
        }
        else
        {
            pwm_error_value = 5;
        }
        if(error_speed < 0)
        {
            if((steering_control[sel].driving_pwm + pwm_error_value) < 255)
            {
                steering_control[sel].driving_pwm += pwm_error_value;
            }
        }
        else
        {
            if(steering_control[sel].driving_pwm > 0)
            {
                steering_control[sel].driving_pwm -= pwm_error_value;
            }
        }

        //serialPrintf("error_speed[%d] =%d Csp=%d setSpped=%d pwm=%d \r\n", sel, error_speed, motor_speed_status[sel].speed,
//                      steering_control[sel].set_speed ,steering_control[sel].driving_pwm);
        ledcWrite(steering_control[sel].channel, steering_control[sel].driving_pwm);

        // if(error_speed > 0)
        // {
        //     // reduce PWM
        //     if(steering_control[sel].driving_pwm){
        //         steering_control[sel].driving_pwm -= pwm_error_value;
        //         ledcWrite(steering_control[sel].channel, steering_control[sel].driving_pwm);
        //         //serialPrintf("R PWM[%d] =%d \r\n",sel, steering_control[sel].driving_pwm);
        //     }
        // }
        // else if(error_speed < 0)
        // {
        //     if(steering_control[sel].driving_pwm < 255){
        //         steering_control[sel].driving_pwm += pwm_error_value;
        //         ledcWrite(steering_control[sel].channel, steering_control[sel].driving_pwm);
        //         //serialPrintf("I PWM[%d] =%d \r\n",sel ,steering_control[sel].driving_pwm);
        //     }
        // }
        // else
        // {

        // }
    }
}


void motor_control_timer_callback_f100ms()
{
    for(uint8_t indx = 0; indx < NUMBER_OF_MOTORS; indx++)
    {
        motor_speed_status[indx].speed = get_time_diff_u16(motor_speed_status[indx].motor_encoder_prev_counter,
                              motor_speed_status[indx].motor_encoder_counter);
        motor_speed_status[indx].motor_encoder_prev_counter = motor_speed_status[indx].motor_encoder_counter;
    }
    motor_control_track_set_point();
    // process_speed_control();
}



void process_speed_control()
{
    uint8_t output;
    Input = (double)motor_speed_status[0].speed;
    motor_left.Compute();
    output = (uint8_t) Output;
    ledcWrite(steering_control[0].channel, (uint8_t)map(output ,0,50,0,255));
    //serialPrintf("Setpoint =%f  motor o/p =%d motor curent speed =%d \r\n",Setpoint ,  output, motor_speed_status[0].speed);


    // for(uint8_t indx =0; indx < NUMBER_OF_MOTORS ; indx++)
    // {
    //     output = (uint8_t)motor_pid.motor_controls_pid_compute(indx ,motor_speed_status[indx].speed);
    //     ledcWrite(steering_control[indx].channel, output);
    // }
}

motor_controls_pid::motor_controls_pid()
{
}

double motor_controls_pid::motor_controls_pid_set_speed(uint8_t motor_sel , uint8_t speed)
{
    // Setpoint[0] = (double)0.10;
}

double motor_controls_pid::motor_controls_pid_compute(uint8_t motor_sel, double input)
{
    Input[motor_sel] = input;
    motor_pid[motor_sel].Compute();
    return Output[motor_sel];
}


motor_controls_pid::~motor_controls_pid()
{
}
