#include "hardware_timer.h"

static hw_timer_t *Timer0_Cfg = NULL;
void IRAM_ATTR Timer0_ISR();

volatile uint32_t timer_counter_inside = 0;

void hw_timer_setup(uint16_t time_ms)
{
    uint32_t pre_scalar;
    //TODO calculat pre_scalar
    pre_scalar = time_ms;
    Timer0_Cfg = timerBegin(0, 8000, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 1000, true);
    timerAlarmEnable(Timer0_Cfg);
}
//TODO weak not working here .. .
void hw_timer_callback() // __attribute__((weak));
{
    /**
     * @brief
     * No Implementation Required ...
     */
    motor_control_timer_callback_f100ms();
}

void IRAM_ATTR Timer0_ISR()
{
    hw_timer_callback();
}
