#include "common.h"


uint16_t get_time_diff_u16(uint16_t start_time, uint16_t end_time)
{
    const uint16_t time_stamp_max = UINT16_MAX;
    if (end_time < start_time)
    {
        return (time_stamp_max - start_time) + end_time;
    }
    return end_time - start_time;
}



uint32_t get_time_diff_u32(uint32_t start_time, uint32_t end_time)
{
    const uint32_t time_stamp_max = UINT32_MAX;
    if (end_time < start_time)
    {
        return (time_stamp_max - start_time) + end_time;
    }
    return end_time - start_time;
}
