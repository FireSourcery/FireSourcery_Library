#pragma once



//todo depreciate

#ifndef MOT_USER_DIN_COUNT
#define MOT_USER_DIN_COUNT 2U
#endif

#ifndef MOT_USER_AIN_COUNT
#define MOT_USER_AIN_COUNT 2U
#endif

/*
    Fixed-slot role binding for AINS[]
*/
typedef enum MotAnalogUser_AinId
{
    MOT_AIN_THROTTLE = 0U,
    MOT_AIN_BRAKE    = 1U,
    MOT_AIN_COUNT
}
MotAnalogUser_AinId_T;
