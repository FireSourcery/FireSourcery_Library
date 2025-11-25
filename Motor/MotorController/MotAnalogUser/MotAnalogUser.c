/******************************************************************************/
/*!
    @file   MotAnalogUser.c
    @author FireSourcery
    @brief  Analog user controls using UserAIn/UserDIn modules
*/
/******************************************************************************/
#include "MotAnalogUser.h"
#include <string.h>

/******************************************************************************/
/*
    Private Helper Functions
*/
/******************************************************************************/
static inline void InitThrottleAIn(const MotAnalogUser_T * p_user) { UserAIn_InitFrom(&p_user->THROTTLE_AIN, &p_user->P_STATE->Config.ThrottleAInConfig); }
static inline void InitBrakeAIn(const MotAnalogUser_T * p_user) { UserAIn_InitFrom(&p_user->BRAKE_AIN, &p_user->P_STATE->Config.BrakeAInConfig); }

/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
void MotAnalogUser_Init(const MotAnalogUser_T * p_user)
{
    /* Load configuration from NVM if available */
    if (p_user->P_NVM_CONFIG != NULL) { p_user->P_STATE->Config = *p_user->P_NVM_CONFIG; }

    /* Initialize analog inputs */
    InitThrottleAIn(p_user);
    InitBrakeAIn(p_user);

    /* Initialize digital pins */
    UserDIn_Init(&p_user->REVERSE_DIN);
    if (p_user->P_STATE->Config.UseForwardPin == true) { UserDIn_Init(&p_user->FORWARD_DIN); }
    if (p_user->P_STATE->Config.UseNeutralPin == true) { UserDIn_Init(&p_user->NEUTRAL_DIN); }
    if (p_user->P_STATE->Config.UseSwitchBrakePin == true) { UserDIn_Init(&p_user->SWITCH_BRAKE_DIN); }

    /* Initialize state */
    p_user->P_STATE->Cmd = MOT_ANALOG_USER_CMD_PROC_ZERO;
}



/******************************************************************************/
/*
    Configuration Functions
*/
/******************************************************************************/
void MotAnalogUser_SetBrakeZero(const MotAnalogUser_T * p_user, uint16_t zero_Adcu)
{
    p_user->P_STATE->Config.BrakeAInConfig.AdcZero = zero_Adcu;
    InitBrakeAIn(p_user);
}

void MotAnalogUser_SetThrottleZero(const MotAnalogUser_T * p_user, uint16_t zero_Adcu)
{
    p_user->P_STATE->Config.ThrottleAInConfig.AdcZero = zero_Adcu;
    InitThrottleAIn(p_user);
}

void MotAnalogUser_SetSwitchBrake(const MotAnalogUser_T * p_user, bool useSwitchBrake, uint16_t bistateBrakeIntensity_Fract16)
{
    p_user->P_STATE->Config.UseSwitchBrakePin = useSwitchBrake;
    p_user->P_STATE->Config.SwitchBrakeValue_Percent16 = bistateBrakeIntensity_Fract16;
}

void MotAnalogUser_SetDirectionPins(const MotAnalogUser_T * p_user, MotAnalogUser_DirectionPins_T pins)
{
    switch (pins)
    {
        case MOT_ANALOG_USER_DIRECTION_PINS_FNR:
            p_user->P_STATE->Config.UseForwardPin = true;
            p_user->P_STATE->Config.UseNeutralPin = true;
            break;
        case MOT_ANALOG_USER_DIRECTION_PINS_FR:
            p_user->P_STATE->Config.UseForwardPin = true;
            p_user->P_STATE->Config.UseNeutralPin = false;
            break;
        case MOT_ANALOG_USER_DIRECTION_PINS_R:
            p_user->P_STATE->Config.UseForwardPin = false;
            p_user->P_STATE->Config.UseNeutralPin = false;
            break;
        default: break;
    }
}

/*
    optional
*/
void MotAnalogUser_SetBrakeRange(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_user->P_STATE->Config.BrakeAInConfig.AdcZero = zero_Adcu;
    p_user->P_STATE->Config.BrakeAInConfig.AdcMax = max_Adcu;
    InitBrakeAIn(p_user);
}

void MotAnalogUser_SetThrottleRange(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu)
{
    p_user->P_STATE->Config.ThrottleAInConfig.AdcZero = zero_Adcu;
    p_user->P_STATE->Config.ThrottleAInConfig.AdcMax = max_Adcu;
    InitThrottleAIn(p_user);
}

void MotAnalogUser_SetBrakeAIn(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useBrakeEdgePin)
{
    MotAnalogUser_SetBrakeRange(p_user, zero_Adcu, max_Adcu);
    p_user->P_STATE->Config.UseBrakeEdgePin = useBrakeEdgePin;
}

void MotAnalogUser_SetThrottleAIn(const MotAnalogUser_T * p_user, uint16_t zero_Adcu, uint16_t max_Adcu, bool useThrottleEdgePin)
{
    MotAnalogUser_SetThrottleRange(p_user, zero_Adcu, max_Adcu);
    p_user->P_STATE->Config.UseThrottleEdgePin = useThrottleEdgePin;
}

/******************************************************************************/
/*
    Variable Access Functions
*/
/******************************************************************************/
int32_t MotAnalogUser_VarId_Get(const MotAnalogUser_T * p_user, MotAnalogUser_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE:           value = UserAIn_GetValue(&p_user->THROTTLE_AIN);              break;
        case MOT_ANALOG_USER_BRAKE:              value = UserAIn_GetValue(&p_user->BRAKE_AIN);                 break;
        case MOT_ANALOG_USER_THROTTLE_DIN:       value = UserAIn_IsOn(&p_user->THROTTLE_AIN);    break;
        case MOT_ANALOG_USER_BRAKE_DIN:          value = UserAIn_IsOn(&p_user->BRAKE_AIN);       break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_DIN:   value = UserDIn_GetState(&p_user->SWITCH_BRAKE_DIN);          break;
        case MOT_ANALOG_USER_FORWARD_DIN:        value = UserDIn_GetState(&p_user->FORWARD_DIN);               break;
        case MOT_ANALOG_USER_REVERSE_DIN:        value = UserDIn_GetState(&p_user->REVERSE_DIN);               break;
        case MOT_ANALOG_USER_NEUTRAL_DIN:        value = UserDIn_GetState(&p_user->NEUTRAL_DIN);               break;
        default: break;
    }
    return value;
}

int32_t MotAnalogUser_VarId_GetAsInput(const MotAnalogUser_T * p_user, MotAnalogUser_VarId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE:           value = p_user->THROTTLE_AIN.P_STATE->RawValue_Adcu;                       break;
        case MOT_ANALOG_USER_BRAKE:              value = p_user->BRAKE_AIN.P_STATE->RawValue_Adcu;                          break;
        case MOT_ANALOG_USER_THROTTLE_DIN:       value = _UserAIn_IsEdgePinOn(p_user->THROTTLE_AIN.P_EDGE_PIN);     break;
        case MOT_ANALOG_USER_BRAKE_DIN:          value = _UserAIn_IsEdgePinOn(p_user->BRAKE_AIN.P_EDGE_PIN);        break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_DIN:   value = _UserAIn_IsEdgePinOn(&p_user->SWITCH_BRAKE_DIN);            break;
        case MOT_ANALOG_USER_FORWARD_DIN:        value = Pin_Input_ReadPhysical(&p_user->FORWARD_DIN.PIN);                  break;
        case MOT_ANALOG_USER_REVERSE_DIN:        value = Pin_Input_ReadPhysical(&p_user->REVERSE_DIN.PIN);                  break;
        case MOT_ANALOG_USER_NEUTRAL_DIN:        value = Pin_Input_ReadPhysical(&p_user->NEUTRAL_DIN.PIN);                  break;
        // case MOT_ANALOG_USER_OPT_DIN:            value = Pin_Input_ReadPhysical(&p_user->OptDin.Pin);                          break;
        default: break;
    }
    return value;
}

int32_t MotAnalogUser_ConfigId_Get(const MotAnalogUser_T * p_user, MotAnalogUser_ConfigId_T id)
{
    int32_t value = 0;
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:             value = p_user->P_STATE->Config.ThrottleAInConfig.AdcZero;              break;
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:              value = p_user->P_STATE->Config.ThrottleAInConfig.AdcMax;               break;
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:    value = p_user->P_STATE->Config.UseThrottleEdgePin;                     break;
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:                value = p_user->P_STATE->Config.BrakeAInConfig.AdcZero;                 break;
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:                 value = p_user->P_STATE->Config.BrakeAInConfig.AdcMax;                  break;
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:       value = p_user->P_STATE->Config.UseBrakeEdgePin;                        break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:             value = p_user->P_STATE->Config.SwitchBrakeValue_Percent16;             break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_IS_ENABLE:         value = p_user->P_STATE->Config.UseSwitchBrakePin;                      break;
        default: break;
    }
    return value;
}

void MotAnalogUser_ConfigId_Set(const MotAnalogUser_T * p_user, MotAnalogUser_ConfigId_T id, int32_t value)
{
    switch (id)
    {
        case MOT_ANALOG_USER_THROTTLE_ZERO_ADCU:             MotAnalogUser_SetThrottleRange(p_user, value, p_user->P_STATE->Config.ThrottleAInConfig.AdcMax);   break;
        case MOT_ANALOG_USER_THROTTLE_MAX_ADCU:              MotAnalogUser_SetThrottleRange(p_user, p_user->P_STATE->Config.ThrottleAInConfig.AdcZero, value);  break;
        case MOT_ANALOG_USER_THROTTLE_EDGE_PIN_IS_ENABLE:    p_user->P_STATE->Config.UseThrottleEdgePin = value;                                                break;
        case MOT_ANALOG_USER_BRAKE_ZERO_ADCU:                MotAnalogUser_SetBrakeRange(p_user, value, p_user->P_STATE->Config.BrakeAInConfig.AdcMax);         break;
        case MOT_ANALOG_USER_BRAKE_MAX_ADCU:                 MotAnalogUser_SetBrakeRange(p_user, p_user->P_STATE->Config.BrakeAInConfig.AdcZero, value);        break;

        /* handle ad compile */
        case MOT_ANALOG_USER_BRAKE_EDGE_PIN_IS_ENABLE:       p_user->P_STATE->Config.UseBrakeEdgePin = value;                                                   break;

        case MOT_ANALOG_USER_SWITCH_BRAKE_VALUE:             p_user->P_STATE->Config.SwitchBrakeValue_Percent16 = value;                                        break;
        case MOT_ANALOG_USER_SWITCH_BRAKE_IS_ENABLE:         p_user->P_STATE->Config.UseSwitchBrakePin = value;                                                 break;
        default: break;
    }
}



