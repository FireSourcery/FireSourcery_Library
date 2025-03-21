/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor.c
    @author FireSourcery
    @brief  Motor module conventional function definitions.
    @version V0
*/
/******************************************************************************/
#include "Motor.h"
#include <string.h>

/* Global Static, for all Motor instances */
static struct
{
    Linear_T UnitsVSource_V; /* VSource Volts of ADCU. effectively const */
    uint16_t VSourceRef_Adcu; /* VabcRef_Adcu. A higher value, matches observed bemf to lower applied voltage */
    uint16_t VSourceRef_V; /*  VSpeedRef */
}
MotorStatic;
// MotorStatic =
// {
//     .UnitsVSource_V = LINEAR_INIT(MOTOR_STATIC.V_ABC_R1, MOTOR_STATIC.V_ABC_R2, GLOBAL_ANALOG.ADC_VREF_MILLIV, GLOBAL_ANALOG.ADC_BITS),
// }

void InitUnitsVSource(void)
{
    Linear_Voltage_Init(&MotorStatic.UnitsVSource_V, MOTOR_STATIC.V_ABC_R1, MOTOR_STATIC.V_ABC_R2, GLOBAL_ANALOG.ADC_VREF_MILLIV, GLOBAL_ANALOG.ADC_BITS, 0U);
}

void Motor_Static_InitVSourceRef_Adcu(uint16_t vSource_Adcu) /* Using the active read value */
{
    InitUnitsVSource();
    MotorStatic.VSourceRef_Adcu = vSource_Adcu;
}

/* Set before Motor_Init */
/* alternatively outer module handle UnitsVabc and and SpeedVRef */
void Motor_Static_InitVSourceRef_V(uint16_t vSource_V)
{
    InitUnitsVSource();
    MotorStatic.VSourceRef_Adcu = Linear_Voltage_AdcuOfV(&MotorStatic.UnitsVSource_V, vSource_V);
    MotorStatic.VSourceRef_V = vSource_V;
}

uint16_t Motor_Static_GetVSource_V(void) { return MotorStatic.VSourceRef_V; }
uint16_t Motor_Static_GetVSource_Adcu(void) { return MotorStatic.VSourceRef_Adcu; }
// alternatively cache rpmRef and handle conversion by caller
// uint16_t Motor_Static_GetVSource_V(void) { return Linear_Voltage_Of(&MotorStatic.UnitsVSource_V, MotorStatic.VSourceRef_Adcu); }
// uint16_t Motor_Static_GetVSource_V10(void) { return Linear_Voltage_Of(&MotorStatic.UnitsVSource_V, (uint32_t)MotorStatic.VSourceRef_Adcu * 10U); }

/* Conversion via adcu. Determine highest precision without overflow */
// uint16_t Motor_Static_RpmOfKv(uint16_t kv) { return Linear_Voltage_ScalarV(&MotorStatic.UnitsVSource_V, MotorStatic.VSourceRef_Adcu, kv); }
// kv < ~1000
// uint16_t Motor_Static_RpmOfKv(uint16_t kv) { return Linear_Voltage_Of(&MotorStatic.UnitsVSource_V, MotorStatic.VSourceRef_Adcu * kv); }


/*
    Init Motor_Static first
*/
void Motor_Init(Motor_T * p_motor)
{
    if(p_motor->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_motor->Config, p_motor->CONST.P_NVM_CONFIG, sizeof(Motor_Config_T)); }
    Motor_InitReboot(p_motor); // move to state machine?
    StateMachine_Init(&p_motor->StateMachine);
}

void Motor_InitReboot(Motor_T * p_motor)
{
    /*
        HW Wrappers Init
    */
    Phase_Init(&p_motor->Phase);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Config.PhasePwmMode);
#endif
    Motor_InitSensor(p_motor);
    Thermistor_Init(&p_motor->Thermistor);

    /*
        SW Structs
    */
    Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
    Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

    FOC_Init(&p_motor->Foc);
    // BEMF_Init(&p_motor->Bemf);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_Init(&p_motor->PidSpeed);
    PID_Init(&p_motor->PidIq);
    PID_Init(&p_motor->PidId);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
#endif

    /*
        e.g. (CONTROL_FREQ, INT16_MAX) => Ramp 0 to SpeedRef_Rpm in 1000ms
        Sets slope until reset
    */
    Ramp_Init(&p_motor->Ramp, p_motor->Config.RampAccel_Cycles, INT16_MAX); /* todo as speed freq cycles */
    Ramp_Init(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, INT16_MAX);
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopAccel_Cycles, p_motor->Config.OpenLoopSpeed_UFract16); /* direction updated on set */
#endif

    Motor_ResetUnitsVabc(p_motor);
    Motor_ResetUnitsIabc(p_motor);

    Motor_ResetSpeedLimitActive(p_motor);
    Motor_ResetILimitActive(p_motor);

    // p_motor->FeedbackMode.Word = p_motor->Config.FeedbackModeDefault.Word; /* StateMachine init sets initial limits using this value. */
    p_motor->ControlTimerBase = 0U;
}


/******************************************************************************/
/*!
    Position Sensor Feedback - Speed, Angle
*/
/******************************************************************************/
/* Re init peripheral registers */

void Motor_InitSensor(Motor_T * p_motor)
{
    // MotorSensor_Init(&p_motor->Sensor Interface);

    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Hall_Init(&p_motor->Hall);
        #if     defined(CONFIG_MOTOR_HALL_MODE_POLLING) || !defined(CONFIG_MOTOR_HALL_MODE_ISR)
            Encoder_ModeDT_Init_Polling(&p_motor->Encoder);
        #elif   defined(CONFIG_MOTOR_HALL_MODE_ISR)
            Encoder_InitInterrupts_ABC(&p_motor->Encoder);
        #endif
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_ModeDT_Init_InterruptQuadrature(&p_motor->Encoder);
            Encoder_EnableQuadratureMode(&p_motor->Encoder);
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            SinCos_Init(&p_motor->SinCos);
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            break;
        #endif
        default:
            break;
    }
    Motor_ResetUnitsSensor(p_motor);
}

bool Motor_VerifySensorCalibration(Motor_T * p_motor)
{
    bool isValid = true;
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            if (Hall_IsStateValid(&p_motor->Hall) == false) { isValid = false; }
            if (Hall_IsTableValid(&p_motor->Hall) == false) { isValid = false; }
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            break;
        #endif
        default:
            break;
    }
    return isValid;
}

angle16_t Motor_PollSensorAngle(Motor_T * p_motor)
{
    angle16_t electricalAngle; /* [0, 65535] maps to negative portions of angle16_t */

    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
        #if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
            if (Hall_PollCaptureSensors(&p_motor->Hall) == true)
            {
                Encoder_SinglePhase_CapturePulse(&p_motor->Encoder);
                Hall_CaptureAngle(&p_motor->Hall);
            }
            // Encoder_CaptureCount

            /* by assigned direction, alternatively compare prev state */
        #endif
            electricalAngle = Hall_GetAngle16(&p_motor->Hall);
            electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
            /* handle direction reset */
            break;

        case MOTOR_SENSOR_MODE_ENCODER:
            electricalAngle = Encoder_GetAngle_Scalar(&p_motor->Encoder, p_motor->Config.PolePairs); /* ElectricalAngle => MechanicalAngle * PolePairs */
            // electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
            break;

        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
            electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
            //todo group
            AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
            AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            //todo observer
            electricalAngle = 0;
            p_motor->FeedbackMode.OpenLoop = 1U;
            p_motor->FeedbackMode.OpenLoop = 1U;
            p_motor->StateFlags.SensorFeedback = 0U;
            p_motor->StateFlags.SensorFeedback = 0U;
            break;
        #endif
        default: electricalAngle = 0; break;
    }

    return electricalAngle;
}

angle16_t Motor_CaptureAngle(Motor_T * p_motor)
{
    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    return p_motor->ElectricalAngle;
}

angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor)
{
    angle16_t angle;
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:    angle = Encoder_GetAngle(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER: angle = Encoder_GetAngle(&p_motor->Encoder);    break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS: angle = SinCos_GetMechanicalAngle(&p_motor->SinCos);     break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS: angle = 0; break;
        #endif
        default: angle = 0; break;
    }
    return angle;
}

/* For SinCos, Sensorless, when not using Encoder module */
// static inline int32_t PollAngleSpeed(Motor_T * p_motor, angle16_t speedAngle)
// {
//     int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
//     int32_t speedFeedback_Fract16 = (p_motor->Speed_Fract16 + Linear_Speed_CalcAngleRpmFract16(&p_motor->UnitsAngleRpm, speedDelta)) / 2;
//     p_motor->SpeedAngle = speedAngle; /* mechanical angle */
//     return speedFeedback_Fract16;
// }

/*!
    @return Fract16 Q1.15 unsaturated
*/
int32_t Motor_PollSensorSpeed(Motor_T * p_motor)
{
    int32_t speed_Fixed32;
    switch (p_motor->Config.SensorMode)
    {
        /* Using assigned direction */
        case MOTOR_SENSOR_MODE_HALL:    speed_Fixed32 = Encoder_ModeDT_CaptureScalarVelocity(&p_motor->Encoder); break;
        /*  */
        case MOTOR_SENSOR_MODE_ENCODER: speed_Fixed32 = Encoder_ModeDT_CaptureScalarVelocity(&p_motor->Encoder); break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS: speed_Fixed32 = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));    break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS: break;
        #endif
        default: speed_Fixed32 = 0; break;
    }
    return speed_Fixed32 / 2;
}

bool Motor_PollCaptureSpeed(Motor_T * p_motor)
{
    bool isCaptureSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
    if (isCaptureSpeed == true) { p_motor->Speed_Fract16 = (Motor_PollSensorSpeed(p_motor) + p_motor->Speed_Fract16) / 2; }
    return isCaptureSpeed;
}

void Motor_ZeroSensor(Motor_T * p_motor)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Hall_SetInitial(&p_motor->Hall);
            Encoder_ModeDT_SetInitial(&p_motor->Encoder);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_ModeDT_SetInitial(&p_motor->Encoder);
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            Motor_SetPositionFeedback(p_motor, 0U);
            break;
        #endif
        default: break;
    }
}

/* From Stop and after Align */
bool _Motor_IsSensorAvailable(const Motor_T * p_motor)
{
    bool isAvailable;
    switch(p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:    isAvailable = true;         break;
        case MOTOR_SENSOR_MODE_ENCODER: isAvailable = Encoder_IsAligned(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     isAvailable = true;     break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:  isAvailable = false;    break;
#endif
        default: isAvailable = false; break;
    }
    return isAvailable;
}

static inline bool _Motor_IsOpenLoop(const Motor_T * p_motor)
{
// #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    return (p_motor->FeedbackMode.OpenLoop == 1U);
// #else
//     (void)p_motor; return false;
// #endif
}

inline bool Motor_IsClosedLoop(const Motor_T * p_motor)
{
    return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
}

// Motor_Direction_T Motor_GetDirection(Motor_T * p_motor)
// {
//     switch (p_motor->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SIN_COS:
//             break;
//         #endif
//         #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:
//             break;
//         #endif
//         default:
//             break;
//     }
// }


/*
    Sensor Direction
*/
// static void Motor_SetSensorCcw(Motor_T * p_motor)
// {
//     switch (p_motor->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
//             Encoder_SinglePhase_SetDirectionPositive(&p_motor->Encoder); // interpolate as +/-
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             break;
//         #if defined (CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:     break;
//         #endif
//         default: break;
//     }
// }

// static void Motor_SetSensorCw(Motor_T * p_motor)
// {
//     switch (p_motor->Config.SensorMode)
//     {
//         case MOTOR_SENSOR_MODE_HALL:
//             Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);
//             Encoder_SinglePhase_SetDirectionNegative(&p_motor->Encoder);
//             break;
//         case MOTOR_SENSOR_MODE_ENCODER:
//             break;
//         #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
//         case MOTOR_SENSOR_MODE_SENSORLESS:  break;
//         #endif
//         default: break;
//     }
// }

static void Motor_SetSensorDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Hall_SetDirection(&p_motor->Hall, (Hall_Direction_T)direction);
            Encoder_SinglePhase_SetDirection(&p_motor->Encoder, direction);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:  break;
        #endif
        default: break;
    }
}

void Motor_ResetUnitsSensor(Motor_T * p_motor)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Motor_ResetUnitsHallEncoder(p_motor);
            Motor_ResetUnitsEncoder(p_motor);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            Motor_ResetUnitsEncoder(p_motor);
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            Motor_ResetUnitsSinCos(&p_motor->SinCos);
            Motor_ResetUnitsAngleSpeed_Mech(&p_motor);
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            Motor_ResetUnitsAngleSpeed_ElecControl(p_motor);
            break;
        #endif
        default:
            break;
    }
}

/******************************************************************************/
/*
    Pid State on FeedbackMode change
*/
/******************************************************************************/
void Motor_MatchSpeedFeedbackState(Motor_T * p_motor, int32_t speedOutput)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        Ramp_SetOutputState(&p_motor->Ramp, p_motor->Speed_Fract16);
        PID_SetOutputState(&p_motor->PidSpeed, speedOutput);
    }
    else
    {
        Ramp_SetOutputState(&p_motor->Ramp, speedOutput);
    }
}

/* as common super function */

static void ApplySpeedLimit(Motor_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U) /* speed limit is applied on input */
    {
        // Ramp_SetTarget(&p_motor->Ramp, math_clamp(Ramp_GetTarget(&p_motor->Ramp), (int32_t)0 - speed_ufract16, speed_ufract16)); /* clamp the req until the next input */
        Ramp_SetTarget(&p_motor->Ramp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->Ramp))); /* clamp the req until the next input */
    }
    /* else speed limit is applied on feedback */
}

/* propagate limit set */
static void ApplyILimit(Motor_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)  /* limit is applied on feedback */
    {
        if (p_motor->FeedbackMode.Current == 1U) /* SpeedPid Output is I */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
            // if (p_motor->Direction == MOTOR_DIRECTION_CCW)  { PID_SetOutputLimits(&p_motor->PidSpeed, 0 - p_motor->ILimitGenerating_Fract16, p_motor->ILimitMotoring_Fract16); }
            // else                                            { PID_SetOutputLimits(&p_motor->PidSpeed, 0 - p_motor->ILimitMotoring_Fract16, p_motor->ILimitGenerating_Fract16); }
        }
        else /* SpeedPid Output is V */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor));
            // if (p_motor->Direction == MOTOR_DIRECTION_CCW)  { PID_SetOutputLimits(&p_motor->PidSpeed, 0, INT16_MAX); }
            // else                                            { PID_SetOutputLimits(&p_motor->PidSpeed, INT16_MIN, 0); }
        }
    }
    else /* limit is applied on input */
    {
        // Ramp_SetTarget(&p_motor->Ramp, math_clamp(Ramp_GetTarget(&p_motor->Ramp), (int32_t)0 - i_ufract16, i_ufract16)); /* clamp the req until the next input */
        Ramp_SetTarget(&p_motor->Ramp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->Ramp))); /* clamp the req until the next input */
    }
}




/* Call from StateMachine only, may update feedback limits after */
void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // if (mode.Word != p_motor->FeedbackMode.Word) // propagate limits updated in the previous mode
    {
        p_motor->FeedbackMode.Value = mode.Value;
        ApplyILimit(p_motor);
        ApplySpeedLimit(p_motor);
    }
}

void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, uint8_t modeValue)
{
    Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(modeValue));
}


/******************************************************************************/
/*
    Direction functions - Call by StateMachine
    applied voltage direcction
*/
/******************************************************************************/
// void Motor_SetDirectionCcw(Motor_T * p_motor)
// {
//     p_motor->Direction = MOTOR_DIRECTION_CCW;
//     // alternatively, common function with repeat check direction logic
//     ApplyILimit(p_motor);
//     ApplySpeedLimit(p_motor);
//     Motor_SetSensorCcw(p_motor);
// }

// void Motor_SetDirectionCw(Motor_T * p_motor)
// {
//     p_motor->Direction = MOTOR_DIRECTION_CW;
//     ApplyILimit(p_motor);
//     ApplySpeedLimit(p_motor);
//     Motor_SetSensorCw(p_motor);
// }

/*
*/
void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    p_motor->Direction = direction;
    Motor_SetSensorDirection(p_motor, direction);
    ApplyILimit(p_motor);
    ApplySpeedLimit(p_motor);
}

void Motor_SetDirection_Cast(Motor_T * p_motor, uint8_t direction)
{
    Motor_SetDirection(p_motor, (Motor_Direction_T)direction);
}

/*
    Forward/Reverse using calibration param
*/
void Motor_SetDirectionForward(Motor_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionForward(p_motor)); }
void Motor_SetDirectionReverse(Motor_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }


/******************************************************************************/
/*!
    Sentinel Active Limits
    Unconditional Set - Inner set, always overwrite.
    Compared on SetUserCmd. Derive directional Feedback Limits
*/
/******************************************************************************/
/*
    Speed Limit
*/

// optionally derive cw/ccw limits first
// void Motor_SetSpeedLimitForward(Motor_T * p_motor, uint16_t speed_ufract16)
// {
//     p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
//     // if (Motor_IsDirectionForward(p_motor) == true) { ApplySpeedLimit(p_motor, speed_ufract16); }
// }

// void Motor_SetSpeedLimitReverse(Motor_T * p_motor, uint16_t speed_ufract16)
// {
//     p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
//     // if (Motor_IsDirectionReverse(p_motor) == true) { ApplySpeedLimit(p_motor, speed_ufract16); }
// }

void Motor_SetSpeedLimit(Motor_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
    p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
    ApplySpeedLimit(p_motor);
}

/* As scalar of base config */
void Motor_SetSpeedLimit_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = fract16_mul(p_motor->Config.SpeedLimitForward_Fract16, scalar_ufract16);
    p_motor->SpeedLimitReverse_Fract16 = fract16_mul(p_motor->Config.SpeedLimitReverse_Fract16, scalar_ufract16);
    ApplySpeedLimit(p_motor);
}

void Motor_ClearSpeedLimit(Motor_T * p_motor)
{
    p_motor->SpeedLimitForward_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    p_motor->SpeedLimitReverse_Fract16 = p_motor->Config.SpeedLimitReverse_Fract16;
}

static uint16_t SpeedLimitSentinelOf(const Motor_T * p_motor, uint16_t speed_ufract16)
{
    return (p_motor->Direction == p_motor->Config.DirectionForward) ?
        math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16) : math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
}



/*
    ILimit

    I_MAX 200A
    User Base 100A => 100A/200A << 16 = [32768] => Cw/Ccw = 16384

    SetILimitActive 90A => 90A/200A << 16 = [29491] => Cw/Ccw = 14745
    SetILimitActive 110A => Clamp 100A
*/
// void Motor_SetILimitMotoring(Motor_T * p_motor, uint16_t i_Fract16)
// {
//     p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
//     ApplyILimit(p_motor);
// }

// void Motor_SetILimitGenerating(Motor_T * p_motor, uint16_t i_Fract16)
// {
//     p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitGenerating_Fract16);
//     ApplyILimit(p_motor);
// }

void Motor_SetILimit(Motor_T * p_motor, uint16_t i_Fract16)
{
    p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
    p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitGenerating_Fract16);
    ApplyILimit(p_motor);
}

/*
    Scalar 50000 => 50000 * 32768 Base >> 16 = [25000] => Cw/Ccw = 12500 => 25000 * 200A = 76A
*/
void Motor_SetILimitMotoring_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16)
{
    p_motor->ILimitMotoring_Fract16 = fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16);
    // p_motor->ILimitGenerating_Fract16 = fract16_mul(p_motor->Config.ILimitGenerating_Fract16, scalar_ufract16);
    ApplyILimit(p_motor);
}

/* scale to Motoring as limit, clamp both. */
void Motor_SetILimit_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16)
{
    Motor_SetILimit(p_motor, fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16));
}

void Motor_ClearILimit(Motor_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
    ApplyILimit(p_motor);
}

static uint16_t ILimitMotoringSentinelOf(const Motor_T * p_motor, uint16_t i_Fract16)
{
    return math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
}

static uint16_t ILimitMotoringSentinelOf_Scalar(const Motor_T * p_motor, uint16_t scalar_ufract16)
{
    return fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16);
}

/*  */


void Motor_ApplyLimitSpeed(Motor_T * p_motor, Limit_T * p_limit)
{
    if (Limit_IsUpperActive(p_limit) == true) { Motor_SetSpeedLimit(p_motor, Limit_GetUpper(p_limit)); }
    // else                                        { Motor_ClearSpeedLimit(p_motor); }
}


// todo move to outer module
/******************************************************************************/
/*
    Conditional -
        Set Sentinel and propagate to effective limits
    Each source use unique entry id
    returns true if the selected id becomes the active id
*/
/******************************************************************************/

bool Motor_SetSpeedLimitEntry(Motor_T * p_motor, uint8_t id, uint16_t speed_ufract16)
{
    uint16_t speedLimit = SpeedLimitSentinelOf(p_motor, speed_ufract16);
    bool isActiveLimit = Limit_Entry_Set(&p_motor->SpeedLimit, id, speedLimit);
    if (isActiveLimit == true) { Motor_SetSpeedLimit(p_motor, speedLimit); }
    return isActiveLimit;
}

// bool Motor_SetSpeedLimitEntry_Scalar(Motor_T * p_motor, uint8_t id, uint16_t scalar_ufract16)
// {
//     // int32_t speed_ufract16 = fract16_mul(Motor_GetSpeedLimit(p_motor), scalar_ufract16); // of base
//     // bool isActiveLimit = Limit_Entry_Set(&p_motor->SpeedLimit, id, speed_ufract16);
//     // if (isActiveLimit == true) { Motor_SetSpeedLimit_Scalar(p_motor, scalar_ufract16); }
//     // return isActiveLimit;
// }

bool Motor_ClearSpeedLimitEntry(Motor_T * p_motor, uint8_t id)
{
    bool isActiveLimit = Limit_Entry_Clear(&p_motor->SpeedLimit, id);
    if(isActiveLimit == true)
    {
        if (Limit_IsUpperActive(&p_motor->SpeedLimit) == true)
            { Motor_SetSpeedLimit(p_motor, Limit_GetUpper(&p_motor->SpeedLimit)); }
        else
            { Motor_ClearSpeedLimit(p_motor); }
    }
    return isActiveLimit;
}

/*
*/
/*! @return true if set */
bool Motor_SetILimitMotoringEntry(Motor_T * p_motor, uint8_t id, uint16_t i_ufract16)
{
    int32_t iLimit = ILimitMotoringSentinelOf(p_motor, i_ufract16);
    bool isActiveLimit = Limit_Entry_Set(&p_motor->ILimit, id, iLimit);
    if (isActiveLimit == true) { Motor_SetILimit(p_motor, iLimit); } /* alteratively maintain scalar comparison satisfy both sides */
    return isActiveLimit;
}

bool Motor_SetILimitMotoringEntry_Scalar(Motor_T * p_motor, uint8_t id, uint16_t scalar_ufract16)
{
    int32_t iLimit = ILimitMotoringSentinelOf_Scalar(p_motor, scalar_ufract16);
    bool isActiveLimit = Limit_Entry_Set(&p_motor->ILimit, id, iLimit);
    if (isActiveLimit == true) { Motor_SetILimit_Scalar(p_motor, iLimit); }
    return isActiveLimit;
}

/*!
    Restores previous limit
    @param[in] id Motor_ILimitId_T
    @return true if cleared. ILimit of input id
*/
bool Motor_ClearILimitMotoringEntry(Motor_T * p_motor, uint8_t id)
{
    bool isActiveLimit = Limit_Entry_Clear(&p_motor->ILimit, id);
    if (isActiveLimit == true)
    {
        if (Limit_IsUpperActive(&p_motor->ILimit) == true)
            { Motor_SetILimit(p_motor, Limit_GetUpper(&p_motor->ILimit)); }
        else
            { Motor_ClearILimit(p_motor); }
    }
    return isActiveLimit;
}


/******************************************************************************/
/*
    Propagate Config values
*/
/******************************************************************************/
void Motor_ResetSpeedLimitActive(Motor_T * p_motor)
{
    Limit_ClearAll(&p_motor->SpeedLimit);
    Motor_ClearSpeedLimit(p_motor);

}

void Motor_ResetILimitActive(Motor_T * p_motor)
{
    Limit_ClearAll(&p_motor->ILimit);
    Motor_ClearILimit(p_motor);
}


/******************************************************************************/
/* Reset fixed point unit conversion structs */
/******************************************************************************/
// void Motor_ResetSpeedFeedbackRef(Motor_T * p_motor)
// {
//     p_motor->SpeedFeedbackRef_Rpm = Motor_GetSpeedVRef_Rpm(p_motor);
// }

void Motor_ResetUnitsIabc(Motor_T * p_motor)
{
    Motor_ResetUnitsIa(p_motor);
    Motor_ResetUnitsIb(p_motor);
    Motor_ResetUnitsIc(p_motor);
}

void Motor_ResetUnitsIa(Motor_T * p_motor)
{
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIa, p_motor->Config.IaZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
    Linear_ADC_SetInverted(&p_motor->UnitsIa);
#endif
}

void Motor_ResetUnitsIb(Motor_T * p_motor)
{
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIb, p_motor->Config.IbZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
    Linear_ADC_SetInverted(&p_motor->UnitsIb);
#endif
}

void Motor_ResetUnitsIc(Motor_T * p_motor)
{
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIc, p_motor->Config.IcZeroRef_Adcu, Motor_GetIPeakRef_Adcu(p_motor));
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
    Linear_ADC_SetInverted(&p_motor->UnitsIc);
#endif
}

void Motor_ResetUnitsVabc(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    Linear_ADC_Init(&p_motor->UnitsVabc, 0U, MotorStatic.VSourceRef_Adcu);
#else
    (void)p_motor;
#endif
}



/* SinCos, Mechanical Rotation Sensor */
void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor)
{
    // Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitsAngleRpm, MOTOR_STATIC.CONTROL_FREQ, 16U, p_motor->Config.PolePairs, Motor_GetSpeedVRef_Rpm(p_motor));
}

void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor)
{
    // Linear_Speed_InitAngleRpm(&p_motor->UnitsAngleRpm, 1000U, 16U, Motor_GetSpeedVRef_Rpm(p_motor));
}

/*
    Reset Sensors, propagate Motor Config to sensor module independent params,
    Sync to motor module params
*/
/*!
    Hall sensors as speed encoder.

    MECH_R = ELECTRIC_R / N_POLE_PAIRS
    ELECTRIC_R = 6 COMMUTATION_STEPS
    Capture Pulse Per Commutation,     CPR = PolePairs*6     => GetSpeed reflects mechanical speed
    Capture Pulse Per Commutation,     CPR = PolePairs      => GetSpeed reflects electrical speed
*/
void Motor_ResetUnitsHallEncoder(Motor_T * p_motor)
{
    if(p_motor->Config.PolePairs * 6U != p_motor->Encoder.Config.CountsPerRevolution)
    {
        Encoder_SetCountsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs * 6U);
    }
    p_motor->Encoder.Config.IsQuadratureCaptureEnabled = false;
}

/* Common, Set after PolePairs */
void Motor_ResetUnitsEncoder(Motor_T * p_motor)
{
    if (Motor_GetSpeedVRef_Rpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
    {
        Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedVRef_Rpm(p_motor));
    }
    if (p_motor->Config.PolePairs != p_motor->Encoder.Config.InterpolateAngleScalar) /* Set for electrical cycle */
    {
        Encoder_DeltaT_SetInterpolateAngleScalar(&p_motor->Encoder, p_motor->Config.PolePairs);
    }
    // if(p_motor->Config.GearRatioOutput != p_motor->Encoder.Config.GearRatioOutput) ||
    // {
    //     Encoder_SetSurfaceRatio(&p_motor->Encoder, p_motor->Config.GearRatio);
    // }
}

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_ResetUnitsSinCos(Motor_T * p_motor)
{
    if(p_motor->Config.PolePairs != p_motor->SinCos.Config.ElectricalRotationsRatio)
    {
        SinCos_SetAngleRatio(&p_motor->SinCos, p_motor->Config.PolePairs);
    }
}
#endif




/******************************************************************************/
/*
    Cw, Ccw use by Control loop
*/
/******************************************************************************/
// static void UpdateILimitsCcw(Motor_T * p_motor)
// // static void UpdateILimitsCcw(Motor_T * p_motor, fract16_t motoring, fract16_t generating)
// {
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitMotoring_Fract16  ;
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitGenerating_Fract16  );
// }

// static void UpdateILimitsCw(Motor_T * p_motor)
// {
//     p_motor->ILimitCw_Fract16 = (int32_t)0 - (p_motor->ILimitMotoring_Fract16  );
//     p_motor->ILimitCcw_Fract16 = p_motor->ILimitGenerating_Fract16  ;
// }

// static void UpdateSpeedLimitsCcw(Motor_T * p_motor)
// {
//     int32_t ccw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Fract16 : p_motor->SpeedLimitReverse_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = ccw  ;
//     p_motor->SpeedLimitCw_Fract16 = 0;
// }

// static void UpdateSpeedLimitsCw(Motor_T * p_motor)
// {
//     int32_t cw = (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitReverse_Fract16 : p_motor->SpeedLimitForward_Fract16;
//     p_motor->SpeedLimitCcw_Fract16 = 0;
//     p_motor->SpeedLimitCw_Fract16 = (int16_t)0 - (cw  );
// }

/*
    UpdateLimits on Limits change
*/
// static void UpdateILimits(Motor_T * p_motor)
// {
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { UpdateILimitsCcw(p_motor); } else { UpdateILimitsCw(p_motor); }

//     if ((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 1U)) /* Only when SpeedPid Output is I */
//         { PID_SetOutputLimits(&p_motor->PidSpeed, p_motor->ILimitCw_Fract16, p_motor->ILimitCcw_Fract16); }
// }

// static void UpdateSpeedLimits(Motor_T * p_motor)
// {
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { UpdateSpeedLimitsCcw(p_motor); } else { UpdateSpeedLimitsCw(p_motor); }
//     // set ramp limits
// }



/*
    Set on Direction change
*/
// static void UpdateDirectionLimitsCcw(Motor_T * p_motor)
// {
//     UpdateSpeedLimitsCcw(p_motor);
//     UpdateILimitsCcw(p_motor);
//     if ((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 0U))
//         { PID_SetOutputLimits(&p_motor->PidSpeed, 0, INT16_MAX); }  /* Speed PID Output is V */
// }

// static void UpdateDirectionLimitsCw(Motor_T * p_motor)
// {
//     UpdateSpeedLimitsCw(p_motor);
//     UpdateILimitsCw(p_motor);
//     if ((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 0U))
//         { PID_SetOutputLimits(&p_motor->PidSpeed, INT16_MIN, 0); }
// }
