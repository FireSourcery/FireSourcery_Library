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

/*

*/
void Motor_Init(Motor_T * p_motor)
{
    // assert(MotorAnalogRef_GetVSource_Fract16() != 0U); /* Must be set before init */

    if (p_motor->CONST.P_NVM_CONFIG != 0U) { memcpy(&p_motor->Config, p_motor->CONST.P_NVM_CONFIG, sizeof(Motor_Config_T)); }
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

    FOC_Init(&p_motor->Foc, MotorAnalogRef_GetVSource_Fract16());
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
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, INT16_MAX);
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, MotorAnalogRef_GetIRatedPeak_Fract16());
    // Ramp_Init(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, INT16_MAX);

// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopSpeedRamp_Cycles, p_motor->Config.OpenLoopSpeedFinal_Fract16); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopIRamp_Cycles, p_motor->Config.OpenLoopIFinal_Fract16);
// #endif

    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);

    Motor_ResetSpeedLimitActive(p_motor);
    Motor_ResetILimitActive(p_motor);

    // p_motor->FeedbackMode.Word = p_motor->Config.FeedbackModeDefault.Word; /* StateMachine init sets initial limits using this value. */
    p_motor->ControlTimerBase = 0U;
}


/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_SetMechAngleFeedforward(Motor_T * p_motor, angle16_t angle)
{
    p_motor->MechanicalAngle = angle;
    p_motor->ElectricalAngle = p_motor->MechanicalAngle * p_motor->Config.PolePairs;
}

void Motor_SetElecAngleFeedforward(Motor_T * p_motor, angle16_t angle)
{
    angle16_t angleDelta = angle - p_motor->ElectricalAngle;
    p_motor->ElectricalAngle = angle;
    p_motor->MechanicalAngle += (angleDelta / p_motor->Config.PolePairs);
}

/******************************************************************************/
/*
    Feedback State
*/
/******************************************************************************/
fract16_t Motor_ProcTorqueRamp(Motor_T * p_motor)
{
    int32_t req = Motor_IReqLimitOf(p_motor, Ramp_ProcOutput(&p_motor->TorqueRamp));
    if (req != Ramp_GetOutput(&p_motor->TorqueRamp)) { Ramp_SetOutput(&p_motor->TorqueRamp, req); }
    return req;
}

fract16_t Motor_ProcSpeedRamp(Motor_T * p_motor)
{
   int32_t speedReq = Motor_SpeedReqLimitOf(p_motor, Ramp_ProcOutput(&p_motor->SpeedRamp));
   if (speedReq != Ramp_GetOutput(&p_motor->SpeedRamp)) { Ramp_SetOutput(&p_motor->SpeedRamp, speedReq); }
   return speedReq;
}

/*
    Ramp input ~100Hz,
    SpeedFeedback update 1000Hz
*/
/* From Input Ramp to ReqQ */
fract16_t Motor_ProcOuterFeedback(Motor_T * p_motor)
{
    if (Motor_PollCaptureSpeed(p_motor) == true)
    {
        if (p_motor->FeedbackMode.Speed == 1U)
        {
            // PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
            PID_ProcPI(&p_motor->PidSpeed, p_motor->Speed_Fract16, Motor_ProcSpeedRamp(p_motor));
        }
        else
        {
            // todo on sign change
            // PID_ProcPI(&p_motor->PidSpeed, p_motor->Speed_Fract16, (int32_t)Motor_GetSpeedLimit(p_motor) * p_motor->Direction);
            // if (Motor_IsSpeedLimitReached(p_motor) == true) { Ramp_SetOutput(&p_motor->TorqueRamp, PID_GetOutput(&p_motor->PidSpeed)); }
        }
    }
}

/*
    Pid State on FeedbackMode/Resume
*/
// void Motor_MatchSpeedFeedbackState(Motor_T * p_motor, int32_t torqueReq)
void Motor_MatchSpeedTorqueState(Motor_T * p_motor, int32_t torqueReq)
{
    if (p_motor->FeedbackMode.Speed == 1U)
    {
        Ramp_SetOutputState(&p_motor->SpeedRamp, p_motor->Speed_Fract16);
        PID_SetOutputState(&p_motor->PidSpeed, torqueReq);
    }
    else
    {
        Ramp_SetOutputState(&p_motor->TorqueRamp, torqueReq);
    }
}


/******************************************************************************/
/*
    Propagate limits Common
    optionally store Cw/Ccw limits
*/
/******************************************************************************/
/* reset input can be removed for on proc */
static void ApplySpeedLimit(Motor_T * p_motor)
{
    // if (p_motor->FeedbackMode.Speed == 1U) /* speed limit is applied on input */
    // {
    //     Ramp_SetTarget(&p_motor->SpeedRamp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->SpeedRamp))); /* clamp the req until the next input */
    // }
    /* else speed limit is applied on feedback */
}

/* only speed loop must update */
static void ApplyILimit(Motor_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)  /* limit is applied on feedback */
    {
        if (p_motor->FeedbackMode.Current == 1U) /* SpeedPid Output is I */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
        }
        // else /* SpeedPid Output is V */
        // {
        //     PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor)); /* as phase voltage if foc */
        // }
    }
//     else /* limit is applied on input */ /* alternatively move to on proc */
//     {
//         if (p_motor->FeedbackMode.Current == 1U)
//         {
//             Ramp_SetTarget(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)));
//         }
//     }
}

/******************************************************************************/
/*
    FeedbackMode
    Call from StateMachine only, may update feedback limits
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    // if (mode.Word != p_motor->FeedbackMode.Word) // propagate limits updated in the previous mode
    {
        p_motor->FeedbackMode.Value = mode.Value;
        ApplyILimit(p_motor);
        ApplySpeedLimit(p_motor);
    }
}

void Motor_SetFeedbackMode_Cast(Motor_T * p_motor, int modeValue) { Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(modeValue)); }


/******************************************************************************/
/*
    Direction functions - Call by StateMachine
    applied voltage direcction
*/
/******************************************************************************/
/*
*/
void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    p_motor->Direction = direction;
    Motor_SetSensorDirection(p_motor, direction);
    ApplyILimit(p_motor);
    ApplySpeedLimit(p_motor);
}

void Motor_SetDirection_Cast(Motor_T * p_motor, int direction) { Motor_SetDirection(p_motor, (Motor_Direction_T)direction); }

/*
    Forward/Reverse using calibration param
*/
void Motor_SetDirectionForward(Motor_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionForward(p_motor)); }
void Motor_SetDirectionReverse(Motor_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }
void Motor_SetUserDirection(Motor_T * p_motor, int sign) { Motor_SetDirection(p_motor, sign * p_motor->Config.DirectionForward); }


/******************************************************************************/
/*!
    Active Limits - Non directional
    Inner Unconditional Set, always overwrite.
    Derive directional Feedback Limits
    optionally store cw/ccw limits first
*/
/******************************************************************************/
void Motor_ResetSpeedLimitActive(Motor_T * p_motor)
{
    p_motor->SpeedLimitForward_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    p_motor->SpeedLimitReverse_Fract16 = p_motor->Config.SpeedLimitReverse_Fract16;
}

void Motor_ResetILimitActive(Motor_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
}

/*
    Speed Limit
*/
void Motor_SetSpeedLimitForward(Motor_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
    // if (Motor_IsDirectionForward(p_motor) == true) { ApplySpeedLimit(p_motor); }
}

void Motor_SetSpeedLimitReverse(Motor_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
    // if (Motor_IsDirectionReverse(p_motor) == true) { ApplySpeedLimit(p_motor); }
}

// void Motor_SetSpeedLimit(Motor_T * p_motor, uint16_t speed_ufract16)
// {
//     p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
//     p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
//     ApplySpeedLimit(p_motor);
// }

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
    ApplySpeedLimit(p_motor);
}


/*
    ILimit
*/
void Motor_SetILimitMotoring(Motor_T * p_motor, uint16_t i_Fract16)
{
    p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
    ApplyILimit(p_motor);
}

void Motor_SetILimitGenerating(Motor_T * p_motor, uint16_t i_Fract16)
{
    p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitGenerating_Fract16);
    ApplyILimit(p_motor);
}

// void Motor_SetILimit(Motor_T * p_motor, uint16_t i_Fract16)
// {
//     p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
//     p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitGenerating_Fract16);
//     ApplyILimit(p_motor);
// }

void Motor_SetILimit_Scalar(Motor_T * p_motor, uint16_t scalar_ufract16)
{
    p_motor->ILimitMotoring_Fract16 = fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16);
    p_motor->ILimitGenerating_Fract16 = fract16_mul(p_motor->Config.ILimitGenerating_Fract16, scalar_ufract16);
    ApplyILimit(p_motor);
}

void Motor_ClearILimit(Motor_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
    ApplyILimit(p_motor);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
bool Motor_TrySpeedLimit(Motor_T * p_motor, uint16_t speed_ufract16)
{
    bool isLimit = false;
    switch (Motor_GetUserDirection(p_motor))
    {
        case 1:
            if (speed_ufract16 > p_motor->SpeedLimitForward_Fract16) { Motor_SetSpeedLimitForward(p_motor, speed_ufract16); isLimit = true; }
            break;
        case -1:
            if (speed_ufract16 > p_motor->SpeedLimitReverse_Fract16) { Motor_SetSpeedLimitReverse(p_motor, speed_ufract16); isLimit = true; }
            break;
        default: break;
    }

    return isLimit;
}

bool Motor_TryILimit(Motor_T * p_motor, uint16_t i_Fract16)
{
    bool isLimit = false;
    if (i_Fract16 < p_motor->ILimitMotoring_Fract16) { p_motor->ILimitMotoring_Fract16 = i_Fract16; isLimit = true; }
    if (i_Fract16 < p_motor->ILimitGenerating_Fract16) { p_motor->ILimitGenerating_Fract16 = i_Fract16; isLimit = true; }
    if (isLimit == true) { ApplyILimit(p_motor); }
    return isLimit;
}


/*
    Interface
    Set using comparison struct
*/
void Motor_SetSpeedLimitWith(Motor_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearSpeedLimit(p_motor); }
}

void Motor_SetILimitWith(Motor_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearILimit(p_motor); }
}

/******************************************************************************/
/*!
todo move
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

/* Control Freq */
angle16_t Motor_PollSensorAngle(Motor_T * p_motor)
{
    angle16_t electricalAngle; /* [0, 65535] maps to negative portions of angle16_t */

    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
        #if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
            if (Hall_PollCaptureSensors(&p_motor->Hall) == true)
            {
                Encoder_SinglePhase_CapturePulse(&p_motor->Encoder); // Encoder_CaptureCount
                Hall_CaptureAngle(&p_motor->Hall);
                // Hall_CaptureDirection(&p_motor->Hall); /* on  direction diff */
            }
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

/*!
    @return Fract16 Q1.15 unsaturated
*/
int32_t Motor_PollSensorSpeed(Motor_T * p_motor)
{
    int32_t speed;
    switch (p_motor->Config.SensorMode)
    {
        /* Using assigned direction */
        case MOTOR_SENSOR_MODE_HALL:
            Encoder_ModeDT_CaptureVelocity(&p_motor->Encoder);
            p_motor->AngularSpeed_DegControl = Encoder_ModeDT_CapturePollingAngle(&p_motor->Encoder);
            speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->Encoder);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_ModeDT_CaptureVelocity(&p_motor->Encoder);
            speed = Encoder_ModeDT_GetScalarVelocity(&p_motor->Encoder);
            break;

        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS: speed = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));    break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS: break;
        #endif
        default: speed = 0; break;
    }
    return speed;
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
    switch (p_motor->Config.SensorMode)
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
void Motor_SetSensorDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Hall_SetDirection(&p_motor->Hall, (Hall_Direction_T)direction);
            Encoder_SinglePhase_SetDirection(&p_motor->Encoder, direction);  /* interpolate as +/- */
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:  break;
        #endif
        default: break;
    }
}

/*
    todo as electrical speed
*/

/*
    Reset Sensors, propagate Motor Config to sensor module independent params,
    Sync to motor module params
*/
void Motor_ResetUnitsHallEncoder_MechSpeed(Motor_T * p_motor)
{
    p_motor->Encoder.Config.IsQuadratureCaptureEnabled = false;

    if (p_motor->Config.PolePairs * 6U != p_motor->Encoder.Config.CountsPerRevolution)
        { Encoder_SetCountsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs * 6U); }

    if (Motor_GetSpeedRatedRef_Rpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
        { Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedRatedRef_Rpm(p_motor)); }

    /* Set for electrical cycle */
    if (p_motor->Config.PolePairs != p_motor->Encoder.Config.PartitionsPerRevolution)
        { Encoder_SetPartitionsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs); }
}

void Motor_ResetUnitsHallEncoder_ElSpeed(Motor_T * p_motor)
{
    p_motor->Encoder.Config.IsQuadratureCaptureEnabled = false;
    if (p_motor->Encoder.Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(&p_motor->Encoder, 6U); }
    if (Motor_GetSpeedRatedRef_ERpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
        { Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedRatedRef_ERpm(p_motor)); }

    Encoder_SetPartitionsPerRevolution(&p_motor->Encoder, 1);
}


void Motor_ResetUnitsEncoder(Motor_T * p_motor)
{
    if (Motor_GetSpeedRatedRef_Rpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
    {
        Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedRatedRef_Rpm(p_motor));
    }
    if (p_motor->Config.PolePairs != p_motor->Encoder.Config.PartitionsPerRevolution) /* Set for electrical cycle */
    {
        Encoder_SetPartitionsPerRevolution(&p_motor->Encoder, p_motor->Config.PolePairs);
    }
    // if(p_motor->Config.GearRatioOutput != p_motor->Encoder.Config.GearRatioOutput) ||
    // {
    //     Encoder_SetSurfaceRatio(&p_motor->Encoder, p_motor->Config.GearRatio);
    // }
}

void Motor_ResetUnitsSensor(Motor_T * p_motor)
{
    switch (p_motor->Config.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Motor_ResetUnitsHallEncoder_MechSpeed(p_motor);
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