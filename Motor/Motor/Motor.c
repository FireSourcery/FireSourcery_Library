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
    Init Global_Motor first
*/
void Motor_Init(MotorPtr_T p_motor)
{
    if(p_motor->CONFIG.P_PARAMS_NVM != 0U) { memcpy(&p_motor->Parameters, p_motor->CONFIG.P_PARAMS_NVM, sizeof(Motor_Params_T)); }
    Motor_InitReboot(p_motor); //move to state machine?
    StateMachine_Init(&p_motor->StateMachine);
}

void Motor_InitReboot(MotorPtr_T p_motor)
{
    /*
        HW Wrappers Init
    */
    Phase_Init(&p_motor->Phase);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_ActivateMode(&p_motor->Phase, p_motor->Parameters.PhasePwmMode);
#endif
    Motor_InitSensor(p_motor);
    Thermistor_Init(&p_motor->Thermistor);

    /*
        SW Structs
    */
    Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
    Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

    FOC_Init(&p_motor->Foc);
    //    BEMF_Init(&p_motor->Bemf);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_Init(&p_motor->PidSpeed);
    PID_Init(&p_motor->PidIq);
    PID_Init(&p_motor->PidId);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
#endif

    /*
        e.g. Ramp 0 to 32767 max in ~500ms, 3.2767 per ControlCycle
        Final value is overwritten, Slope is persistent unless reset
    */
    Linear_Ramp_Init(&p_motor->Ramp, p_motor->Parameters.RampAccel_Cycles, 0, INT16_MAX);
    Linear_Ramp_Init(&p_motor->AuxRamp, p_motor->Parameters.RampAccel_Cycles, 0, 0);
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Linear_Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Parameters.OpenLoopAccel_Cycles, 0, 0);
#endif

    Motor_ResetUnitsVabc(p_motor);
    Motor_ResetUnitsIabc(p_motor);
    Motor_ResetUnitsVSpeed(p_motor);
    Motor_ResetSpeedLimitActive(p_motor);
    Motor_ResetILimitActive(p_motor);

    p_motor->FeedbackMode.Word = p_motor->Parameters.FeedbackModeDefault.Word; /* StateMachine init sets initial limits using this value. */
    p_motor->ControlTimerBase = 0U;
}

void Motor_InitSensor(MotorPtr_T p_motor)
{
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            p_motor->CONFIG.INIT_SENSOR_HALL();  //todo move to hal
            Encoder_ModeDT_Init(&p_motor->Encoder);
            Hall_Init(&p_motor->Hall);
#if defined(CONFIG_MOTOR_HALL_MODE_ISR)
            Encoder_InitInterrupts_ABC(&p_motor->Encoder);
#endif
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            p_motor->CONFIG.INIT_SENSOR_ENCODER(); //todo move to hal
            Encoder_ModeDT_Init(&p_motor->Encoder);
            Encoder_InitInterrupts_Quadrature(&p_motor->Encoder);
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

/******************************************************************************/
/*!
    Position Sensor Feedback - Speed, Angle
*/
/******************************************************************************/
qangle16_t Motor_PollSensorAngle(MotorPtr_T p_motor)
{
    qangle16_t electricalAngle; /* FracU16 [0, 65535] maps to negative portions of qangle16_t */

    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
#if defined(CONFIG_MOTOR_HALL_MODE_POLLING)
            if(Hall_PollCaptureRotorAngle(&p_motor->Hall) == true) { Encoder_CapturePulse(&p_motor->Encoder); } //Encoder_CapturePulse_SinglePhase
#endif
            electricalAngle = Hall_GetRotorAngle_Degrees16(&p_motor->Hall);
            electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
            break;

        case MOTOR_SENSOR_MODE_ENCODER:
            electricalAngle = Encoder_GetAngle_Scalar(&p_motor->Encoder, p_motor->Parameters.PolePairs); /* ElectricalAngle => MechanicalAngle * PolePairs */
            // electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
            break;

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
            electricalAngle = SinCos_GetElectricalAngle(&p_motor->SinCos);
            //todo group
            AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
            AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
            break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:
            //todo observer
            electricalAngle = 0;
            p_motor->FeedbackMode.OpenLoop = 1U;
            p_motor->FeedbackMode.OpenLoop = 1U;
            p_motor->StatusFlags.SensorFeedback = 0U;
            p_motor->StatusFlags.SensorFeedback = 0U;
            break;
#endif
        default: electricalAngle = 0; break;
    }

    return electricalAngle;
}

qangle16_t Motor_GetMechanicalAngle(const MotorPtr_T p_motor)
{
    qangle16_t angle;
    switch(p_motor->Parameters.SensorMode)
    {
        // case MOTOR_SENSOR_MODE_HALL:    angle = Encoder_GetAngle(&p_motor->Encoder);    break;
        case MOTOR_SENSOR_MODE_ENCODER: angle = Encoder_GetAngle(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     angle = SinCos_GetMechanicalAngle(&p_motor->SinCos);     break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:  angle = 0;     break;
#endif
        default:                             angle = 0;     break;
    }
    return angle;
}

/* For SinCos, Sensorless, when not using Encoder module */
// static inline int32_t PollAngleSpeed(MotorPtr_T p_motor, qangle16_t speedAngle)
// {
//     int32_t speedDelta = speedAngle - p_motor->SpeedAngle; /* loops if no overflow past 1 full cycle */
//     int32_t speedFeedback_Frac16 = (p_motor->Speed_FracS16 + Linear_Speed_CalcAngleRpmFrac16(&p_motor->UnitsAngleRpm, speedDelta)) / 2;
//     p_motor->SpeedAngle = speedAngle; /* mechanical angle */
//     return speedFeedback_Frac16;
// }

/* returns Encoder return Scalar16[-65536:65536] as [-1:1] unsaturated */
int32_t Motor_PollSensorSpeed(MotorPtr_T p_motor)
{
    int32_t speed_Fixed32;
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:    speed_Fixed32 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); break;
        case MOTOR_SENSOR_MODE_ENCODER: speed_Fixed32 = Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder); break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:        speed_Fixed32 = PollAngleSpeed(p_motor, SinCos_GetMechanicalAngle(&p_motor->SinCos));    break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS: break;
#endif
        default: speed_Fixed32 = 0; break;
    }
    return speed_Fixed32;
}

bool Motor_ProcSensorSpeed(MotorPtr_T p_motor)
{
    bool procSpeed = Timer_Periodic_Poll(&p_motor->SpeedTimer);
    if(procSpeed == true) { p_motor->Speed_FracS16 = ((Motor_PollSensorSpeed(p_motor) / 2) + p_motor->Speed_FracS16) / 2; }
    return procSpeed;
}


/******************************************************************************/
/*
    Reset Sensors/Align
*/
/******************************************************************************/
void Motor_ZeroSensor(MotorPtr_T p_motor)
{
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:
            Hall_SetInitial(&p_motor->Hall);
            Encoder_ModeDT_SetInitial(&p_motor->Encoder);
            break;
        case MOTOR_SENSOR_MODE_ENCODER:
            Encoder_ModeDT_SetInitial(&p_motor->Encoder);
            break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:        break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:            Motor_SetPositionFeedback(p_motor, 0U);        break;
#endif
        default: break;
    }
}

/* From Stop and after Align */
bool Motor_CheckSensorFeedback(const MotorPtr_T p_motor)
{
    bool isAvailable;
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL:    isAvailable = true;        break;
        case MOTOR_SENSOR_MODE_ENCODER: isAvailable = Encoder_GetIsAligned(&p_motor->Encoder);    break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:     isAvailable = true;        break;
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     isAvailable = false;    break;
#endif
        default: isAvailable = false; break;
    }
    return isAvailable;
}


static inline bool _Motor_CheckOpenLoop(const MotorPtr_T p_motor)
{
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    return (p_motor->FeedbackMode.OpenLoop == 1U);
#else
    (void)p_motor; return false;
#endif
}

/* User request or no sensor feedback */
bool Motor_CheckOpenLoop(const MotorPtr_T p_motor)
{
    return ((Motor_CheckSensorFeedback(p_motor) == false) || (_Motor_CheckOpenLoop(p_motor) == true));
}

/******************************************************************************/
/*
   Limits
*/
/******************************************************************************/
// static inline bool Motor_CheckSpeedOverLimit(MotorPtr_T p_motor)
// {
//     return !math_isbound(p_motor->Speed_FracS16, p_motor->SpeedLimitCw_FracS16, p_motor->SpeedLimitCcw_FracS16);
// }

int32_t Motor_GetSpeedLimitReq(const MotorPtr_T p_motor)
{
    int32_t req = Linear_Ramp_GetOutput(&p_motor->Ramp);

    if(p_motor->FeedbackMode.Speed == 1U)
    {
        // if(p_motor->Speed_FracS16 < p_motor->SpeedLimitCw_FracS16)          { req = p_motor->SpeedLimitCw_FracS16; }
        // else if(p_motor->Speed_FracS16 > p_motor->SpeedLimitCcw_FracS16)    { req = p_motor->SpeedLimitCcw_FracS16; }
        // else                                                                { req = Linear_Ramp_GetOutput(&p_motor->Ramp); }
        req = math_clamp(req, p_motor->SpeedLimitCw_FracS16, p_motor->SpeedLimitCcw_FracS16);
    }
    else
    {
        if(p_motor->Speed_FracS16 < p_motor->SpeedLimitCw_FracS16)          { req = 0 - (p_motor->SpeedLimitCw_FracS16 / p_motor->Speed_FracS16); }
        else if(p_motor->Speed_FracS16 > p_motor->SpeedLimitCcw_FracS16)    { req = p_motor->SpeedLimitCcw_FracS16 / p_motor->Speed_FracS16; }
        // else                                                                { req = Linear_Ramp_GetOutput(&p_motor->Ramp); }
        //todo flag for threshold, match output on return to user cmd
        //alternatively enable PID
        // p_motor->FeedbackMode.Speed = 1U;
        // MatchOutput
    }

    return req;
}

int32_t Motor_GetILimitReq(const MotorPtr_T p_motor,  int32_t req, int32_t feedback)
{
    if(p_motor->FeedbackMode.Current == 1U)
    {
        if(feedback < p_motor->ILimitCw_FracS16)          { req = p_motor->ILimitCw_FracS16; }
        else if(feedback > p_motor->ILimitCcw_FracS16)    { req = p_motor->ILimitCcw_FracS16; }
    }
    else
    {
        if(feedback < p_motor->ILimitCw_FracS16)          { req = 0 - (p_motor->ILimitCw_FracS16 / feedback); }
        else if(feedback > p_motor->ILimitCcw_FracS16)    { req = p_motor->ILimitCcw_FracS16 / feedback; }
        //todo flag for threshold, match output on return to user cmd
        // PID_SetOutputState(&p_motor->PidIq, FOC_GetVq(&p_motor->Foc));
        // PID_SetOutputState(&p_motor->PidId, FOC_GetVd(&p_motor->Foc));
        //alternatively enable PID
        // p_motor->FeedbackMode.I = 1U;
        // MatchOutput
    }

    return req;
}

static void UpdateILimitsCcw(MotorPtr_T p_motor)
{
    p_motor->ILimitCcw_FracS16 = p_motor->ILimitMotoring_Scalar16 / 2;
    p_motor->ILimitCw_FracS16 = (int16_t)0 - p_motor->ILimitGenerating_Scalar16 / 2;
    if((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 1U))      /* Only when SpeedPid Output is I */
        { PID_SetOutputLimits(&p_motor->PidSpeed, (int16_t)0 - p_motor->ILimitGenerating_Scalar16 / 2, p_motor->ILimitMotoring_Scalar16 / 2); }
}

static void UpdateILimitsCw(MotorPtr_T p_motor)
{
    p_motor->ILimitCw_FracS16 = (int16_t)0 - p_motor->ILimitMotoring_Scalar16 / 2;
    p_motor->ILimitCcw_FracS16 = p_motor->ILimitGenerating_Scalar16 / 2;
    if((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 1U))
        { PID_SetOutputLimits(&p_motor->PidSpeed, (int16_t)0 - p_motor->ILimitMotoring_Scalar16 / 2, p_motor->ILimitGenerating_Scalar16 / 2); }
}

static void UpdateSpeedLimitsCcw(MotorPtr_T p_motor)
{
    p_motor->SpeedLimitDirect_Scalar16 =
        (p_motor->Parameters.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitForward_Scalar16 : p_motor->SpeedLimitReverse_Scalar16;
    p_motor->SpeedLimitCcw_FracS16 = p_motor->SpeedLimitDirect_Scalar16 / 2;
    p_motor->SpeedLimitCw_FracS16 = 0;
}

static void UpdateSpeedLimitsCw(MotorPtr_T p_motor)
{
    p_motor->SpeedLimitDirect_Scalar16 =
        (p_motor->Parameters.DirectionForward == MOTOR_DIRECTION_CCW) ? p_motor->SpeedLimitReverse_Scalar16 : p_motor->SpeedLimitForward_Scalar16;
    p_motor->SpeedLimitCw_FracS16 = (int16_t)0 - p_motor->SpeedLimitDirect_Scalar16 / 2;
    p_motor->SpeedLimitCcw_FracS16 = 0;
}

/*
    UpdateLimits
    Set on Limits change
*/
void Motor_UpdateFeedbackILimits(MotorPtr_T p_motor)
{
    if(p_motor->Direction == MOTOR_DIRECTION_CCW) { UpdateILimitsCcw(p_motor); } else { UpdateILimitsCw(p_motor); }
}

void Motor_UpdateFeedbackSpeedLimits(MotorPtr_T p_motor)
{
    if(p_motor->Direction == MOTOR_DIRECTION_CCW) { UpdateSpeedLimitsCcw(p_motor); } else { UpdateSpeedLimitsCw(p_motor); }
}

/******************************************************************************/
/*
    Direction functions - Call by StateMachine
*/
/******************************************************************************/
/*
    Set on Direction change
*/
static void Motor_SetFeedbackLimitsCcw(MotorPtr_T p_motor)
{
    UpdateSpeedLimitsCcw(p_motor);
    UpdateILimitsCcw(p_motor);
    if((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 0U))
        { PID_SetOutputLimits(&p_motor->PidSpeed, 0, INT16_MAX); }     /* Speed PID Output is V */
}

static void Motor_SetFeedbackLimitsCw(MotorPtr_T p_motor)
{
    UpdateSpeedLimitsCw(p_motor);
    UpdateILimitsCw(p_motor);
    if((p_motor->FeedbackMode.Speed == 1U) && (p_motor->FeedbackMode.Current == 0U))
        { PID_SetOutputLimits(&p_motor->PidSpeed, INT16_MIN, 0); }
}

void Motor_SetDirectionCcw(MotorPtr_T p_motor)
{
    p_motor->Direction = MOTOR_DIRECTION_CCW;
    Motor_SetFeedbackLimitsCcw(p_motor);
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL: Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW); Encoder_SetSinglePhaseDirection(&p_motor->Encoder, true); break;
        case MOTOR_SENSOR_MODE_ENCODER:     break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     break;
#endif
        default: break;
    }
}

void Motor_SetDirectionCw(MotorPtr_T p_motor)
{
    p_motor->Direction = MOTOR_DIRECTION_CW;
    Motor_SetFeedbackLimitsCw(p_motor);
    switch(p_motor->Parameters.SensorMode)
    {
        case MOTOR_SENSOR_MODE_HALL: Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW); Encoder_SetSinglePhaseDirection(&p_motor->Encoder, false); break;
        case MOTOR_SENSOR_MODE_ENCODER:     break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:  break;
#endif
        default: break;
    }
}

/*
    CCW or CW
*/
void Motor_SetDirection(MotorPtr_T p_motor, Motor_Direction_T direction)
{
    if(direction == MOTOR_DIRECTION_CCW) { Motor_SetDirectionCcw(p_motor); } else { Motor_SetDirectionCw(p_motor); }
}

/*
    Forward/Reverse using calibration param
*/
void Motor_SetDirectionForward(MotorPtr_T p_motor) { Motor_SetDirection(p_motor, Motor_DirectionForward(p_motor)); }
void Motor_SetDirectionReverse(MotorPtr_T p_motor) { Motor_SetDirection(p_motor, Motor_DirectionReverse(p_motor)); }

/******************************************************************************/
/*
    Propagate param values
*/
/******************************************************************************/
void Motor_ResetSpeedLimitActive(MotorPtr_T p_motor)
{
    p_motor->SpeedLimitForward_Scalar16 = p_motor->Parameters.SpeedLimitForward_Scalar16;
    p_motor->SpeedLimitReverse_Scalar16 = p_motor->Parameters.SpeedLimitReverse_Scalar16;
    p_motor->SpeedLimitDirect_Scalar16 = UINT16_MAX;
    for(uint8_t idIndex = 0U; idIndex < sizeof(p_motor->SpeedLimitBuffer) / sizeof(p_motor->SpeedLimitBuffer[0]); idIndex++)
        { p_motor->SpeedLimitBuffer[idIndex] = UINT16_MAX; }
}

void Motor_ResetILimitActive(MotorPtr_T p_motor)
{
    p_motor->ILimitMotoring_Scalar16 = p_motor->Parameters.ILimitMotoring_Scalar16;
    p_motor->ILimitGenerating_Scalar16 = p_motor->Parameters.ILimitGenerating_Scalar16;
    p_motor->ILimitActiveSentinel_Scalar16 = UINT16_MAX;
    for(uint8_t idIndex = 0U; idIndex < sizeof(p_motor->ILimitBuffer) / sizeof(p_motor->ILimitBuffer[0]); idIndex++)
        { p_motor->ILimitBuffer[idIndex] = UINT16_MAX; }
}

void Motor_ResetUnitsSensor(MotorPtr_T p_motor)
{
    switch(p_motor->Parameters.SensorMode)
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

void Motor_ResetUnitsVabc(MotorPtr_T p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    Linear_Voltage_Init(&p_motor->UnitsVabc, GLOBAL_MOTOR.V_ABC_R1, GLOBAL_MOTOR.V_ABC_R2, GLOBAL_ANALOG.ADC_BITS, GLOBAL_ANALOG.ADC_VREF_MILLIV, Global_Motor_GetVSource_V());
#else
    (void)p_motor;
#endif
}

void Motor_ResetUnitsIabc(MotorPtr_T p_motor)
{
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
    uint16_t iPeakRef_Adcu = p_motor->Parameters.IPeakRef_Adcu;
#elif defined(CONFIG_MOTOR_DEBUG_DISABLE)
    uint16_t iPeakRef_Adcu = GLOBAL_MOTOR.I_MAX_ADCU;
#endif
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIa, p_motor->Parameters.IaZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_MAX_AMPS);
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIb, p_motor->Parameters.IbZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_MAX_AMPS);
    Linear_ADC_Init_ZeroToPeak(&p_motor->UnitsIc, p_motor->Parameters.IcZeroRef_Adcu, iPeakRef_Adcu, 0, GLOBAL_MOTOR.I_MAX_AMPS);
#ifdef CONFIG_MOTOR_I_SENSORS_INVERT
    Linear_ADC_SetInverted(&p_motor->UnitsIa);
    Linear_ADC_SetInverted(&p_motor->UnitsIb);
    Linear_ADC_SetInverted(&p_motor->UnitsIc);
#endif
}

/*
   when SpeedFeedbackRef_Rpm = Kv * VRef_V => Speed_Frac16 = V_Frac16
        V = RPM / Kv
        V_Frac16 / 32767 * VRef_V = Speed_FracS16 / 32767 * SpeedFeedbackRef_Rpm / Kv
        V_Frac16 = Speed_FracS16 * SpeedFeedbackRef_Rpm * VRef_V / Kv
*/
void Motor_ResetUnitsVSpeed(MotorPtr_T p_motor)
{
    Linear_Frac16_Init(&p_motor->UnitsVSpeed, 0, p_motor->Parameters.SpeedFeedbackRef_Rpm * INT16_MAX / p_motor->Parameters.VSpeedRef_Rpm, 0, 0);
    // Linear_Init(&p_motor->UnitsVSpeed, p_motor->Parameters.VSpeedRef_Rpm, p_motor->Parameters.SpeedFeedbackRef_Rpm, 0, INT16_MAX);
}

/* SinCos, Mechanical Rotation Sensor */
void Motor_ResetUnitsAngleSpeed_ElecControl(MotorPtr_T p_motor)
{
    // Linear_Speed_InitElectricalAngleRpm(&p_motor->UnitsAngleRpm, GLOBAL_MOTOR.CONTROL_FREQ, 16U, p_motor->Parameters.PolePairs, p_motor->Parameters.SpeedFeedbackRef_Rpm);
}

void Motor_ResetUnitsAngleSpeed_Mech(MotorPtr_T p_motor)
{
    // Linear_Speed_InitAngleRpm(&p_motor->UnitsAngleRpm, 1000U, 16U, p_motor->Parameters.SpeedFeedbackRef_Rpm);
}

/*
    Reset Sensors, propagate Motor Params to sensor module independent params,
    Sync to motor module params
*/
/*!
    Hall sensors as speed encoder.

    MECH_R = ELECTRIC_R / N_POLE_PAIRS
    ELECTRIC_R = 6 COMMUTATION_STEPS
    Capture Pulse Per Commutation,     CPR = PolePairs*6     => GetSpeed reflects mechanical speed
    Capture Pulse Per Commutation,     CPR = PolePairs      => GetSpeed reflects electrical speed
*/
void Motor_ResetUnitsHallEncoder(MotorPtr_T p_motor)
{
    if(p_motor->Parameters.PolePairs * 6U != p_motor->Encoder.Params.CountsPerRevolution)
    {
        Encoder_SetCountsPerRevolution(&p_motor->Encoder, p_motor->Parameters.PolePairs * 6U);
    }
    p_motor->Encoder.Params.IsQuadratureCaptureEnabled = false;
}

/* Common, Set after PolePairs */
void Motor_ResetUnitsEncoder(MotorPtr_T p_motor)
{
    if(p_motor->Parameters.SpeedFeedbackRef_Rpm != p_motor->Encoder.Params.ScalarSpeedRef_Rpm)
    {
        Encoder_SetScalarSpeedRef(&p_motor->Encoder, p_motor->Parameters.SpeedFeedbackRef_Rpm);
    }
    if(p_motor->Parameters.PolePairs != p_motor->Encoder.Params.InterpolateAngleScalar) /* Set for electrical cycle */
    {
        Encoder_DeltaT_SetInterpolateAngleScalar(&p_motor->Encoder, p_motor->Parameters.PolePairs);
    }
    // if(p_motor->Parameters.GearRatio_Factor != p_motor->Encoder.Params.GearRatio_Factor) ||
    // {
    //     Encoder_SetSurfaceRatio(&p_motor->Encoder, p_motor->Parameters.GearRatio);
    // }
}

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_ResetUnitsSinCos(MotorPtr_T p_motor)
{
    if(p_motor->Parameters.PolePairs != p_motor->SinCos.Params.ElectricalRotationsRatio)
    {
        SinCos_SetAngleRatio(&p_motor->SinCos, p_motor->Parameters.PolePairs);
    }
}
#endif


/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_Jog12Step(MotorPtr_T p_motor, uint8_t step)
{
    const uint16_t duty = p_motor->Parameters.AlignPower_Scalar16;
    uint16_t index = step % 12U;
    switch(index)
    {
        case 0U: Phase_Polar_ActivateA(&p_motor->Phase, duty); break;
        case 1U: Phase_Polar_ActivateAC(&p_motor->Phase, duty); break;
        case 2U: Phase_Polar_ActivateInvC(&p_motor->Phase, duty); break;
        case 3U: Phase_Polar_ActivateBC(&p_motor->Phase, duty); break;
        case 4U: Phase_Polar_ActivateB(&p_motor->Phase, duty); break;
        case 5U: Phase_Polar_ActivateBA(&p_motor->Phase, duty); break;
        case 6U: Phase_Polar_ActivateInvA(&p_motor->Phase, duty); break;
        case 7U: Phase_Polar_ActivateCA(&p_motor->Phase, duty); break;
        case 8U: Phase_Polar_ActivateC(&p_motor->Phase, duty); break;
        case 9U: Phase_Polar_ActivateCB(&p_motor->Phase, duty); break;
        case 10U: Phase_Polar_ActivateInvB(&p_motor->Phase, duty); break;
        case 11U: Phase_Polar_ActivateAB(&p_motor->Phase, duty); break;
        default: break;
    }
}

void Motor_Jog6PhaseStep(MotorPtr_T p_motor, uint8_t step)
{
    const uint16_t duty = p_motor->Parameters.AlignPower_Scalar16;
    uint16_t index = step % 6U;
    switch(index)
    {
        case 0U: Phase_Polar_ActivateAC(&p_motor->Phase, duty); break;
        case 1U: Phase_Polar_ActivateBC(&p_motor->Phase, duty); break;
        case 2U: Phase_Polar_ActivateBA(&p_motor->Phase, duty); break;
        case 3U: Phase_Polar_ActivateCA(&p_motor->Phase, duty); break;
        case 4U: Phase_Polar_ActivateCB(&p_motor->Phase, duty); break;
        case 5U: Phase_Polar_ActivateAB(&p_motor->Phase, duty); break;
        default: break;
    }
}

/*
    Preferred for stability
*/
void Motor_Jog6Step(MotorPtr_T p_motor, uint8_t step)
{
    const uint16_t duty = p_motor->Parameters.AlignPower_Scalar16;
    uint16_t index = step % 6U;
    switch(index)
    {
        case 0U: Phase_Polar_ActivateA(&p_motor->Phase, duty); break;
        case 1U: Phase_Polar_ActivateInvC(&p_motor->Phase, duty); break;
        case 2U: Phase_Polar_ActivateB(&p_motor->Phase, duty); break;
        case 3U: Phase_Polar_ActivateInvA(&p_motor->Phase, duty); break;
        case 4U: Phase_Polar_ActivateC(&p_motor->Phase, duty); break;
        case 5U: Phase_Polar_ActivateInvB(&p_motor->Phase, duty); break;
        default: break;
    }
}

void Motor_Jog6(MotorPtr_T p_motor)
{
    Motor_Jog6Step(p_motor, p_motor->JogIndex);
    p_motor->JogIndex++;
}

void Motor_Jog6Phase(MotorPtr_T p_motor)
{
    Motor_Jog6PhaseStep(p_motor, p_motor->JogIndex);
    p_motor->JogIndex++;
}

void Motor_Jog12(MotorPtr_T p_motor)
{
    Motor_Jog12Step(p_motor, p_motor->JogIndex);
    p_motor->JogIndex++;
}