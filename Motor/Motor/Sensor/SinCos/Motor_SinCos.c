/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor_SinCos.c
    @author FireSourcery
    @brief  Motor-level SinCos calibration state machine.

    Calibration sequence (each step holds 1x AlignTime):
        0: Energize Phase A — rotor settles at electrical 0
        1: Sample sin/cos -> AngleOffset
        2: Energize Phase B — rotor settles at electrical +120°
        3: Sample sin/cos -> IsCcwPositive (sign of decoded angle)
        4: Deactivate phases, finish
*/
/******************************************************************************/
#include "Motor_SinCos.h"
#include "SinCos_Sensor.h"

#include "../../Motor_StateMachine.h"


/******************************************************************************/
/*!
    Accessors
*/
/******************************************************************************/
static inline const SinCos_RotorSensor_T * GetSinCosSensor(const Motor_T * p_motor)
{
    return &p_motor->SENSOR_TABLE.SIN_COS;
}


/******************************************************************************/
/*!
    Calibration State
*/
/******************************************************************************/
static void Calibration_Entry(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Phase_ActivateV0(&p_motor->PHASE);
    p_motor->P_MOTOR->CalibrationStateIndex = 0U;
}

static State_T * Calibration_End(const Motor_T * p_motor);

static void Calibration_Proc(const Motor_T * p_motor)
{
    const SinCos_RotorSensor_T * p_sensor = GetSinCosSensor(p_motor);
    SinCos_State_T * p_sinCosState = p_sensor->SIN_COS.P_STATE;
    const uint16_t duty = Motor_GetVAlign_Duty(&p_motor->P_MOTOR->Config);

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        switch (p_motor->P_MOTOR->CalibrationStateIndex)
        {
            case 0U:
                Phase_Align(&p_motor->PHASE, PHASE_ID_A, duty);
                p_motor->P_MOTOR->CalibrationStateIndex = 1U;
                break;

            case 1U:
                SinCos_CalibrateAngleOffset(p_sinCosState, SinCos_Analog_GetSin(&p_sensor->ANALOG), SinCos_Analog_GetCos(&p_sensor->ANALOG));
                Phase_Align(&p_motor->PHASE, PHASE_ID_INV_C, duty);  /* electrical +120° */
                p_motor->P_MOTOR->CalibrationStateIndex = 2U;
                break;

            case 2U:
                SinCos_CalibrateCcwPositive(p_sinCosState, SinCos_Analog_GetSin(&p_sensor->ANALOG), SinCos_Analog_GetCos(&p_sensor->ANALOG));
                Phase_Deactivate(&p_motor->PHASE);
                p_motor->P_MOTOR->CalibrationStateIndex = 3U;
                break;

            default: break;
        }
    }
}

static State_T * Calibration_End(const Motor_T * p_motor)
{
    if (p_motor->P_MOTOR->CalibrationStateIndex >= 3U)
    {
        p_motor->P_MOTOR->FaultFlags.PositionSensor = !RotorSensor_VerifyCalibration(p_motor->P_MOTOR->p_ActiveSensor);
        return &MOTOR_STATE_CALIBRATION;
    }
    return NULL;
}

static const State_T CALIBRATION_STATE_SIN_COS =
{
    .ID         = 0U,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)Calibration_Entry,
    .LOOP       = (State_Action_T)Calibration_Proc,
    .NEXT       = (State_Input0_T)Calibration_End,
};


/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
static State_T * Calibration_Start(const Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE_SIN_COS; }

void Motor_SinCos_Calibrate(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Calibration_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

void Motor_SinCos_Cmd(const Motor_T * p_motor, int varId, int varValue)
{
    (void)varValue;
    if (!RotorSensor_Validate(&p_motor->SENSOR_TABLE, p_motor->P_MOTOR->p_ActiveSensor, ROTOR_SENSOR_ID_SIN_COS)) return;
    if (p_motor->P_MOTOR->Config.SensorMode != ROTOR_SENSOR_ID_SIN_COS) return;

    switch (varId)
    {
        case 0:   Motor_SinCos_Calibrate(p_motor); break;
        default:  Motor_SinCos_Calibrate(p_motor); break;
    }
}
