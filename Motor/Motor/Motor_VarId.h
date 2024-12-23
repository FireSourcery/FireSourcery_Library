
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
    @file   Motor_Include.h
    @author FireSourcery
    @version V0

    @brief  Var Id Access - Consistent access to Motor variables, using id/value
*/
/******************************************************************************/
#include "Motor_User.h"
#include "Motor_Config.h"

#include <assert.h>


/******************************************************************************/
/*
    RealTime
*/
/******************************************************************************/
typedef enum Motor_User_Id
{
    MOTOR_VAR_SPEED,
    MOTOR_VAR_I_PHASE,
    MOTOR_VAR_V_PHASE,
    MOTOR_VAR_POWER,
    MOTOR_VAR_RAMP_SET_POINT,
    MOTOR_VAR_ELECTRICAL_ANGLE,
    MOTOR_VAR_MECHANICAL_ANGLE,
    MOTOR_VAR_MOTOR_STATE,
    MOTOR_VAR_MOTOR_STATUS_FLAGS,
    MOTOR_VAR_MOTOR_FAULT_FLAGS,
    MOTOR_VAR_MOTOR_HEAT,
    MOTOR_VAR_MOTOR_ACTIVE_SPEED_LIMIT,
    MOTOR_VAR_MOTOR_ACTIVE_I_LIMIT,
    MOTOR_VAR_MOTOR_V_SPEED,
    MOTOR_VAR_MOTOR_END,
}
Motor_VarId_GetPrimary_T;

typedef enum Motor_User_Id
{
    MOTOR_USER_SPEED,
    MOTOR_USER_I_PHASE,
    MOTOR_USER_V_PHASE,
    MOTOR_USER_POWER,
    MOTOR_USER_RAMP_SET_POINT,
    MOTOR_USER_ELECTRICAL_ANGLE,
    MOTOR_USER_MECHANICAL_ANGLE,
    MOTOR_USER_STATE,
    MOTOR_USER_STATUS_FLAGS,
    MOTOR_USER_FAULT_FLAGS,
    MOTOR_USER_HEAT,
    MOTOR_USER_ACTIVE_SPEED_LIMIT,
    MOTOR_USER_ACTIVE_I_LIMIT,
    MOTOR_USER_V_SPEED,
    MOTOR_USER_END,
}
Motor_User_Get_T;
// Motor_VarId_Set_T;
// Motor_VarId_IO_T;

// static_assert((MOTOR_VAR_MOTOR_END <= 16U));

static inline int32_t Motor_GetVar_Primary(const Motor_T * p_motor, Motor_VarId_GetPrimary_T varId)
{
    int32_t value;
    switch (varId)
    {
        case MOTOR_VAR_SPEED:                     value = Motor_User_GetSpeed_UFract16(p_motor);                break;
        case MOTOR_VAR_I_PHASE:                   value = Motor_User_GetIPhase_UFract16(p_motor);               break;
        case MOTOR_VAR_V_PHASE:                   value = Motor_User_GetVPhase_UFract16(p_motor);               break;
        case MOTOR_VAR_POWER:                     value = Motor_User_GetElectricalPower_UFract16(p_motor);      break;
        case MOTOR_VAR_RAMP_SET_POINT:            value = Linear_Ramp_GetOutput(&p_motor->Ramp);                break;
        case MOTOR_VAR_MOTOR_STATE:               value = Motor_User_GetStateId(p_motor);                       break;
        case MOTOR_VAR_MOTOR_STATUS_FLAGS:        value = Motor_User_GetStateFlags(p_motor).Word;               break;
        case MOTOR_VAR_MOTOR_FAULT_FLAGS:         value = Motor_User_GetFaultFlags(p_motor).Value;              break;
        case MOTOR_VAR_MOTOR_HEAT:                value = Motor_User_GetHeat_Adcu(p_motor);                     break;
        case MOTOR_VAR_MOTOR_ACTIVE_SPEED_LIMIT:  value = Motor_User_GetSpeedLimit(p_motor);              break;
        case MOTOR_VAR_MOTOR_ACTIVE_I_LIMIT:      value = Motor_User_GetILimit(p_motor);                  break;
        case MOTOR_VAR_MOTOR_V_SPEED:             value = Motor_GetVSpeed_Fract16(p_motor);                     break;
        default: value = 0; break;
    }
    return value;
}

static inline int32_t Motor_GetVar_Secondary(const Motor_T * p_motor, Motor_VarId_GetPrimary_T varId)
{

}

// case MOT_VAR_ID_TYPE_CMD_MOTOR:
//     //todo wrap motor cmd with MotorController StateMachine check
//     //    if (MotorController_User_IsActiveState(p_mc))
//     //    {

//     //    }
//     switch ((MotVarId_Cmd_Motor_T)varId.NameBase)
//     {
//         // case MOT_VAR_MOTOR_USER_CMD:        Motor_User_SetActiveCmdValue(p_motor, varValue);    break;
//         case MOT_VAR_MOTOR_CMD_SPEED:       Motor_User_SetSpeedCmd(p_motor, varValue);     break;
//         case MOT_VAR_MOTOR_CMD_CURRENT:     Motor_User_SetTorqueCmd_Scalar(p_motor, varValue);    break;
//         case MOT_VAR_MOTOR_CMD_VOLTAGE:     Motor_User_SetVoltageCmd(p_motor, varValue);   break;
//         case MOT_VAR_MOTOR_CMD_ANGLE:       Motor_User_SetPositionCmd(p_motor, varValue);  break;
//         case MOT_VAR_MOTOR_CMD_OPEN_LOOP:   Motor_User_SetOpenLoopCmd(p_motor, varValue);  break;
//         case MOT_VAR_MOTOR_FORCE_DISABLE_CONTROL:           break;
//         case MOT_VAR_MOTOR_TRY_RELEASE:                     break;
//         case MOT_VAR_MOTOR_TRY_HOLD:                        break;
//         case MOT_VAR_MOTOR_CLEAR_FAULT:                     break;
//         default: break;
//     }
//     break;

// case MOT_VAR_ID_TYPE_CONTROL_MOTOR:
//     switch ((MotVarId_Control_Motor_T)varId.NameBase)
//     {
//         case MOT_VAR_MOTOR_DIRECTION:               isSuccess = Motor_User_TryDirection(p_motor, (Motor_Direction_T)varValue);  break;
//         case MOT_VAR_MOTOR_USER_SET_POINT:          Motor_User_SetActiveCmdValue(p_motor, varValue);            break;
//         case MOT_VAR_MOTOR_USER_SPEED_LIMIT:          isSuccess = Motor_User_TrySpeedLimit(p_motor, varValue);    break;
//         case MOT_VAR_MOTOR_USER_I_LIMIT:              isSuccess = Motor_User_TryILimit(p_motor, varValue);        break;
//             // case MOT_VAR_MOTOR_USER_FEEDBACK_MODE:   Motor_User_StartControl_Cast(p_motor, (uint8_t)varValue);    break;
//         default: break;
//     }
//     break;



/*
    Speed/IPhase/VPhase/Power
        Units0 => UFract16, may over saturate
*/
typedef enum MotVarId_Monitor_Motor
{
    MOT_VAR_SPEED,
    MOT_VAR_I_PHASE,
    MOT_VAR_V_PHASE,
    MOT_VAR_ELECTRICAL_ANGLE,
    MOT_VAR_MECHANICAL_ANGLE,
    MOT_VAR_POWER,
    MOT_VAR_MOTOR_HEAT,
    MOT_VAR_MOTOR_STATE,
    MOT_VAR_MOTOR_STATE_FLAGS,
    MOT_VAR_MOTOR_FAULT_FLAGS,
    MOT_VAR_MOTOR_EFFECTIVE_FEEDBACK_MODE,
    MOT_VAR_MOTOR_EFFECTIVE_SPEED_LIMIT,
    MOT_VAR_MOTOR_EFFECTIVE_I_LIMIT,
    MOT_VAR_MOTOR_V_SPEED_DEBUG,
    MOT_VAR_MOTOR_V_SPEED_EFFECTIVE,
}
MotVarId_Monitor_Motor_T;

typedef enum MotVarId_Monitor_MotorFoc
{
    MOT_VAR_FOC_IA,
    MOT_VAR_FOC_IB,
    MOT_VAR_FOC_IC,
    MOT_VAR_FOC_IQ,
    MOT_VAR_FOC_ID,
    MOT_VAR_FOC_VQ,
    MOT_VAR_FOC_VD,
    MOT_VAR_FOC_REQ_Q,
    MOT_VAR_FOC_REQ_D,
    MOT_VAR_FOC_VA,
    MOT_VAR_FOC_VB,
    MOT_VAR_FOC_VC,
}
MotVarId_Monitor_MotorFoc_T;

typedef enum MotVarId_Monitor_MotorSensor
{
    MOT_VAR_ENCODER_FREQ,
}
MotVarId_Monitor_MotorSensor_T;

//
// ControlTimerBase;
// Motor_OpenLoopState_T OpenLoopState;
// Motor_CalibrationState_T CalibrationState;

/*
    Value [-32768:32767]
*/
typedef enum MotVarId_Cmd_Motor
{
    // MOT_VAR_MOTOR_USER_CMD,      // RampIn Always reflects the input value
    MOT_VAR_MOTOR_CMD_SPEED,        // UserCmd as Speed
    MOT_VAR_MOTOR_CMD_CURRENT,
    MOT_VAR_MOTOR_CMD_VOLTAGE,
    MOT_VAR_MOTOR_CMD_ANGLE,
    MOT_VAR_MOTOR_CMD_OPEN_LOOP,
    MOT_VAR_MOTOR_FORCE_DISABLE_CONTROL,    // No value arg. Force Disable control Non StateMachine checked, also handled via Call
    MOT_VAR_MOTOR_TRY_RELEASE,              // No value arg. same as either neutral or driveZero
    MOT_VAR_MOTOR_TRY_HOLD,                 // No value arg. bypass FOC, MOT_VAR_USER_CMD = 0, VoltageMode
    MOT_VAR_MOTOR_CLEAR_FAULT,              // No value arg. Clear Faults
}
MotVarId_Cmd_Motor_T;

typedef enum MotVarId_Control_Motor
{
    MOT_VAR_MOTOR_DIRECTION,            // Motor_Direction_T - CW/CCW. Read state value, write interface value,
    MOT_VAR_MOTOR_USER_SET_POINT,       // RampIn(UserCmd)/RampOut(SetPoint), Generic mode select
    /* IO Vars, Read effective value, write interface value */
    MOT_VAR_MOTOR_USER_FEEDBACK_MODE,
    MOT_VAR_MOTOR_USER_SPEED_LIMIT,
    MOT_VAR_MOTOR_USER_I_LIMIT,
}
MotVarId_Control_Motor_T;


/******************************************************************************/
/*
    Config
    Alternatively use a schema def
*/
/******************************************************************************/
typedef enum Motor_Config_Member
{
    MOTOR_CONFIG_IA_ZERO_REF_ADCU,
    MOTOR_CONFIG_IB_ZERO_REF_ADCU,
    MOTOR_CONFIG_IC_ZERO_REF_ADCU,
    MOTOR_CONFIG_I_PEAK_REF_ADCU,
    MOTOR_CONFIG_SPEED_LIMIT_FORWARD_PERCENT16,
    MOTOR_CONFIG_SPEED_LIMIT_REVERSE_PERCENT16,
    MOTOR_CONFIG_I_LIMIT_MOTORING_PERCENT16,
    MOTOR_CONFIG_I_LIMIT_GENERATING_PERCENT16,
    MOTOR_CONFIG_RAMP_ACCEL_CYCLES,
    MOTOR_CONFIG_ALIGN_POWER_PERCENT16,
    MOTOR_CONFIG_ALIGN_TIME_CYCLES,
    MOTOR_CONFIG_OPEN_LOOP_SPEED_PERCENT16,
    MOTOR_CONFIG_OPEN_LOOP_POWER_PERCENT16,
    MOTOR_CONFIG_OPEN_LOOP_ACCEL_CYCLES,
    MOTOR_CONFIG_SURFACE_DIAMETER,
    MOTOR_CONFIG_GEAR_RATIO_FACTOR,
    MOTOR_CONFIG_GEAR_RATIO_DIVISOR,
    MOTOR_CONFIG_MEMBER_COUNT // This can be used to get the number of members
}
Motor_Config_Member_T;

/*
    Instance0 -> Motor[0]
    Enum values for =>
        Motor_CommutationMode_T,
        Motor_SensorMode_T,
        Motor_FeedbackModeId_T,
        Motor_DirectionCalibration_T
*/
typedef enum MotVarId_Config_MotorPrimary
{
    MOT_VAR_COMMUTATION_MODE, // if runtime supported
    MOT_VAR_SENSOR_MODE,
    MOT_VAR_DIRECTION_CALIBRATION,
    MOT_VAR_POLE_PAIRS,
    MOT_VAR_KV,
    MOT_VAR_V_SPEED_SCALAR,
    MOT_VAR_SPEED_V_REF_RPM,
    MOT_VAR_SPEED_V_MATCH_REF_RPM,
    MOT_VAR_IA_ZERO_REF_ADCU,
    MOT_VAR_IB_ZERO_REF_ADCU,
    MOT_VAR_IC_ZERO_REF_ADCU,
    MOT_VAR_I_PEAK_REF_ADCU,
}
MotVarId_Config_MotorPrimary_T;

static inline int32_t Motor_GetConfigMember(const Motor_T * p_motor, Motor_VarId_GetPrimary_T varId)
{

}

/*
    Limits
        Units0 - > Percent16
*/
typedef enum MotVarId_Config_MotorSecondary
{
    MOT_VAR_BASE_SPEED_LIMIT_FORWARD,
    MOT_VAR_BASE_SPEED_LIMIT_REVERSE,
    MOT_VAR_BASE_I_LIMIT_MOTORING,
    MOT_VAR_BASE_I_LIMIT_GENERATING,
    MOT_VAR_RAMP_ACCEL_TIME,
    MOT_VAR_ALIGN_MODE,
    MOT_VAR_ALIGN_POWER,
    MOT_VAR_ALIGN_TIME,
    MOT_VAR_OPEN_LOOP_POWER,
    MOT_VAR_OPEN_LOOP_SPEED,
    MOT_VAR_OPEN_LOOP_ACCEL_TIME,
    MOT_VAR_PHASE_PWM_MODE,
}
MotVarId_Config_MotorSecondary_T;