
/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   MotorController_Traction.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "MotorController_Traction.h"
#include "../MotorController_StateMachine.h"
#include "../MotorController_User.h"


/******************************************************************************/
/*!
    @brief [Traction] HSM SubState

    [Throttle] + [Brake] + [Neutral State]
    override MotorController StateMachine Inputs
*/
/******************************************************************************/
static const State_T STATE_DRIVE;
static const State_T STATE_NEUTRAL; /* setting MOTOR_DIRECTION_NULL remaps to Neutral State VZ/V0 */

static inline Traction_T * TractionAdapter(MotorController_T * p_mc) { return ((Traction_T *)(p_mc->P_APP_STATE)); }

/* From Park */
static State_T * EnterMain(MotorController_T * p_mc, state_value_t fromPark)
{
    (void)fromPark;
    if (!Motor_Table_IsEvery(&p_mc->MOTORS, Motor_IsSpeedZero)) { return &STATE_NEUTRAL; }

    switch (TractionAdapter(p_mc)->Input.Direction)
    {
        case MOTOR_DIRECTION_NULL:       return &STATE_NEUTRAL;
        case MOTOR_DIRECTION_FORWARD:    return &STATE_DRIVE;
        case MOTOR_DIRECTION_REVERSE:    return &STATE_DRIVE;
        default:                         return NULL; /* Stays in Park */
    }
}

static void Init(MotorController_T * p_mc)
{
    Traction_InitFrom(TractionAdapter(p_mc), (const Traction_Config_T *)p_mc->P_APP_NVM_CONFIG);
}

MotorController_App_T MC_APP_TRACTION =
{
    .PROC_ANALOG_USER = MotorController_Traction_ProcAnalogUser,
    .ENTER_MAIN = (State_Input_T)EnterMain,
    .INIT = Init,
};

/******************************************************************************/
/*
    Called from StateMachine
    State Mode use full context
*/
/******************************************************************************/
/* alternatively convert input only */
void Traction_StartThrottleMode(const Traction_T * p_vehicle, Motor_Table_T * p_motors)
{
    switch (p_vehicle->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_Table_ApplyFeedbackMode(p_motors, MOTOR_FEEDBACK_MODE_SPEED_CURRENT);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_Table_ApplyFeedbackMode(p_motors, MOTOR_FEEDBACK_MODE_CURRENT);           break;
        default: break;
    }

    /* alternatively from Release only */
    Motor_Table_ApplyControl(p_motors, PHASE_VOUT_PWM);
}

/* if handle each value update with statemachine on input */
void Traction_ApplyThrottleValue(const Traction_T * p_vehicle, Motor_Table_T * p_motors, uint16_t userCmdThrottle)
{
    int16_t cmdValue = (int32_t)userCmdThrottle / 2;

    switch (p_vehicle->Config.ThrottleMode)
    {
        case TRACTION_THROTTLE_MODE_SPEED:  Motor_Table_SetCmdWith(p_motors, Motor_SetSpeedMotoringCmdScalar, (int32_t)cmdValue);     break;
        case TRACTION_THROTTLE_MODE_TORQUE: Motor_Table_SetCmdWith(p_motors, Motor_SetICmdScalar, (int32_t)cmdValue);         break;
        default: break;
    }
}

// apply hold on low speed
void Traction_StartBrakeMode(const Traction_T * p_vehicle, Motor_Table_T * p_motors)
{
    switch (p_vehicle->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE:  Motor_Table_ApplyFeedbackMode(p_motors, MOTOR_FEEDBACK_MODE_CURRENT);  break;
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_Table_ApplyFeedbackMode(p_motors, MOTOR_FEEDBACK_MODE_VOLTAGE);  break;
        default: break;
    }
    Motor_Table_ApplyControl(p_motors, PHASE_VOUT_PWM);     /* alternatively from Release only */
}

/*!
    Always request opposite direction current
    req opposite iq, bound vq to 0 for no plugging brake

    transition from accelerating to decelerating,
    use signed ramp to transition through 0 without discontinuity
    ramp from in-direction torque to 0 to counter-direction torque

    @param[in] brake [0:32767]
*/
void Traction_ApplyBrakeValue(const Traction_T * p_vehicle, Motor_Table_T * p_motors, uint16_t userCmdBrake)
{
    int16_t cmdValue = 0 - ((int32_t)userCmdBrake / 2); // 32767 max

    switch (p_vehicle->Config.BrakeMode)
    {
        case TRACTION_BRAKE_MODE_TORQUE: Motor_Table_SetCmdWith(p_motors, Motor_SetICmdScalar, cmdValue); break; /* note: torqueRamp also written in voltage mode */
            // case TRACTION_BRAKE_MODE_VOLTAGE: Motor_Table_SetCmdWith(p_motors, Motor_SetRegenCmd, 0); break;
        default: break;
    }
}


/* an alternate cmd for float is required */
void Traction_StartDriveZero(const Traction_T * p_vehicle, Motor_Table_T * p_motors)
{
    switch (p_vehicle->Config.ZeroMode)
    {
        case TRACTION_ZERO_MODE_FLOAT:   Motor_Table_ApplyControl(p_motors, PHASE_VOUT_Z);         break;
        case TRACTION_ZERO_MODE_IZERO:   Motor_Table_ForEachApply(p_motors, Motor_ApplyTorque0);   break;
        case TRACTION_ZERO_MODE_REGEN:       /* Traction_SetRegenMotorAll(p_this); */ break;
        default: break;
    }
}

// optionally select speed ramp down
void Traction_ProcRelease(const Traction_T * p_vehicle, Motor_Table_T * p_motors)
{
    (void)p_vehicle; (void)p_motors;
    // Motor_Table_ForEachApply(p_motors, Motor_Hypervisor);
}

/******************************************************************************/
/*!
    @brief Drive State

    Motor States: Run, Freewheel
    Accepted Inputs: Throttle, Brake
    Associate a drive release mode to cmd value 0, on top of motor layer
    SubStates: Fwd/Rev,

    neutral and drive share stop via motor stop state.
    DriveZero must release control, to transition into Park State
*/
/******************************************************************************/
static void Drive_Entry(const MotorController_T * p_mc)
{
    assert(TractionAdapter(p_mc)->Input.Direction != MOTOR_DIRECTION_NULL); /* Direction should have been checked on transition.*/
    TractionAdapter(p_mc)->Input.DriveCmd = TRACTION_CMD_RELEASE; // next input is edge transition
    Motor_Table_ApplyUserDirection(&p_mc->MOTORS, TractionAdapter(p_mc)->Input.Direction); /* Buffered direction cmd from user input, independent of motor state */
}

static void Drive_Proc(const MotorController_T * p_mc)
{

}

static State_T * Drive_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    switch (direction)
    {
        case MOTOR_DIRECTION_NULL:       return &STATE_NEUTRAL;
        case MOTOR_DIRECTION_FORWARD:    return NULL;
        case MOTOR_DIRECTION_REVERSE:    return NULL;
        default:                         return NULL;
    }
}

static State_T * Drive_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    switch (mode)
    {
        case TRACTION_CMD_BRAKE:     Traction_StartBrakeMode(TractionAdapter(p_mc), &p_mc->MOTORS);      break;
        case TRACTION_CMD_THROTTLE:  Traction_StartThrottleMode(TractionAdapter(p_mc), &p_mc->MOTORS);   break;
        case TRACTION_CMD_RELEASE:   Traction_StartDriveZero(TractionAdapter(p_mc), &p_mc->MOTORS);      break;
        default: break;
    }
    return NULL;
}

static State_T * Drive_InputThrottleValue(const MotorController_T * p_mc, state_value_t value)
{
    Traction_ApplyThrottleValue(TractionAdapter(p_mc), &p_mc->MOTORS, value);

    // Motor_Table_SetCmdWith(&p_mc->MOTORS, TractionAdapter(p_mc)->p_ThrottleDrive->APPLY_CMD, value / 2);
    return NULL;
}

static State_T * Drive_InputBrakeValue(const MotorController_T * p_mc, state_value_t value)
{
    Traction_ApplyBrakeValue(TractionAdapter(p_mc), &p_mc->MOTORS, value);
    return NULL;
}

// State_Input_T Drive_TransitionMapper(state_input_t inputId)
// {
//     State_Input_T inputHandler = NULL;
//     switch (inputId)
//     {
//         case TRACTION_STATE_INPUT_DIRECTION: inputHandler = (State_Input_T)Drive_InputDirection; break;
//         case TRACTION_STATE_INPUT_DRIVE_CMD: inputHandler = (State_Input_T)Drive_InputCmdStart; break;
//         default: break;
//     }
//     return inputHandler;
// }

/* App-specific commands — reads from Traction.Input (buffered by MotorController_Traction_ApplyDirection etc.) */
static State_T * Drive_InputAppUser(const MotorController_T * p_mc, state_value_t appCmd)
{
    Traction_Input_T * p_input = &TractionAdapter(p_mc)->Input;
    switch (appCmd)
    {
        case TRACTION_STATE_INPUT_DIRECTION: return Drive_InputDirection(p_mc, p_input->Direction);
        case TRACTION_STATE_INPUT_DRIVE_CMD: return Drive_InputCmdStart(p_mc, p_input->DriveCmd);
        case TRACTION_STATE_INPUT_THROTTLE_VALUE: return Drive_InputThrottleValue(p_mc, p_input->ThrottleValue);
        case TRACTION_STATE_INPUT_BRAKE_VALUE: return Drive_InputBrakeValue(p_mc, p_input->BrakeValue);
        default: return NULL;
    }

    // Traction_State_T * s = &TractionAdapter(p_mc)[0];
    // Motor_Table_ApplyInput(&p_mc->MOTORS, Traction_ToMotorInput(&s->Input, &s->Config));
    // return NULL;
}

static const State_Input_T DRIVE_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_APP_USER] = (State_Input_T)Drive_InputAppUser,
    [MC_STATE_INPUT_STATE_CMD] = NULL, /* propagate up */
};

static const State_T STATE_DRIVE =
{
    .ID         = TRACTION_STATE_ID_DRIVE,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Drive_Entry,
    .LOOP       = (State_Action_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
    // .TRANSITION_MAPPER = (State_InputMapper_T)Drive_TransitionMapper,
};


/******************************************************************************/
/*!
    @brief  Neutral State
    Drive extension
    Motor States: Run (Braking only), Freewheel
        Motor maybe in Run when Braking
        TRACTION remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(const MotorController_T * p_mc)
{
    Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z);
    // if (p_mc->p_mc_STATE->Input.Cmd != TRACTION_CMD_BRAKE) { Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z); }  /* If enter neutral while braking, handle discontinuity */
}

static void Neutral_Proc(const MotorController_T * p_mc)
{

}

/*
    Motor Keeps Direction State
*/
static State_T * InputDriveDirection(const MotorController_T * p_mc, state_value_t direction)
{
    // Motor_Table_ApplyUserDirection(&p_mc->MOTORS, direction); /* optionally set, async mode returns immediately.  */
    if (Motor_Table_IsEveryUserDirection(&p_mc->MOTORS, direction) == true) { return &STATE_DRIVE; }
    if (Motor_Table_IsEvery(&p_mc->MOTORS, Motor_IsSpeedZero) == true) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, direction); return &STATE_DRIVE; }
    return NULL;
}

static State_T * Neutral_InputDirection(const MotorController_T * p_mc, state_value_t direction)
{
    switch ((sign_t)direction)
    {
        case MOTOR_DIRECTION_FORWARD:   return InputDriveDirection(p_mc, direction);
        case MOTOR_DIRECTION_REVERSE:   return InputDriveDirection(p_mc, direction);
        case MOTOR_DIRECTION_NULL:      return &STATE_NEUTRAL;
        default:                        return NULL;
    }
}

static State_T * Neutral_InputCmdStart(const MotorController_T * p_mc, state_value_t mode)
{
    switch ((Traction_Cmd_T)mode)
    {
        case TRACTION_CMD_RELEASE:   Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z);                            break;
        case TRACTION_CMD_BRAKE:     Traction_StartBrakeMode(TractionAdapter(p_mc), &p_mc->MOTORS);                    break;
        case TRACTION_CMD_THROTTLE:  break;
        default: break;
    }
    return NULL;
}

static State_T * Neutral_InputBrakeValue(const MotorController_T * p_mc, state_value_t value)
{
    Traction_ApplyBrakeValue(TractionAdapter(p_mc), &p_mc->MOTORS, value);
    return NULL;
}

/* App-specific commands — reads from Traction.Input (buffered by MotorController_Traction_ApplyDirection etc.) */
static State_T * Neutral_InputAppUser(const MotorController_T * p_mc, state_value_t appCmd)
{
    Traction_Input_T * p_input = &TractionAdapter(p_mc)->Input;
    switch (appCmd)
    {
        case TRACTION_STATE_INPUT_DIRECTION: return Neutral_InputDirection(p_mc, p_input->Direction);
        case TRACTION_STATE_INPUT_DRIVE_CMD: return Neutral_InputCmdStart(p_mc, p_input->DriveCmd);
        case TRACTION_STATE_INPUT_BRAKE_VALUE: return Neutral_InputBrakeValue(p_mc, p_input->BrakeValue);
        case TRACTION_STATE_INPUT_THROTTLE_VALUE: return NULL; /* Throttle has no effect in Neutral */
        default: return NULL;
    }
}

static const State_Input_T NEUTRAL_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_APP_USER] = (State_Input_T)Neutral_InputAppUser,
};

static const State_T STATE_NEUTRAL =
{
    .ID         = TRACTION_STATE_ID_NEUTRAL,
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)Neutral_Entry,
    .LOOP       = (State_Action_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* also returns NEUTRAL on motors mismatch  */
/* Motors may keep direction state in Neutral */
sign_t MotorController_Traction_GetDirection(const MotorController_T * p_mc)
{
    return (StateMachine_GetLeafState(p_mc->STATE_MACHINE.P_ACTIVE) == &STATE_NEUTRAL) ? (sign_t)MOTOR_DIRECTION_NULL : MotorController_GetDirection(p_mc);
}

Traction_StateId_T MotorController_Traction_GetStateId(MotorController_T * p_dev)
{
    if (StateMachine_GetLeafState(p_dev->STATE_MACHINE.P_ACTIVE) == &STATE_NEUTRAL) { return TRACTION_STATE_ID_NEUTRAL; }
    if (StateMachine_GetLeafState(p_dev->STATE_MACHINE.P_ACTIVE) == &STATE_DRIVE) { return TRACTION_STATE_ID_DRIVE; }
    return (Traction_StateId_T)STATE_ID_NULL;
}


/******************************************************************************/
/*!
    @brief StateMachine Input

    Input ~10-50ms
    Proc State/Buffer ~1ms
    Apply state changes on input

    alternatively implement as nested state machine.
        seperate set input alphabet. inputs must be buffer only to prvent proc when outer state is not active.

    edge detection prior to state machine call.
        alternatively, handle within input handler. State machine virtualizes input entirely.
        MotorController sm layer as input mapper. cmd value can always call input handler, unlike motor sm setpoint value which buffers.
*/
/******************************************************************************/

/* Alternatively MotorController   allocate reserved input ids */
void _MotorController_Traction_ApplyCmd(MotorController_T * p_mc, Traction_StateInput_T cmd)
{
    _StateMachine_Branch_CallInput(p_mc->STATE_MACHINE.P_ACTIVE, (void *)p_mc, MC_STATE_INPUT_APP_USER, cmd);
}

// void MotorController_Traction_StartThrottle(MotorController_T * p_mc) { MotorController_Traction_ApplyStartCmd(p_mc, TRACTION_CMD_THROTTLE); }
// void MotorController_Traction_StartBrake(MotorController_T * p_mc) { MotorController_Traction_ApplyStartCmd(p_mc, TRACTION_CMD_BRAKE); }
// void MotorController_Traction_StartRelease(MotorController_T * p_mc) { MotorController_Traction_ApplyStartCmd(p_mc, TRACTION_CMD_RELEASE); }



/******************************************************************************/
/*
    User layer
*/
/******************************************************************************/
/*
    capture user cmd
    edge detect
    send to state machine

    p_input->DriveCmd proc through input handler only.
*/
/*
    Unified CmdId detection, inclusive of edge detection.
    Cmd "state" == input, changes on edge, handle outside of StateMachine
    propagating value needs state constraint. even though edge detect does not
*/
void MotorController_Traction_PollStartCmd(MotorController_T * p_mc)
{
    if (Traction_Input_PollCmdEdge(&TractionAdapter(p_mc)->Input)) { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_DRIVE_CMD); }
}

void MotorController_Traction_SetThrottle(MotorController_T * p_mc, uint16_t userCmd)
{
    if (Traction_Input_PollThrottle(&TractionAdapter(p_mc)->Input, userCmd)) { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_DRIVE_CMD); }
    else { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_THROTTLE_VALUE); }
}

void MotorController_Traction_SetBrake(MotorController_T * p_mc, uint16_t userCmd)
{
    if (Traction_Input_PollBrake(&TractionAdapter(p_mc)->Input, userCmd)) { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_DRIVE_CMD); }
    else { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_BRAKE_VALUE); }
}

void MotorController_Traction_SetRelease(MotorController_T * p_mc)
{
    TractionAdapter(p_mc)->Input.ThrottleValue = 0U;
    TractionAdapter(p_mc)->Input.BrakeValue = 0U;
    MotorController_Traction_PollStartCmd(p_mc);
}

void MotorController_Traction_SetThrottleBrake(MotorController_T * p_mc, uint16_t throttle, uint16_t brake)
{
    TractionAdapter(p_mc)->Input.ThrottleValue = throttle;
    TractionAdapter(p_mc)->Input.BrakeValue = brake;
    MotorController_Traction_PollStartCmd(p_mc);
    switch (TractionAdapter(p_mc)->Input.DriveCmd)
    {
        case TRACTION_CMD_BRAKE:     _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_BRAKE_VALUE);         break;
        case TRACTION_CMD_THROTTLE:  _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_THROTTLE_VALUE);      break;
        case TRACTION_CMD_RELEASE:   break;
        default: break;
    }
}


/*
    Direction is write only
*/
// caller handled edge detection
void MotorController_Traction_ApplyDirectionCmd(MotorController_T * p_mc, sign_t direction)
{
    TractionAdapter(p_mc)->Input.Direction = direction;
    _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_DIRECTION);
    if (MotorController_Traction_GetDirection(p_mc) != direction) { Blinky_Blink(&p_mc->BUZZER, 500U); }
}

/* polls edge */
void MotorController_Traction_CaptureDirection(MotorController_T * p_mc, sign_t direction)
{
    if (Traction_Input_PollDirectionEdge(&TractionAdapter(p_mc)->Input, direction))
    {
        _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_DIRECTION);
    }
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline uint16_t AInThrottle(const MotorController_T * p_mc) { return UserAIn_GetValue(&p_mc->AINS[MOT_AIN_THROTTLE].PIN); }
static inline uint16_t AInBrake(const MotorController_T * p_mc) { return UserAIn_GetValue(&p_mc->AINS[MOT_AIN_BRAKE].PIN); }

#include "../MotAnalogUser/OptPin/MotorController_SwitchBrake.h"
void MotorController_Traction_ProcAnalogUser(const MotorController_T * p_mc)
{
    MotorController_Traction_CaptureDirection(p_mc, (sign_t)Shifter_ResolveDirection(&p_mc->SHIFTER));
    MotorController_Traction_SetThrottleBrake(p_mc, AInThrottle(p_mc), MotorController_FuseSwitchBrake(p_mc, AInBrake(p_mc)));
}



/******************************************************************************/
/*!
    Protocol inputs as known edge trigger
*/
/******************************************************************************/
/*
    throttle=0 and brake=X writes in the same packet —
    SetThrottle(0) evaluates RELEASE, Motor transitions to Passive.
    SetBrake(X) evaluates BRAKE, Motor transitions to Run.
    Passive_Proc never runs, BEMF is never captured. FOC_Vq must match to speed
*/
void MotorController_Traction_VarId_Set(MotorController_T * p_mc, Traction_VarId_T id, int value)
{
    switch (id)
    {
        case TRACTION_VAR_DIRECTION:   MotorController_Traction_ApplyDirectionCmd(p_mc, (sign_t)value);  break;
        case TRACTION_VAR_THROTTLE:    MotorController_Traction_SetThrottle(p_mc, (uint16_t)value);      break;
        case TRACTION_VAR_BRAKE:       MotorController_Traction_SetBrake(p_mc, (uint16_t)value);         break;
        case TRACTION_VAR_STATE_ID:    break; /* read only */
    }
}

int MotorController_Traction_VarId_Get(MotorController_T * p_mc, Traction_VarId_T id)
{
    int value = 0;
    switch (id)
    {
        case TRACTION_VAR_DIRECTION:   value = MotorController_Traction_GetDirection(p_mc);     break;
        case TRACTION_VAR_THROTTLE:    value = TractionAdapter(p_mc)->Input.ThrottleValue;  break;
        case TRACTION_VAR_BRAKE:       value = TractionAdapter(p_mc)->Input.BrakeValue;     break;
        case TRACTION_VAR_STATE_ID:    value = MotorController_Traction_GetStateId(p_mc); break;
        default: break;
    }
    return value;
}

int MotorController_Traction_ConfigId_Get(const MotorController_T * p_mc, Traction_ConfigId_T id)
{
    return Traction_ConfigId_Get(&TractionAdapter(p_mc)->Config, id);
}

void MotorController_Traction_ConfigId_Set(MotorController_T * p_mc, Traction_ConfigId_T id, int value)
{
    Traction_ConfigId_Set(&TractionAdapter(p_mc)->Config, id, value);
}



