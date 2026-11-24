
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

/* From Standby */
static State_T * EnterMain(MotorController_T * p_mc, state_value_t fromStandby)
{
    (void)fromStandby;
    if (!Motor_Table_IsEverySpeedZero(&p_mc->MOTORS)) { return &STATE_NEUTRAL; }

    switch (TractionAdapter(p_mc)->Input.Direction)
    {
        case MOTOR_DIRECTION_NULL:       return &STATE_NEUTRAL;
        case MOTOR_DIRECTION_FORWARD:    return &STATE_DRIVE;
        case MOTOR_DIRECTION_REVERSE:    return &STATE_DRIVE;
        default:                         return NULL; /* Stays in Standby */
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
    Traction over the motor table
    Each motor through [Motor_Traction.h]. The decisions that read every motor stay here.
*/
/******************************************************************************/
typedef void (*Motor_TractionProc_T)(Motor_T * p_motor, const Traction_T * p_vehicle);

static void ForEachMotor(MotorController_T * p_mc, Motor_TractionProc_T proc)
{
    for (uint8_t iMotor = 0U; iMotor < p_mc->MOTORS.LENGTH; iMotor++) { proc(&p_mc->MOTORS.P_DEVS[iMotor], TractionAdapter(p_mc)); }
}

/* The hold engages once no motor advances, so every motor holds together */
static void ApplyBrakeValue(MotorController_T * p_mc, uint16_t userCmdBrake)
{
    for (uint8_t iMotor = 0U; iMotor < p_mc->MOTORS.LENGTH; iMotor++) { Motor_Traction_ApplyBrakeValue(&p_mc->MOTORS.P_DEVS[iMotor], TractionAdapter(p_mc), userCmdBrake); }
    if (Motor_Table_IsAny(&p_mc->MOTORS, Motor_IsSpeedAligned) == false) { ForEachMotor(p_mc, Motor_Traction_ApplyBrakeHold); }
}

/******************************************************************************/
/*
    State
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief Drive State

    Motor States: Run, Freewheel
    Accepted Inputs: Throttle, Brake
    Associate a drive release mode to cmd value 0, on top of motor layer
    SubStates: Fwd/Rev,

    neutral and drive share stop via motor stop state.
    DriveZero must release control, to transition into Standby State
*/
/******************************************************************************/
static void Drive_Entry(MotorController_T * p_mc)
{
    assert(TractionAdapter(p_mc)->Input.Direction != SIGN_ZERO); /* Direction should have been checked on transition.*/
    /* sync DriveCmd to current input: a throttle held at entry yields no start-edge, so StartThrottleMode never fires and the motor stays passive until released then re-pressed */
    Traction_Input_PollCmd(&TractionAdapter(p_mc)->Input);
    Motor_Table_ApplyUserDirection(&p_mc->MOTORS, (Motor_Direction_T)TractionAdapter(p_mc)->Input.Direction); /* Buffered direction cmd from user input, independent of motor state */
}

static void Drive_Proc(MotorController_T * p_mc)
{

}

/*
    Reverse at standstill only, as the lever [Drive_InputLeverDirection]; the Motor refuses DIRECTION while in RUN.
    A selector with no neutral position (SHIFTER_PIN_MODE_R) never leaves DRIVE, so this is its only reversal path.
    While rolling the edge is consumed and Input.Direction holds the request; it is not re-applied when speed reaches zero.
*/
static State_T * Drive_InputDirection(MotorController_T * p_mc, state_value_t direction)
{
    switch (direction)
    {
        case MOTOR_DIRECTION_NULL:       return &STATE_NEUTRAL;
        case MOTOR_DIRECTION_FORWARD:
        case MOTOR_DIRECTION_REVERSE:
            if (Motor_Table_IsEverySpeedZero(&p_mc->MOTORS) == true) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, (Motor_Direction_T)direction); }
            return NULL;
        default:                         return NULL;
    }
}

static State_T * Drive_InputCmdStart(MotorController_T * p_mc, state_value_t mode)
{
    switch (mode)
    {
        case TRACTION_CMD_BRAKE:     ForEachMotor(p_mc, Motor_Traction_StartBrakeMode);       break;
        case TRACTION_CMD_THROTTLE:  ForEachMotor(p_mc, Motor_Traction_StartThrottleMode);    break;
        case TRACTION_CMD_RELEASE:   ForEachMotor(p_mc, Motor_Traction_StartDriveZero);       break;
        default: break;
    }
    return NULL;
}

static State_T * Drive_InputThrottleValue(MotorController_T * p_mc, state_value_t value)
{
    for (uint8_t iMotor = 0U; iMotor < p_mc->MOTORS.LENGTH; iMotor++) { Motor_Traction_ApplyThrottleValue(&p_mc->MOTORS.P_DEVS[iMotor], TractionAdapter(p_mc), value); }

    // Motor_Table_SetCmdWith(&p_mc->MOTORS, TractionAdapter(p_mc)->p_ThrottleDrive->APPLY_CMD, value / 2);
    return NULL;
}

static State_T * Drive_InputBrakeValue(MotorController_T * p_mc, state_value_t value)
{
    ApplyBrakeValue(p_mc, value);
    return NULL;
}

/* Reverse at standstill only. While rolling, the lever is read against the motion */
static State_T * Drive_InputLeverDirection(MotorController_T * p_mc, state_value_t direction)
{
    if (Motor_Table_IsEverySpeedZero(&p_mc->MOTORS) == true) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, (Motor_Direction_T)direction); }
    return NULL;
}

static State_T * Drive_InputLeverValue(MotorController_T * p_mc, state_value_t value)
{
    for (uint8_t iMotor = 0U; iMotor < p_mc->MOTORS.LENGTH; iMotor++) { Motor_Traction_ApplyLeverValue(&p_mc->MOTORS.P_DEVS[iMotor], TractionAdapter(p_mc), value); }
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
static State_T * Drive_InputAppUser(MotorController_T * p_mc, state_value_t appCmd)
{
    Traction_Input_T * p_input = &TractionAdapter(p_mc)->Input;
    switch (appCmd)
    {
        case TRACTION_STATE_INPUT_DIRECTION:        return Drive_InputDirection(p_mc, p_input->Direction);
        case TRACTION_STATE_INPUT_DRIVE_CMD:        return Drive_InputCmdStart(p_mc, p_input->DriveCmd);
        case TRACTION_STATE_INPUT_THROTTLE_VALUE:   return Drive_InputThrottleValue(p_mc, p_input->ThrottleValue);
        case TRACTION_STATE_INPUT_BRAKE_VALUE:      return Drive_InputBrakeValue(p_mc, p_input->BrakeValue);
        case TRACTION_STATE_INPUT_LEVER_DIRECTION:  return Drive_InputLeverDirection(p_mc, math_sign(p_input->LeverValue));
        case TRACTION_STATE_INPUT_LEVER_VALUE:      return Drive_InputLeverValue(p_mc, p_input->LeverValue);
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
    .PATH_ID      = { .Depth0 = MC_STATE_ID_MAIN }, /* sub-state id not yet allocated: reports as the root state */
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
static void Neutral_Entry(MotorController_T * p_mc)
{
    Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z);
    // if (p_mc->p_mc_STATE->Input.Cmd != TRACTION_CMD_BRAKE) { Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z); }  /* If enter neutral while braking, handle discontinuity */
}

static void Neutral_Proc(MotorController_T * p_mc)
{

}

/*
    Motor Keeps Direction State
*/
static State_T * InputDriveDirection(MotorController_T * p_mc, state_value_t direction)
{
    // Motor_Table_ApplyUserDirection(&p_mc->MOTORS, direction); /* optionally set, async mode returns immediately.  */
    if (Motor_Table_IsEveryUserDirection(&p_mc->MOTORS, direction) == true) { return &STATE_DRIVE; }
    if (Motor_Table_IsEverySpeedZero(&p_mc->MOTORS) == true) { Motor_Table_ApplyUserDirection(&p_mc->MOTORS, direction); return &STATE_DRIVE; }
    return NULL;
}

static State_T * Neutral_InputDirection(MotorController_T * p_mc, state_value_t direction)
{
    switch ((sign_t)direction)
    {
        case MOTOR_DIRECTION_FORWARD:   return InputDriveDirection(p_mc, direction);
        case MOTOR_DIRECTION_REVERSE:   return InputDriveDirection(p_mc, direction);
        case MOTOR_DIRECTION_NULL:      return &STATE_NEUTRAL;
        default:                        return NULL;
    }
}

static State_T * Neutral_InputCmdStart(MotorController_T * p_mc, state_value_t mode)
{
    switch ((Traction_Cmd_T)mode)
    {
        case TRACTION_CMD_RELEASE:   Motor_Table_ApplyControl(&p_mc->MOTORS, PHASE_VOUT_Z);                            break;
        case TRACTION_CMD_BRAKE:     ForEachMotor(p_mc, Motor_Traction_StartBrakeMode);                                 break;
        case TRACTION_CMD_THROTTLE:  break;
        default: break;
    }
    return NULL;
}

static State_T * Neutral_InputBrakeValue(MotorController_T * p_mc, state_value_t value)
{
    ApplyBrakeValue(p_mc, value);
    return NULL;
}

/* App-specific commands — reads from Traction.Input (buffered by MotorController_Traction_ApplyDirection etc.) */
static State_T * Neutral_InputAppUser(MotorController_T * p_mc, state_value_t appCmd)
{
    Traction_Input_T * p_input = &TractionAdapter(p_mc)->Input;
    switch (appCmd)
    {
        case TRACTION_STATE_INPUT_DIRECTION:        return Neutral_InputDirection(p_mc, p_input->Direction);
        case TRACTION_STATE_INPUT_DRIVE_CMD:        return Neutral_InputCmdStart(p_mc, p_input->DriveCmd);
        case TRACTION_STATE_INPUT_BRAKE_VALUE:      return Neutral_InputBrakeValue(p_mc, p_input->BrakeValue);
        case TRACTION_STATE_INPUT_THROTTLE_VALUE:   return NULL; /* Throttle has no effect in Neutral */
        case TRACTION_STATE_INPUT_LEVER_DIRECTION:  return NULL; /* Neutral exit by direction select only */
        case TRACTION_STATE_INPUT_LEVER_VALUE:      return NULL;
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
    .PATH_ID      = { .Depth0 = MC_STATE_ID_MAIN }, /* sub-state id not yet allocated: reports as the root state */
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
sign_t MotorController_Traction_GetDirection(MotorController_T * p_mc)
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

/* Start edge, then the value of the active cmd */
static void ApplyDriveCmd(MotorController_T * p_mc, Traction_StateInput_T throttleValueInput)
{
    MotorController_Traction_PollStartCmd(p_mc);
    switch (TractionAdapter(p_mc)->Input.DriveCmd)
    {
        case TRACTION_CMD_BRAKE:     _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_BRAKE_VALUE);   break;
        case TRACTION_CMD_THROTTLE:  _MotorController_Traction_ApplyCmd(p_mc, throttleValueInput);                break;
        case TRACTION_CMD_RELEASE:   break;
        default: break;
    }
}

void MotorController_Traction_SetThrottleBrake(MotorController_T * p_mc, uint16_t throttle, uint16_t brake)
{
    TractionAdapter(p_mc)->Input.ThrottleValue = throttle;
    TractionAdapter(p_mc)->Input.BrakeValue = brake;
    ApplyDriveCmd(p_mc, TRACTION_STATE_INPUT_THROTTLE_VALUE);
}

/*
    Single axis lever, positive as user Forward. Resolves to [Traction_Cmd_T] by [Traction_LeverMode_T]:
        BRAKE_TO_STOP - Throttle/Brake along the drive direction, motoring frame
        THROUGH_ZERO  - Throttle along the lever, value applied signed
    Engage from release requests the lever direction.
*/
void MotorController_Traction_SetLever(MotorController_T * p_mc, int16_t lever)
{
    Traction_T * p_traction = TractionAdapter(p_mc);

    p_traction->Input.LeverValue = lever;
    if (Traction_Input_IsLeverEngage(&p_traction->Input) == true) { _MotorController_Traction_ApplyCmd(p_mc, TRACTION_STATE_INPUT_LEVER_DIRECTION); }
    Traction_Input_ResolveLever(&p_traction->Input, Traction_LeverReference(p_traction->Config.LeverMode, (sign_t)MotorController_GetDirection(p_mc), lever));
    ApplyDriveCmd(p_mc, (p_traction->Config.LeverMode == TRACTION_LEVER_MODE_THROUGH_ZERO) ? TRACTION_STATE_INPUT_LEVER_VALUE : TRACTION_STATE_INPUT_THROTTLE_VALUE);
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
static inline uint16_t AInThrottle(MotorController_T * p_mc) { return UserAIn_GetValue(&p_mc->AINS[MOT_AIN_THROTTLE].PIN); }
static inline uint16_t AInBrake(MotorController_T * p_mc) { return UserAIn_GetValue(&p_mc->AINS[MOT_AIN_BRAKE].PIN); }

#include "../MotAnalogUser/OptPin/MotorController_SwitchBrake.h"
void MotorController_Traction_ProcAnalogUser(MotorController_T * p_mc)
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
        case TRACTION_VAR_COMMAND:     break; /* read only, derived from throttle/brake */
        case TRACTION_VAR_LEVER:       MotorController_Traction_SetLever(p_mc, (int16_t)value);          break;
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
        case TRACTION_VAR_LEVER:       value = TractionAdapter(p_mc)->Input.LeverValue;     break;
        default: break;
    }
    return value;
}

int MotorController_Traction_ConfigId_Get(MotorController_T * p_mc, Traction_ConfigId_T id)
{
    return Traction_ConfigId_Get(&TractionAdapter(p_mc)->Config, id);
}

void MotorController_Traction_ConfigId_Set(MotorController_T * p_mc, Traction_ConfigId_T id, int value)
{
    Traction_ConfigId_Set(&TractionAdapter(p_mc)->Config, id, value);
}



