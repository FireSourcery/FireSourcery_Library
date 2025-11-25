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
    @file   MotorController_StateMachine.c
    @author FireSourcery
    @brief  MotorController StateMachine
*/
/******************************************************************************/
#include "MotorController_StateMachine.h"
#include "MotorController_App.h"
#include "Vehicle/Vehicle_StateMachine.h"

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Reboot.h)

/******************************************************************************/
/*!
    @brief
*/
/*
    ┌────────────────────────────────────────────────────────────┐
    │                   SYSTEM (Top Level)                       │
    │  ┌───────────┐  ┌──────────────────────────────────────┐   │
    │  │   INIT    │──│         OPERATIONAL                  │   │
    │  └───────────┘  │  ┌────────────────────────────────┐  │   │
    │                 │  │        PARK/STANDBY            │  │   │
    │                 │  └────────────────────────────────┘  │   │
    │                 │  ┌────────────────────────────────┐  │   │
    │                 │  │         SUB-TREE               │  │   │
    │                 │  │                                │  │   │
    │                 │  └────────────────────────────────┘  │   │
    │                 └──────────────────────────────────────┘   │
    │  ┌───────────┐                                             │
    │  │   FAULT   │◄────────────────────────────────────────────│
    │  └───────────┘                                             │
    └────────────────────────────────────────────────────────────┘
*/
/******************************************************************************/
static const State_T STATE_INIT;
static const State_T STATE_PARK;
static const State_T STATE_FAULT;

const StateMachine_Machine_T MCSM_MACHINE =
{
    .P_STATE_INITIAL = &STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MCSM_TRANSITION_TABLE_LENGTH,
};

/******************************************************************************/
/*!
    @brief Common
*/
/******************************************************************************/
/* Clear Latching */
/* Main thread only sets [FaultFlags]. call to check clear. via results of Monitor State */
/* PollMonitorFaults */
void MotorController_PollFaultFlags(MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    p_mc->FaultFlags.VSourceLimit = RangeMonitor_IsAnyFault(p_context->V_SOURCE.P_STATE);
    p_mc->FaultFlags.VAccsLimit = RangeMonitor_IsAnyFault(p_context->V_ACCESSORIES.P_STATE);
    p_mc->FaultFlags.VAnalogLimit = RangeMonitor_IsAnyFault(p_context->V_ANALOG.P_STATE);
    p_mc->FaultFlags.PcbOverheat = (Monitor_GetStatus(p_context->HEAT_PCB.P_STATE) == MONITOR_STATUS_FAULT);
    p_mc->FaultFlags.MosfetsOverheat = (HeatMonitor_Group_GetStatus(&p_context->HEAT_MOSFETS) == MONITOR_STATUS_FAULT);
}

static State_T * TransitionFault(MotorController_T * p_context, state_value_t faultFlags)
{
    (void)faultFlags; /* // p_context->FaultFlags.Value |= faultFlags; */
    return &STATE_FAULT;
}

/* Check Lock Common */
static State_T * Common_InputLock(MotorController_T * p_context, state_value_t lockId)
{
    (void)lockId;
    State_T * p_nextState = NULL;
    if ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER)
    {
        if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { p_nextState = &MC_STATE_LOCK; }
        // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == true) { p_nextState = &MC_STATE_LOCK; }
    }
    return p_nextState;
}


static State_T * Common_InputPark(MotorController_T * p_context)
{
    State_T * p_nextState = NULL;
    if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { p_nextState = &STATE_PARK; }
    // else { MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_NONE); }
    else { MotorController_BeepShort(p_context); }
    return p_nextState;
}

// input motor user direction with stop
// switch (p_input->Direction)
// {
//     case MOTOR_DIRECTION_NONE:  /* Called from serial only */
//         if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { p_nextState = &STATE_PARK; }
//         else { MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_NONE); }
//         break;
//     case MOTOR_DIRECTION_FORWARD:   MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_FORWARD);   break;
//     case MOTOR_DIRECTION_REVERSE:   MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_REVERSE);   break;
//     default:  break;
// }

/*
    MotorController AppTable
*/
static State_T * MainStateOf(MotorController_MainMode_T mode)
{
    switch (mode)
    {
        case MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD:  return &MC_STATE_MAIN_MOTOR_CMD;
        case MOTOR_CONTROLLER_MAIN_MODE_VEHICLE: return &MC_STATE_MAIN_VEHICLE;
        default: return &MC_STATE_MAIN_MOTOR_CMD;
    }
}

static State_T * GetMainState(const MotorController_T * p_context)
{
    return MainStateOf(p_context->P_MC_STATE->Config.InitMode);
    // return MotorController_App_GetMainState(&p_context->APPS, p_context->P_MC_STATE->Config.InitMode);
}


/*
    General Direction
*/
/*
    MOTOR_CONTROLLER_DIRECTION_PARK => MOTOR_DIRECTION_NONE
    MOTOR_CONTROLLER_DIRECTION_FORWARD => MOTOR_DIRECTION_FORWARD
    MOTOR_CONTROLLER_DIRECTION_REVERSE => MOTOR_DIRECTION_REVERSE
    MOTOR_CONTROLLER_DIRECTION_NEUTRAL => ERROR
    MOTOR_CONTROLLER_DIRECTION_ERROR => MOTOR_DIRECTION_ERROR
*/
// alternatively map getter to State
Motor_User_Direction_T MotorController_StateMachine_GetDirection(MotorController_T * p_context)
{
    switch (StateMachine_GetActiveStateId(p_context->STATE_MACHINE.P_ACTIVE))
    {
        case MCSM_STATE_ID_MAIN:       return _MotMotors_GetDirectionAll(&p_context->MOTORS); /* None is error in this case */
        case MCSM_STATE_ID_PARK:       return MOTOR_DIRECTION_NONE;
        case MCSM_STATE_ID_LOCK:       return MOTOR_DIRECTION_NONE;
        case MCSM_STATE_ID_FAULT:      return MOTOR_DIRECTION_NONE;
        default:                       return MOTOR_DIRECTION_NONE;
    }
}

/******************************************************************************/
/*!
    Multi Input to Multi States

    Input Aggregation -> State Table
        All State accept aggregated inputs

    All State handle each input type
        App State handles Each input type

    App layer determines input mapping to aggregated input or each states input index
    or determine mapping callback to each input module

*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief Init State

    Init State does not transistion to fault, wait for ADC
*/
/******************************************************************************/
#define MC_STATE_MACHINE_INIT_WAIT (1500U + 50U) /* Let 1000ms ADC sampling process once */

static_assert(MC_STATE_MACHINE_INIT_WAIT > MOTOR_STATE_MACHINE_INIT_WAIT);

static void Init_Entry(const MotorController_T * p_context)
{
    (void)p_context;
}

// static void Init_Exit(const MotorController_T * p_context)
// {
//     MotorController_BeepShort(p_context);
// }

static void Init_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
}

static State_T * Init_Next(const MotorController_T * p_context)
{
    State_T * p_nextState = NULL;
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    /* Wait for initial ADC readings */
    // wait for every motor exit init
    if (SysTime_GetMillis() > MC_STATE_MACHINE_INIT_WAIT)
    {
        MotorController_PollFaultFlags(p_context); /* Clear fault flags set by sensor polling in Main thread */

        if (Phase_Calibration_IsLoaded() == false) { p_mc->FaultFlags.InitCheck = 1U; }
        /* Enforce VMonitor Enable */ /* Disabled on invalid config */
        if (RangeMonitor_IsEnabled(p_context->V_SOURCE.P_STATE) == false) { p_mc->FaultFlags.InitCheck = 1U; p_mc->FaultFlags.VSourceLimit = 1U; }
        // if (RangeMonitor_ValidateConfig(p_context->V_SOURCE.P_STATE) == false) { p_mc->FaultFlags.VSourceLimit = 1U; }

        // if (BootRef_IsValid() == false) { MotorController_InitVSupplyAutoValue(p_context); } /* optionally auto on first time boot */

        if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == false) { p_mc->FaultFlags.Motors = 1U; }

        /* In the case of boot into motor spinning state. Go to fault state disable output */
        if (p_mc->FaultFlags.Value == 0U)
        {
            MotorController_BeepShort(p_context);
            // p_nextState = &MC_STATE_MAIN;
            p_nextState = &STATE_PARK;
        }
        else
        {
            p_nextState = &STATE_FAULT;
        }
    }

    return p_nextState;
}

static const State_Input_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]  = NULL, /* MotorController_StateMachine_EnterFault is disabled for INIT_STATE */
};

static const State_T STATE_INIT =
{
    .ID                 = MCSM_STATE_ID_INIT,
    .ENTRY              = (State_Action_T)Init_Entry,
    .LOOP               = (State_Action_T)Init_Proc,
    .NEXT               = (State_InputVoid_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief [Park] Top-level safe state for stationary operation
    Motor State: Stop.

    May enter from Neutral State or Drive State
*/
/******************************************************************************/
static void Park_Entry(const MotorController_T * p_context)
{
    // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { MotMotors_ActivateControlState(&p_context->MOTORS, PHASE_OUTPUT_V0); }
}

static void Park_Proc(const MotorController_T * p_context)
{

}

// Background monitoring (e.g., poll for user inputs to exit Park)
// static State_T * Park_Next(const MotorController_T * p_context)
// {
//     // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == false) {  // Optional: set warning or force stop }
//     // Motor_User_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;
//     State_T * p_nextState = NULL;
//     return p_nextState;
// }

static State_T * Park_InputLock(const MotorController_T * p_context, state_value_t lockId)
{
    return Common_InputLock(p_context, lockId);  // Reuse for entering LOCK
}

/* Consistent Park transition */
static State_T * Park_InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
{
    switch ((MotorController_StateCmd_T)cmd)
    {
        // case MOTOR_CONTROLLER_STATE_CMD_PARK: return Common_InputPark(p_context);
        // case MOTOR_CONTROLLER_STATE_CMD_E_STOP: return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN: MotMotors_StopAll(&p_context->MOTORS); break;
        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN: return GetMainState(p_context); /* Protocol ExitPPark */
        default:  break;
    }
    return NULL;
}

/* Consistent Park transition for AnalogUser */
static State_T * Park_InputDirection(const MotorController_T * p_context, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((Motor_User_Direction_T)direction)
    {
        case MOTOR_DIRECTION_NONE:    break; // optionally apply V0 or VFLOAT
        case MOTOR_DIRECTION_FORWARD: p_nextState = GetMainState(p_context); break;
        case MOTOR_DIRECTION_REVERSE: p_nextState = GetMainState(p_context); break;
        default:  break;
    }
    return p_nextState;
}

// static State_T * Park_InputBuffer(const MotorController_T * p_context, state_value_t cmd)
// {
//     if (p_input->IsUpdated == true)
//     {
//         MotMotors_ApplyInputs(&p_context->MOTORS, p_input);

//         switch (p_input->Direction)
//         {
//             case MOTOR_DIRECTION_NONE:    break; // optionally apply V0 or VFLOAT
//             case MOTOR_DIRECTION_FORWARD: p_nextState = GetMainState(p_context); break;
//             case MOTOR_DIRECTION_REVERSE: p_nextState = GetMainState(p_context); break;
//             default:  break;
//         }

//         p_input->IsUpdated = false;
//     }

//     return NULL;
// }

static const State_Input_T PARK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK]           = (State_Input_T)Park_InputLock,
    [MCSM_INPUT_STATE_COMMAND]  = (State_Input_T)Park_InputStateCmd,
    [MCSM_INPUT_DIRECTION]      = (State_Input_T)Park_InputDirection,
// [MCSM_INPUT_STATE_GENERIC]      = (State_Input_T)Park_InputServo,
// #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
//     [VEHICLE_INPUT_SERVO]      = (State_Input_T)Park_InputServo,
// #endif
    // [VEHICLE_INPUT_USER_MODE]  = (State_Input_T)Park_InputUser,
};

static const State_T STATE_PARK =
{
    .ID     = MCSM_STATE_ID_PARK,
    .ENTRY  = (State_Action_T)Park_Entry,
    .LOOP   = (State_Action_T)Park_Proc,
    // .NEXT   = (State_InputVoid_T)Park_Next,
    .P_TRANSITION_TABLE = &PARK_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @name Main App State

    Motor States: STOP, PASSIVE, RUN, OPEN_LOOP
    Top State as Main_Stop/Idle  before transition
    @{
*/
/******************************************************************************/
static void Main_Entry(const MotorController_T * p_context)
{
    MotMotors_ActivateControlState(&p_context->MOTORS, PHASE_OUTPUT_FLOAT);
}

static void Main_Proc(const MotorController_T * p_context)
{

}

/* Submachine common transitions */
/* handle motor exit stop */
static State_T * Main_InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
{
    switch (cmd)
    {
        case MOTOR_CONTROLLER_STATE_CMD_PARK: return Common_InputPark(p_context);
        case MOTOR_CONTROLLER_STATE_CMD_E_STOP: return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN: return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN:
            if (StateMachine_GetLeafState(p_context->STATE_MACHINE.P_ACTIVE) == &MC_STATE_MAIN) /* At Main Top */
            {
                // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP))
                return GetMainState(p_context); /* transition to main top state. stops processing inputs */
            }
            break;
        default:  break;
    }
    return NULL;
}


/*
    Motors Maybe spinning in same direction
*/
/* Top level overridable */
// static State_T * Main_InputForward(const MotorController_T * p_context)
// {
//     if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsDirectionForward) == true) { return GetMainState(p_context); }
//     if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_FORWARD); return GetMainState(p_context); }
//     return NULL;
//     // return (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsDirectionForward) == true) ? GetMainState(p_context) : NULL;
// }

// static State_T * Main_InputReverse(const MotorController_T * p_context)
// {
//     if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsDirectionReverse) == true) { return GetMainState(p_context); }
//     if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsSpeedZero) == true) { MotMotors_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_REVERSE); return GetMainState(p_context); }
//     return NULL;
//     // return (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsDirectionReverse) == true) ? GetMainState(p_context) : NULL;
// }

// /* Overridden after transition to substate */
// static State_T * Main_InputDirection(const MotorController_T * p_context, state_value_t direction)
// {
//     State_T * p_nextState = NULL;
//     switch ((Motor_User_Direction_T)direction)
//     {
//         case MOTOR_DIRECTION_FORWARD: p_nextState = Main_InputForward(p_context); break;
//         case MOTOR_DIRECTION_REVERSE: p_nextState = Main_InputReverse(p_context); break;
//         case MOTOR_DIRECTION_NONE: p_nextState = NULL; break;
//         default: break;
//     }
//     return p_nextState;
// }

static State_T * Main_InputDirection(const MotorController_T * p_context, state_value_t direction)
{
    // if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_User_IsDirection, (Motor_User_Direction_T)direction) == true) { return GetMainState(p_context); }
    if (MotMotors_IsEveryUserDirection(&p_context->MOTORS, (Motor_User_Direction_T)direction)) { return GetMainState(p_context); }
    if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsSpeedZero)) { MotMotors_ApplyUserDirection(&p_context->MOTORS, (Motor_User_Direction_T)direction); return GetMainState(p_context); }
    return NULL;
}

/*
    Transition to Lock to select substate
*/
static const State_Input_T MAIN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_STATE_COMMAND]  = (State_Input_T)Main_InputStateCmd,
    [MCSM_INPUT_DIRECTION]      = (State_Input_T)Main_InputDirection,
    // [MCSM_INPUT_GENERIC]     = (State_Input_T)Main_ ,
    // [MCSM_INPUT_MAIN_MODE]      = (State_Input_T)Main_InputMainMode,
};

const State_T MC_STATE_MAIN =
{
    .ID     = MCSM_STATE_ID_MAIN,
    .ENTRY  = (State_Action_T)Main_Entry,
    .LOOP   = (State_Action_T)Main_Proc,
    .P_TRANSITION_TABLE = &MAIN_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief Default Substate Motor Command
    Motors passthrough coordinated default
    marker for accepting [MotorController_User_Set] interface inputs

    Call handles enabling disabling output, FeedbackMode

    optionally move neutral to top state
*/
/******************************************************************************/
static void MotorCmd_Entry(const MotorController_T * p_context)
{
    // MotorController_State_T * p_mc = p_context->P_MC_STATE;
    MotMotors_ActivateControlState(&p_context->MOTORS, PHASE_OUTPUT_VPWM); /* Set PWM Output */
}

/* Motor_User_Input_T   Only */
static void MotorCmd_Proc(const MotorController_T * p_context)
{
    Motor_User_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;

    /* FeedbackMode must be set by protocol */
    // tod  handle in app
//     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG: // effectively Motor_Input_OfMotAnalogUser / Motor_Input_OfVehicleInput
//         switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
//         {
//             case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  p_input->Direction = MOTOR_DIRECTION_FORWARD; p_input->PhaseState = PHASE_OUTPUT_VPWM;  break;
//             case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  p_input->Direction = MOTOR_DIRECTION_REVERSE; p_input->PhaseState = PHASE_OUTPUT_VPWM;  break;
//             case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  p_input->PhaseState = PHASE_OUTPUT_FLOAT;         break; // p_input->Direction = MOTOR_DIRECTION_NONE;// or return to top main
//             default: break;
//         }

//         if (MotAnalogUser_IsAnyBrakeOn(&p_context->ANALOG_USER) == true)
//         {
//             p_input->CmdValue = 0U;
//             p_input->PhaseState = PHASE_OUTPUT_FLOAT;
//         }
//         else
//         {
//             p_input->CmdValue = MotAnalogUser_GetThrottle(&p_context->ANALOG_USER) / 2U;
//         }
    // MotMotors_ApplyInputs(&p_context->MOTORS, p_input); // passthrough buffered, or implement var for apply

}

static State_T * MotorCmd_InputDirection(const MotorController_T * p_context, state_value_t direction)
{
    // if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_User_IsDirection, (Motor_User_Direction_T)direction) == true) { return GetMainState(p_context); }
    if (MotMotors_IsEvery(&p_context->MOTORS, Motor_IsSpeedZero)) { MotMotors_ApplyUserDirection(&p_context->MOTORS, (Motor_User_Direction_T)direction); }
    return NULL;
}

const State_T MC_STATE_MAIN_MOTOR_CMD =
{
    .ID         = MOTOR_CONTROLLER_MAIN_MODE_MOTOR_CMD, /* as StateId */
    .DEPTH      = 1U,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .ENTRY      = (State_Action_T)MotorCmd_Entry,
    .LOOP       = (State_Action_T)MotorCmd_Proc,
    // .P_TRANSITION_TABLE = &MOTORS_TRANSITION_TABLE[0U],
};



/******************************************************************************/
/*!
    @brief [Lock] State - Blocking functions, and extended async operations
    Nvm functions
    Calibration routines. set status id upon completion.
*/
/******************************************************************************/
static const State_T MC_STATE_LOCK_CALIBRATE_ADC;

static void Lock_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    assert(MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP));
    // _StateMachine_EndSubState(p_context->STATE_MACHINE.P_ACTIVE);

    MotMotors_EnterCalibration(&p_context->MOTORS); /* Enter Calibration State for all motors */
    // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }

    p_mc->LockOpStatus = 0U;

    MotorController_BeepShort(p_context);
}

static void Lock_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }
    // _StateMachine_ProcBranch_Nested(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context);
}

/* Lock SubState/Cmd by passed value */
/* alternatively StateMachine_TransitionInput_T replace lockId */
static State_T * Lock_InputLockOp_Blocking(const MotorController_T * p_context, state_value_t lockId)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    State_T * p_nextState = NULL;

    /* From Top state only. no sub state active. */
    if (StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK))
    {
        switch ((MotorController_LockId_T)lockId)
        {
            case MOTOR_CONTROLLER_LOCK_ENTER:
                p_mc->LockOpStatus = 0;
                break;

            case MOTOR_CONTROLLER_LOCK_EXIT:
                // MotMotors_StopAll(&p_context->MOTORS);
                if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == true)
                {
                    MotMotors_StopAll(&p_context->MOTORS);
                    p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                    p_nextState = &STATE_PARK;
                }
                else if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true)
                {
                    p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                    p_nextState = &STATE_PARK;
                }
                else
                {
                    p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR;
                }
                break;

            /* todo check start from top state only substate == current state */
            case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
                p_mc->NvmStatus = MotNvm_SaveConfigAll_Blocking(&p_context->MOT_NVM); /* NvM function will block + disable interrupts */
                p_mc->LockOpStatus = 0;
                break;

            case MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG:
                break;

            case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC: /* alternatively split */
                MotMotors_EnterCalibrateAdc(&p_context->MOTORS); /* Motor handles it own state */
                p_nextState = &MC_STATE_LOCK_CALIBRATE_ADC; /* Enter Calibration SubState */
                break;


            case MOTOR_CONTROLLER_LOCK_REBOOT:
                HAL_Reboot();  // optionally deinit clock select
                /* No return */
                break;

            // case MOTOR_CONTROLLER_NVM_BOOT:                  p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_context);       break;
            // case MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_WriteOnce_Blocking(p_context);         break;
            // case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC_ALL:    break;
            /* Generic select or call motor function */
            // case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:  StartCalibrateSensor(p_context);    break;
        }
    }
    else
    {
        p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR;
    }

    return p_nextState;
}

static State_T * Lock_InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
{
    switch (cmd)
    {
        case MOTOR_CONTROLLER_STATE_CMD_PARK: return Common_InputPark(p_context);
        case MOTOR_CONTROLLER_STATE_CMD_E_STOP: return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        default:  break;
    }
}

/* Transition from lock only */
/* for run time experimental mode without reboot. alternatively on init only */
// static State_T * Lock_InputMainMode(const MotorController_T * p_context, state_value_t mainMode)
// {
//     State_T * p_nextState = NULL;

//     if (!StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK)) { p_nextState = NULL; }
//     else if (!MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP)) { p_nextState = NULL; }
//     /* Allow temporary mapping to experimental state */
//     else { p_nextState = GetMainState(p_context); } // change to selected state

//     return p_nextState;
// }

static const State_Input_T LOCK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK]           = (State_Input_T)Lock_InputLockOp_Blocking,
    // [MCSM_INPUT_MAIN_MODE]      = (State_Input_T)Lock_InputMainMode,
    [MCSM_INPUT_STATE_COMMAND]  = (State_Input_T)Lock_InputStateCmd,
};

const State_T MC_STATE_LOCK =
{
    .ID                 = MCSM_STATE_ID_LOCK,
    .ENTRY              = (State_Action_T)Lock_Entry,
    .LOOP               = (State_Action_T)Lock_Proc,
    .P_TRANSITION_TABLE = &LOCK_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief CalibrateAdc SubState
*/
/******************************************************************************/
void StartCalibrateAdc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    p_mc->StateCounter = 0U;
    Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
    Analog_Conversion_ClearResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_ClearResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
    Filter_Init(&p_mc->AvgBuffer0);
    Filter_Init(&p_mc->AvgBuffer1);
    // MotMotors_EnterCalibrateAdc(&p_context->MOTORS); /* Motor handles it own state */
}

/* Proc Per ms */
void ProcCalibrateAdc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    if (p_mc->StateCounter != 0U) /* skip first time */
    {
        Filter_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE));
        Filter_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE));
        Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE);
        Analog_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS.BRAKE);
    }

    p_mc->StateCounter++;
}

static State_T * EndCalibrateAdc(const MotorController_T * p_context)
{
    const uint32_t TIME = 2000U; /* > Motor calibrate adc time */

    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    State_T * p_nextState = NULL;

    if (p_mc->StateCounter > TIME)
    {
        // if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false)
        MotAnalogUser_SetThrottleZero(&p_context->ANALOG_USER, Filter_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.THROTTLE)));
        MotAnalogUser_SetBrakeZero(&p_context->ANALOG_USER, Filter_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_context->ANALOG_USER_CONVERSIONS.BRAKE)));
        p_mc->LockOpStatus = 0; /* success */

        p_nextState = &MC_STATE_LOCK; /* return to lock state */
    }

    return p_nextState;
}

static const State_T MC_STATE_LOCK_CALIBRATE_ADC =
{
    .ID = MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC,
    .P_TOP = &MC_STATE_LOCK,
    .P_PARENT = &MC_STATE_LOCK,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)StartCalibrateAdc,
    .LOOP = (State_Action_T)ProcCalibrateAdc,
    .NEXT = (State_InputVoid_T)EndCalibrateAdc,
};

// static State_T * Lock_CalibrateAdc(const MotorController_T * p_context, state_value_t lockId)
// {
//     return &MC_STATE_LOCK_CALIBRATE_ADC;
// }

// void MotorController_Lock_CalibrateAdc(const MotorController_T * p_context)
// {
//     static const StateMachine_TransitionInput_T CMD = { .P_START = &MC_STATE_LOCK, .TRANSITION = (State_Input_T)Lock_CalibrateAdc, };
//     StateMachine_Branch_InvokeTransition(&p_context->STATE_MACHINE, &CMD, 0);
// }

/******************************************************************************/
/*!
    @brief Calibrate Sensor SubState
*/
/******************************************************************************/
// if (MotorController_IsAnyMotorFault(p_context) == true) { p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER; p_mc->LockOpStatus = 1U; }

// simplify check complete
// static State_T * EndCalibrateSensor(const MotorController_T * p_context)
// {
//     MotorController_State_T * p_mc = p_context->P_MC_STATE;
//     State_T * p_nextState = NULL;

//     if (MotMotors_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false)
//     {
//         p_nextState = &MC_STATE_LOCK; /* return to lock state */
//     }

//     return p_nextState;
// }

// static const State_T STATE_LOCK_CALIBRATE_MOTOR_SENSOR =
// {
//     .P_TOP = &MC_STATE_LOCK,
//     .P_PARENT = &MC_STATE_LOCK,
//     .DEPTH = 1U,
//     .ENTRY = (State_Action_T)0,
//     .LOOP = (State_Action_T)0,
//     .NEXT = (State_InputVoid_T)0,
// };



/******************************************************************************/
/*!
    @brief Fault State
*/
/******************************************************************************/
static void Fault_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    MotMotors_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */
// #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
//     memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
// #endif
    MotorController_BeepPeriodic(p_context);
}

static void Fault_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    MotMotors_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */

    // switch (p_mc->Config.InputMode)
    // {
    //     /* Protocol Rx Lost use auto recover, without user input */
    //     case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    p_mc->FaultFlags.RxLost = MotorController_PollRxLost(p_context); break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_CAN:       break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    break;
    // }

    if (p_mc->FaultFlags.Value == 0U)
    {
        Blinky_Stop(&p_context->BUZZER);
        // _StateMachine_Transition(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context, &MC_STATE_MAIN);
    }
}

/* Fault State Input Fault Checks Fault */
/* Sensor faults only clear on user input */
static State_T * Fault_InputClearFault(const MotorController_T * p_context, state_value_t faultFlags)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    // p_mc->FaultFlags.Value &= faultFlags; //repeat calls with filled flags do not clear
    p_mc->FaultFlags.Value = 0U;
    MotorController_PollFaultFlags(p_context);
    // p_mc->FaultFlags.Motors = 0U; /* updated by [MotorController_Main_Thread] */
    for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_StateMachine_ExitFault(&p_context->MOTORS.P_CONTEXTS[iMotor]); }
    // return NULL;
    Blinky_Stop(&p_context->BUZZER); /* Stops until its set again */
    return (p_mc->FaultFlags.Value == 0U) ? &MC_STATE_MAIN : NULL;
}

static State_T * Fault_InputLockSaveConfig_Blocking(const MotorController_T * p_context, state_value_t lockId)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    // p_mc->LockSubState = lockId;
    // p_mc->LockOpStatus = -1;
    switch ((MotorController_LockId_T)lockId)
    {
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
            p_mc->NvmStatus = MotNvm_SaveConfigAll_Blocking(&p_context->MOT_NVM);
            p_mc->LockOpStatus = p_mc->NvmStatus;
            break;
        default: break;
    }

    return NULL;
}

static const State_Input_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]  = (State_Input_T)Fault_InputClearFault,
    [MCSM_INPUT_LOCK]   = (State_Input_T)Fault_InputLockSaveConfig_Blocking,
};

static const State_T STATE_FAULT =
{
    .ID                 = MCSM_STATE_ID_FAULT,
    .ENTRY              = (State_Action_T)Fault_Entry,
    .LOOP               = (State_Action_T)Fault_Proc,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*
    Fault Interface Functions
*/
/******************************************************************************/
/* todo thread safe without lock */
void MotorController_StateMachine_EnterFault(const MotorController_T * p_context)
{
    if (MotorController_StateMachine_IsFault(p_context) == false) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, -1); }
}

bool MotorController_StateMachine_ExitFault(const MotorController_T * p_context)
{
    if (MotorController_StateMachine_IsFault(p_context) == true) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, 0); }
    return !MotorController_StateMachine_IsFault(p_context);
}

// ((const MotorController_FaultFlags_T) { .VAccsLimit = 1U }).Value
void MotorController_StateMachine_SetFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    if (MotorController_StateMachine_IsFault(p_context) == false) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, faultFlags); }
}

/* ensure repeat inputs without lock do not mismatch */
void MotorController_StateMachine_ClearFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    if (MotorController_StateMachine_IsFault(p_context) == true) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, ~faultFlags); }
    // return !MotorController_StateMachine_IsFault(p_context); /* alternatively use cleared diff */
}



