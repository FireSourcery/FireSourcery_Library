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
    │                 │  │         SUB-TREE (APP)         │  │   │
    │                 │  │                                │  │   │
    │                 │  └────────────────────────────────┘  │   │
    │                 └──────────────────────────────────────┘   │
    │  ┌───────────┐                                             │
    │  │   FAULT   │◄────────────────────────────────────────────│
    │  └───────────┘                                             │
    └────────────────────────────────────────────────────────────┘
*/
/*
    Top layer handles Park State
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
/*
    MotorController AppTable
*/
static State_T * GetMainState(MotorController_T * p_context) { return MotorController_App_GetMainState(p_context); }
static State_T * ParkState(MotorController_T * p_context) { return(p_context->P_MC_STATE->Config.isParkStateEnabled ? &STATE_PARK : &MC_STATE_MAIN); }

/* Clear Latching */
/* Main thread only sets [FaultFlags]. call to check clear. via results of Monitor State */
/* PollMonitorFaults */
/* InitCheck clears on reset only */
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
    State_T * p_nextState = NULL;
    if ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER)
    {
        if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { p_nextState = &MC_STATE_LOCK; }
        // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == true) { p_nextState = &MC_STATE_LOCK; }
    }
    return p_nextState;
}


static State_T * Common_InputPark(MotorController_T * p_context)
{
    State_T * p_nextState = NULL;
    if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { p_nextState = ParkState(p_context); }
    // else { Motor_Table_ApplyUserDirection(&p_context->MOTORS, MOTOR_DIRECTION_NULL); }
    else { MotorController_BeepShort(p_context); }
    return p_nextState;
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
        MotorController_PollFaultFlags(p_context); /* Clear latching fault flags set by sensor polling in Main thread */

        if (Phase_Calibration_IsLoaded() == false) { p_mc->FaultFlags.InitCheck = 1U; }
        /* Enforce VMonitor Enable */ /* Disabled on invalid config */
        if (RangeMonitor_IsEnabled(p_context->V_SOURCE.P_STATE) == false) { p_mc->FaultFlags.InitCheck = 1U; p_mc->FaultFlags.VSourceLimit = 1U; }
        /* Check separately */
        // if (RangeMonitor_ValidateConfig(p_context->V_SOURCE.P_STATE) == false) { p_mc->FaultFlags.VSourceLimit = 1U; }

        // if (BootRef_IsValid() == false) { MotorController_InitVSupplyAutoValue(p_context); } /* optionally auto on first time boot */

        if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == false) { p_mc->FaultFlags.Motors = 1U; }

        /* In the case of boot into motor spinning state. Go to fault state disable output */
        if (p_mc->FaultFlags.Value == 0U)
        {
            MotorController_BeepShort(p_context);
            // p_nextState = (p_mc->Config.IsParkStateEnabled) ? &STATE_PARK : &MC_STATE_MAIN;
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
    [MCSM_INPUT_FAULT]  = NULL, /* MotorController_EnterFault is disabled for INIT_STATE */
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
    // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true) { Motor_Table_StopAll(&p_context->MOTORS); Motor_Table_ActivateVOutput(&p_context->MOTORS, PHASE_OUTPUT_VFLOAT); }
}

static void Park_Proc(const MotorController_T * p_context)
{

}

// Background monitoring (e.g., poll for user inputs to exit Park)
// static State_T * Park_Next(const MotorController_T * p_context)
// {
//     // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == false) {  // Optional: set warning or force stop }
//     // Motor_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;
//     State_T * p_nextState = NULL;
//     return p_nextState;
// }

static State_T * Park_InputLock(const MotorController_T * p_context, state_value_t lockId)
{
    return Common_InputLock(p_context, lockId);
}

/* Consistent Park transition */
static State_T * Park_InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
{
    switch ((MotorController_StateCmd_T)cmd)
    {
        // case MOTOR_CONTROLLER_STATE_CMD_PARK:
        // case MOTOR_CONTROLLER_STATE_CMD_E_STOP:
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN: Motor_Table_StopAll(&p_context->MOTORS); break;
        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN: return GetMainState(p_context); /* Protocol ExitPPark, protocol apply direction */
        default:  break;
    }
    return NULL;
}

static State_T * Park_InputDirection(const MotorController_T * p_context, state_value_t direction)
{
    State_T * p_nextState = NULL;
    switch ((int)direction)
    {
        case 0:    break; // optionally apply V0 or VFLOAT
        case 1: Motor_Table_ApplyUserDirection(&p_context->MOTORS, (int)direction); p_nextState = GetMainState(p_context); break;
        case -1: Motor_Table_ApplyUserDirection(&p_context->MOTORS, (int)direction); p_nextState = GetMainState(p_context); break;
        default:  break;
    }
    return p_nextState;
}

static State_T * Park_InputUser(const MotorController_T * p_context, state_value_t event)
{
    switch ((MotorController_UserEvent_T)event)
    {
        case MOTOR_CONTROLLER_USER_CMD_DIRECTION: return Park_InputDirection(p_context, p_context->P_MC_STATE->CmdInput.Direction);
        // case MOTOR_CONTROLLER_USER_CMD_PHASE: // MotorController_InputPhaseOutput(p_context, (Phase_Output_T)p_context->P_MC_STATE->CmdInput.Phase);
        // case MOTOR_CONTROLLER_USER_CMD_SETPOINT: // MotorController_InputStateCommand(p_context, (MotorController_StateCmd_T)p_context->P_MC_STATE->CmdInput.CmdValue); break;
        default: break;
    }

    return NULL;
}

static const State_Input_T PARK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK]           = (State_Input_T)Park_InputLock,
    [MCSM_INPUT_STATE_COMMAND]  = (State_Input_T)Park_InputStateCmd,
    [MCSM_INPUT_USER]           = (State_Input_T)Park_InputUser,
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
    Base State as Stop/Idle before transition
    A mounting point for app states
    @{
*/
/******************************************************************************/
static void Main_Entry(const MotorController_T * p_context)
{
    // Motor_Table_ActivateVOutput(&p_context->MOTORS, PHASE_OUTPUT_FLOAT);
    // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == false) { Motor_Table_StopAll(&p_context->MOTORS); }
}

/* App State common background proc */
static void Main_Proc(const MotorController_T * p_context)
{
}

/* App State defaults transitions */
static State_T * Main_InputStateCmd(const MotorController_T * p_context, state_value_t cmd)
{
    switch (cmd)
    {
        case MOTOR_CONTROLLER_STATE_CMD_PARK: return Common_InputPark(p_context); /* Motors in Stop first */
        case MOTOR_CONTROLLER_STATE_CMD_E_STOP: //return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:
            Motor_Table_ActivateVOutput(&p_context->MOTORS, PHASE_OUTPUT_FLOAT);
            Motor_Table_StopAll(&p_context->MOTORS);
            return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
            //alternatively mount app state at root level so it cannot be exited

        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN:
            if (StateMachine_GetLeafState(p_context->STATE_MACHINE.P_ACTIVE) == &MC_STATE_MAIN) /* At Main Top */
            {
                // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP))
                return GetMainState(p_context); /* transition to main sub state. start processing inputs */
                // direction must be set
            }
            break;
        default:  break;
    }
    return NULL;
}

/* Main idle base */
static State_T * Main_InputDirection(MotorController_T * p_context, state_value_t direction)
{
    if (Motor_Table_IsEveryUserDirection(&p_context->MOTORS, direction) == true) { return GetMainState(p_context); }
    if (Motor_Table_IsEvery(&p_context->MOTORS, Motor_IsSpeedZero) == true) { Motor_Table_ApplyUserDirection(&p_context->MOTORS, direction); return GetMainState(p_context);; }
    return NULL;
}

static State_T * Main_InputUser(const MotorController_T * p_context, state_value_t event)
{
    Motor_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;
    switch ((MotorController_UserEvent_T)event)
    {
        case MOTOR_CONTROLLER_USER_CMD_DIRECTION:   Motor_Table_ApplyUserDirection(&p_context->MOTORS, p_input->Direction); break;
        case MOTOR_CONTROLLER_USER_CMD_PHASE:       Motor_Table_ActivateVOutput(&p_context->MOTORS, p_input->PhaseOutput); break;
        // case MOTOR_CONTROLLER_USER_CMD_SETPOINT:    Motor_Table_SetCmdWith(&p_context->MOTORS, Motor_SetActiveCmdValue_Scalar, p_input->CmdValue); break;
        // case MOTOR_CONTROLLER_USER_CMD_FEEDBACK:    Motor_Table_ApplyFeedbackMode(&p_context->MOTORS, p_input->FeedbackMode); break;
        default:  break;
    }
    return NULL;
}

/*
    Transition to Lock to select substate
*/
static const State_Input_T MAIN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_STATE_COMMAND]  = (State_Input_T)Main_InputStateCmd,
    [MCSM_INPUT_USER]           = (State_Input_T)Main_InputUser,
    // [MCSM_INPUT_DIRECTION]      = (State_Input_T)Main_InputDirection,
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
    marker for accepting [Motor_VarId] interface inputs

    Call handles enabling disabling output, FeedbackMode

    optionally move neutral to top state

    protocol MotorCmd enable even for analog input only
*/
/******************************************************************************/
static void MotorCmd_Entry(const MotorController_T * p_context)
{
    // MotorController_State_T * p_mc = p_context->P_MC_STATE;
    Motor_Table_ActivateVOutput(&p_context->MOTORS, PHASE_OUTPUT_FLOAT); /* Set PWM Output */
}

/* Motor_Input_T Only */
static void MotorCmd_Proc(const MotorController_T * p_context)
{
    // Motor_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;
    // Motor_Table_ApplyInputs(&p_context->MOTORS, p_input); // passthrough buffered, or implement var for apply
}

/* DIRECTION_NONE => Stop by default */
// static State_T * MotorCmd_InputDirection(const MotorController_T * p_context, state_value_t direction)
// {
//     Motor_Table_ApplyUserDirection(&p_context->MOTORS, (Motor_Direction_T)direction); /* Motor maintain direction state */
//     return NULL;
// }

static State_T * MotorCmd_Input(const MotorController_T * p_context, state_value_t cmd)
{
    Motor_Input_T * p_input = &p_context->P_MC_STATE->CmdInput;
    switch ((MotorController_UserEvent_T)cmd)
    {
        case MOTOR_CONTROLLER_USER_CMD_SETPOINT:  Motor_Table_SetCmdWith(&p_context->MOTORS, Motor_SetActiveCmdValue_Scalar, p_input->CmdValue); break;
        case MOTOR_CONTROLLER_USER_CMD_PHASE:  Motor_Table_ActivateVOutput(&p_context->MOTORS, p_input->PhaseOutput); break;
        case MOTOR_CONTROLLER_USER_CMD_FEEDBACK:  Motor_Table_ApplyFeedbackMode(&p_context->MOTORS, p_input->FeedbackMode); break;
        case MOTOR_CONTROLLER_USER_CMD_DIRECTION: Motor_Table_ApplyUserDirection(&p_context->MOTORS, p_input->Direction); break;
        default:  break;
    }
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
    .P_TRANSITION_TABLE = (State_Input_T[MCSM_TRANSITION_TABLE_LENGTH])
    {
        [MCSM_INPUT_USER]        = (State_Input_T)MotorCmd_Input,
    },
    .TRANSITION_MAPPER = NULL,
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

    assert(Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP));
    Motor_Table_StopAll(&p_context->MOTORS);
    // _StateMachine_EndSubState(p_context->STATE_MACHINE.P_ACTIVE);

    Motor_Table_EnterCalibration(&p_context->MOTORS); /* Enter Calibration State for all motors */
    // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }

    p_mc->LockOpStatus = 0U;

    MotorController_BeepShort(p_context);
}

static void Lock_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;
    // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }
    // _StateMachine_Branch_Proc_Nested(p_context->STATE_MACHINE.P_ACTIVE, (void *)p_context);
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
                // Motor_Table_StopAll(&p_context->MOTORS);
                if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == true)
                {
                    Motor_Table_StopAll(&p_context->MOTORS);
                    p_mc->LockOpStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                    p_nextState = &STATE_PARK;
                }
                else if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP) == true)
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
                Motor_Table_EnterCalibrateAdc(&p_context->MOTORS); /* Motor handles it own state */
                p_nextState = &MC_STATE_LOCK_CALIBRATE_ADC; /* Enter Calibration SubState */
                break;

                /* Generic select or call motor function */
            // case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:  StartCalibrateSensor(p_context);    break;
                // MOTOR_CONTROLLER_LOCK_MOTOR_CMD_MODE

            /* No return */
            case MOTOR_CONTROLLER_LOCK_REBOOT:
                HAL_Reboot();  // optionally deinit clock select
                break;

            case MOTOR_CONTROLLER_LOCK_MOTOR_CMD_MODE:
                p_mc->LockOpStatus = 0;
                p_nextState = &MC_STATE_MAIN_MOTOR_CMD; /* Motor disable on entry */
                break;

            // case MOTOR_CONTROLLER_NVM_BOOT:                  p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_context);       break;
            // case MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_WriteOnce_Blocking(p_context);         break;

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
        case MOTOR_CONTROLLER_STATE_CMD_PARK:       return Common_InputPark(p_context);
        case MOTOR_CONTROLLER_STATE_CMD_E_STOP:     return &MC_STATE_MAIN; /* transition to main top state. stops processing inputs */
        // case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:  return NULL;
        // case MOTOR_CONTROLLER_STATE_CMD_START_MAIN: return &MC_STATE_MAIN;
        default:                                    return NULL;
    }
}

/* Transition from lock only */
/* for run time experimental mode without reboot. alternatively on init only */
// static State_T * Lock_InputMainMode(const MotorController_T * p_context, state_value_t mainMode)
// {
//     State_T * p_nextState = NULL;

//     if (!StateMachine_IsLeafState(p_context->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK)) { p_nextState = NULL; }
//     else if (!Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_STOP)) { p_nextState = NULL; }
//     /* Allow temporary mapping to experimental state */
//     else { p_nextState = GetMainState(p_context); } // change to selected state

//     return p_nextState;
// }

static const State_Input_T LOCK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK]           = (State_Input_T)Lock_InputLockOp_Blocking,
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
    // Motor_Table_EnterCalibrateAdc(&p_context->MOTORS); /* Motor handles it own state */
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
        // if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false)
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

//     if (Motor_Table_IsEveryState(&p_context->MOTORS, MSM_STATE_ID_CALIBRATION) == false)
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

    Motor_Table_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */
// #if defined(MOTOR_CONTROLLER_DEBUG_ENABLE)
//     memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
// #endif
    MotorController_BeepPeriodic(p_context);
}

static void Fault_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_MC_STATE;

    Motor_Table_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */

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
void MotorController_EnterFault(const MotorController_T * p_context)
{
    if (MotorController_IsFault(p_context) == false) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, -1); }
}

bool MotorController_ExitFault(const MotorController_T * p_context)
{
    if (MotorController_IsFault(p_context) == true) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, 0); }
    return !MotorController_IsFault(p_context);
}

// ((const MotorController_FaultFlags_T) { .VAccsLimit = 1U }).Value
void MotorController_SetFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    if (MotorController_IsFault(p_context) == false) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, faultFlags); }
}

/* ensure repeat inputs without lock do not mismatch */
void MotorController_ClearFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    if (MotorController_IsFault(p_context) == true) { StateMachine_InputAsyncTransition(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, ~faultFlags); }
    // return !MotorController_IsFault(p_context); /* alternatively use cleared diff */
}



