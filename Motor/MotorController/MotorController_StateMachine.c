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

    Architecture decoupling requirements:
    Main State Machine does not know about App specific states/inputs

    Behavioral requirements:
    The user sets the direction selector (forward/reverse) while parked. The controller must accept and remember this selection.
    When the user subsequently releases the park brake / engages the throttle, the system enters drive in the selected direction.

    Implementation options:
    Buffer user inputs as 'cmd/input state'
        Direction selection is not an event that causes a state transition.
        It is a configuration parameter that the user sets independently of the operating state.

    Motor handles direction state.

    Inactive orthogonal region

    Multi Input to Multi States

    - A new state is justified by a different safety envelope, not a different purpose
    - One-shot automated procedures get a state; interactive parameter adjustment does not
    - Orthogonal concerns get orthogonal mechanisms
        e.g. what control structure is running vs. what input source is connected
    - Lock is for things the operator must commit to and acknowledge as risky
        (out of normal operation to do something special)
    - The operator use case should not conflate with the system's state
*/
/******************************************************************************/
/******************************************************************************/
/*

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
    .TRANSITION_TABLE_LENGTH = MC_TRANSITION_TABLE_LENGTH,
};

/******************************************************************************/
/*!
    @brief Common
*/
/******************************************************************************/

/* Clear Latching */
/* Main thread only sets [FaultFlags]. call to check clear. via results of Monitor State */
void MotorController_PollFaultFlags(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    p_mc->FaultFlags.VBusLimit = VBus_IsAnyFault(p_dev->P_VBUS);
    p_mc->FaultFlags.VAccsLimit = RangeMonitor_IsAnyFault(p_dev->V_ACCESSORIES.P_STATE);
    p_mc->FaultFlags.VAnalogLimit = RangeMonitor_IsAnyFault(p_dev->V_ANALOG.P_STATE);
    p_mc->FaultFlags.PcbOverheat = (Monitor_GetStatus(p_dev->HEAT_PCB.P_STATE) == MONITOR_STATUS_FAULT);
    p_mc->FaultFlags.MosfetsOverheat = (HeatMonitor_Group_GetStatus(&p_dev->HEAT_MOSFETS) == MONITOR_STATUS_FAULT);
}

/* Non-Fault states: apply set/clear, transition to Fault if any flags remain */
static State_T * TransitionFault(MotorController_T * p_dev, state_value_t faultCmd)
{
    p_dev->P_MC->FaultFlags.Value |= (MotorController_FaultCmd_T) { .Value = faultCmd }.FaultSet;
    return (p_dev->P_MC->FaultFlags.Value != 0U) ? &STATE_FAULT : NULL;
}


/*
    MotorController AppTable
*/
static State_T * EnterAppMain(MotorController_T * p_dev) { Motor_Table_EnableAll(&p_dev->MOTORS); return MotorController_App_EnterMain(p_dev); }
static State_T * AppParkState(MotorController_T * p_dev) { return(p_dev->P_MC->Config.IsParkStateEnabled ? &STATE_PARK : EnterAppMain(p_dev)); }


/******************************************************************************/
/*!
    @brief MotorController State Constraints
    Each MotorController State constrains the range of Motor States
    INIT            INIT or already DISABLED)
    PARK            DISABLED
    MAIN            PASSIVE / RUN / OPEN_LOOP / DISABLED?
    LOCK            DISABLED / CALIBRATION
    FAULT           FAULT
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

static void Init_Entry(MotorController_T * p_dev)
{
    SysTime_Millis = 0U; /* Reset SysTime in case of reboot */
    MotorController_BeepShort(p_dev);
}

// static void Init_Exit(MotorController_T * p_dev)
// {
//     MotorController_BeepShort(p_dev);
// }

static void Init_Proc(MotorController_T * p_dev)
{
    (void)p_dev;
}

static State_T * Init_Next(MotorController_T * p_dev)
{
    State_T * p_nextState = NULL;
    MotorController_State_T * p_mc = p_dev->P_MC;

    /* Wait for initial ADC readings */
    // wait for every motor exit init
    if (SysTime_GetMillis() > MC_STATE_MACHINE_INIT_WAIT)
    {
        MotorController_PollFaultFlags(p_dev); /* Clear latching fault flags set by sensor polling in Main thread */

        if (Phase_Calibration_IsValid() == false) { p_mc->FaultFlags.InitCheck = 1U; }
        if (VBus_Config_IsValid(&p_dev->P_VBUS->Config) == false) { p_mc->FaultFlags.InitCheck = 1U; p_mc->FaultFlags.VBusLimit = 1U; }
        /* Enforce VMonitor Enable */
        if (VBus_IsEnabled(p_dev->P_VBUS) == false) { p_mc->FaultFlags.InitCheck = 1U; p_mc->FaultFlags.VBusLimit = 1U; }

        if (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_DISABLED) == false) { p_mc->FaultFlags.Motors = 1U; }

        /* In the case of boot into motor spinning state. Go to fault state disable output */
        if (p_mc->FaultFlags.Value == 0U) { p_nextState = AppParkState(p_dev); } else { p_nextState = &STATE_FAULT; }
    }

    return p_nextState;
}

static const State_Input_T INIT_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT]  = NULL, /* MotorController_EnterFault is disabled for INIT_STATE */
};

static const State_T STATE_INIT =
{
    .ID                 = MC_STATE_ID_INIT,
    .ENTRY              = (State_Action_T)Init_Entry,
    .LOOP               = (State_Action_T)Init_Proc,
    .NEXT               = (State_Input0_T)Init_Next,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief [Park] Top-level safe state for stationary operation
    Motor State: Stop.

    May enter from Neutral State or Drive State
*/
/******************************************************************************/
static void Park_Entry(MotorController_T * p_dev)
{
    Motor_Table_ApplyControl(&p_dev->MOTORS, PHASE_VOUT_Z); /* */
    Motor_Table_DisableAll(&p_dev->MOTORS);
}

static void Park_Proc(MotorController_T * p_dev)
{
    (void)p_dev;
}

/*
    Park accepts state commands only. Motor-generic and app-specific inputs are
    buffered by the caller (via CmdInput / Traction.Input) and sink here (NULL table slots).
    Buffered values persist and are consumed when Main/sub-state is entered.
*/
static State_T * Park_InputStateCmd(MotorController_T * p_dev, state_value_t cmd)
{
    switch ((MotorController_StateCmd_T)cmd)
    {
        case MOTOR_CONTROLLER_STATE_CMD_PARK:       return &STATE_PARK;
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:  return &STATE_PARK;
        // case MOTOR_CONTROLLER_STATE_CMD_E_STOP:
        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN: return EnterAppMain(p_dev); /* App resolves initial sub-state, reads buffered direction */
        default: break;
    }
    return NULL;
}

static State_T * Park_InputLock(MotorController_T * p_dev, state_value_t lockId)
{
    State_T * p_nextState = NULL;
    if ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER)
    {
        if (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_DISABLED) == true) { p_nextState = &MC_STATE_LOCK; }
    }
    return p_nextState;
}

static const State_Input_T PARK_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MC_STATE_INPUT_LOCK]           = (State_Input_T)Park_InputLock,
    [MC_STATE_INPUT_STATE_CMD]      = (State_Input_T)Park_InputStateCmd,
    [MC_STATE_INPUT_MOTOR_CMD]      = NULL, /* Sink: motor-generic commands buffered by caller, not applied in Park */
    [MC_STATE_INPUT_APP_USER]       = NULL, /* Sink: app-specific commands buffered by caller, not applied in Park */
};

static const State_T STATE_PARK =
{
    .ID                 = MC_STATE_ID_PARK,
    .ENTRY              = (State_Action_T)Park_Entry,
    .LOOP               = (State_Action_T)Park_Proc,
    .P_TRANSITION_TABLE = &PARK_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @name Main App State
    Motor States:  PASSIVE, RUN, OPEN_LOOP
    Base State as Stop/Idle before transition
    A mounting point for app states. Main base passthrough only
*/
/******************************************************************************/
static void Main_Entry(MotorController_T * p_dev)
{
    Motor_Table_EnableAll(&p_dev->MOTORS); /* Start in passive, Start to exit Stop */
    Motor_Table_ApplyControl(&p_dev->MOTORS, PHASE_VOUT_0); /* */
}

/* Handle Motor cmd arbitration if needed */
/* App State common background proc */
static void Main_Proc(MotorController_T * p_dev) { (void)p_dev; }

/*
    Note: Motor_OpenLoop exits on VOut 0/Z
    Motor_Calibration needs Calibration_Exit
*/
/* Entry guard for part */
static State_T * Common_InputPark(MotorController_T * p_dev)
{
    State_T * p_nextState = NULL;
    // Motor_Table_DisableAll(&p_dev->MOTORS); /* simplifies caller side, when Motor set to Async transition */
    /* Guard applies for both Async and Sync Motor handling transitions */
    if (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_DISABLED)) { p_nextState = &STATE_PARK; }
    /* If caller buffers input. Caller includes knowedge of whether callee is in an accepting state. */
    else if (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_PASSIVE) && Motor_Table_IsEvery(&p_dev->MOTORS, Motor_IsSpeedZero)) { p_nextState = &STATE_PARK; } /* Applies stop on enter */
    else { MotorController_BeepShort(p_dev); }
    return p_nextState;
}

/* App State defaults transitions — propagated from sub-states via HSM */
static State_T * Main_InputStateCmd(MotorController_T * p_dev, state_value_t cmd)
{
    switch (cmd)
    {
        case MOTOR_CONTROLLER_STATE_CMD_PARK:           return Common_InputPark(p_dev); /* Motors in Stop first */
            // case MOTOR_CONTROLLER_STATE_CMD_E_STOP:   return NULL; /* Motors in Stop first */
        case MOTOR_CONTROLLER_STATE_CMD_STOP_MAIN:      /* Motor_Table_DisableAll(&p_dev->MOTORS); */    return NULL; /* return MAIN to  Exit sub-state, disable inputs */
        case MOTOR_CONTROLLER_STATE_CMD_START_MAIN:     return EnterAppMain(p_dev); /* Enter app sub-state from Main idle */
            // if rootState == MAIN
        default:     return NULL;
    }
}

static const State_Input_T MAIN_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MC_STATE_INPUT_STATE_CMD]      = (State_Input_T)Main_InputStateCmd,
    [MC_STATE_INPUT_MOTOR_CMD]      = NULL, /* potentially app wraps calls. */
    [MC_STATE_INPUT_APP_USER]       = NULL, /* App-specific: handled by sub-states only, sink at Main */
    // [MC_STATE_INPUT_LOCK]           = (State_Input_T)Lock_Input,
};

const State_T MC_STATE_MAIN =
{
    .ID     = MC_STATE_ID_MAIN,
    .ENTRY  = (State_Action_T)Main_Entry,
    .LOOP   = (State_Action_T)Main_Proc,
    .P_TRANSITION_TABLE = &MAIN_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Default Motor Command

    Motors passthrough coordinated default
    marker for accepting [Motor_VarId] interface inputs
    as top state until id scheme is determined.
*/
/******************************************************************************/
static void MotorCmd_Entry(MotorController_T * p_dev)
{
    Motor_Table_ApplyControl(&p_dev->MOTORS, PHASE_VOUT_0); /* Set PWM Output */
}

static void MotorCmd_Proc(MotorController_T * p_dev) { (void)p_dev; }

/* Passthrough from buffered CmdInput — same pattern as Main_InputMotorCmd */
static State_T * MotorCmd_Input(MotorController_T * p_dev, state_value_t cmd)
{
    (void)p_dev; (void)cmd;
    // Motor_Input_T * p_input = &p_dev->P_MC->CmdInput;
    // switch ((MotorController_MotorCmd_T)cmd)
    // {
    //     case MOTOR_CONTROLLER_USER_CMD_SETPOINT:    Motor_Table_SetCmdWith(&p_dev->MOTORS, Motor_SetActiveCmdScalar, p_input->CmdValue); break;
    //     case MOTOR_CONTROLLER_USER_CMD_PHASE:       Motor_Table_ApplyControl(&p_dev->MOTORS, p_input->PhaseOutput); break;
    //     case MOTOR_CONTROLLER_USER_CMD_FEEDBACK:    Motor_Table_ApplyFeedbackMode(&p_dev->MOTORS, p_input->FeedbackMode); break;
    //     case MOTOR_CONTROLLER_USER_CMD_DIRECTION:   Motor_Table_ApplyUserDirection(&p_dev->MOTORS, p_input->Direction); break;
    //     default: break;
    // }
    // return NULL;

    // per motor
    // Motor_ApplyUserDirection(Motor_Table_At(&p_dev->MOTORS, p_input->MotorId), p_input->Direction);
    // Motor_ApplyFeedbackMode(p_motor, Motor_FeedbackMode_Cast(varValue));
    // Motor_ApplyControlState(p_motor, (Phase_Output_T)varValue);
}

static const State_Input_T MOTOR_CMD_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT] = (State_Input_T)TransitionFault,
    [MC_STATE_INPUT_STATE_CMD] = (State_Input_T)Main_InputStateCmd,
    [MC_STATE_INPUT_MOTOR_CMD] = (State_Input_T)MotorCmd_Input,
};

const State_T MC_STATE_MAIN_MOTOR_CMD =
{
    .ID         = MC_STATE_ID_MOTOR_CMD,
    .ENTRY      = (State_Action_T)MotorCmd_Entry,
    .LOOP       = (State_Action_T)MotorCmd_Proc,
    .P_TRANSITION_TABLE = &MOTOR_CMD_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief
    Inherits from Main, transition from Lock only
    marker for interface write, optionally  route through state input
*/
/******************************************************************************/
static void MotorTuning_Entry(MotorController_T * p_dev)
{
    Motor_Table_ForEachApply(&p_dev->MOTORS, _Motor_ResetTuning); /* Enter Tuning State for all motors */
}

static void MotorTuning_Proc(MotorController_T * p_dev) { (void)p_dev; }

static State_T * MotorTuning_Input(MotorController_T * p_dev, state_value_t cmd)
{
    (void)p_dev; (void)cmd;
}

/* allow reset */
static State_T * MotorTuning_InputTuning(MotorController_T * p_dev, state_value_t lockId)
{
    switch ((MotorController_LockId_T)lockId)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER: return &MC_STATE_LOCK;
        case MOTOR_CONTROLLER_LOCK_MOTOR_TUNING_MODE: return &MC_STATE_MAIN_TUNING;
    }
    return NULL;
}
//todo other exits restore

static const State_Input_T TUNING_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_LOCK] = (State_Input_T)MotorTuning_InputTuning,
};

const State_T MC_STATE_MAIN_TUNING =
{
    .ID         = MC_STATE_ID_MAIN_TUNING,
    .P_TOP      = &MC_STATE_MAIN,
    .P_PARENT   = &MC_STATE_MAIN,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)MotorTuning_Entry,
    .LOOP       = (State_Action_T)MotorTuning_Proc,
    .P_TRANSITION_TABLE = &TUNING_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief [Lock] State - Blocking functions, and extended async operations
    Nvm functions
    Calibration routines. set status id upon completion.
*/
/******************************************************************************/
static const State_T MC_STATE_LOCK_CALIBRATE_ADC;

static void Lock_Entry(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    Motor_Table_DisableAll(&p_dev->MOTORS);
    Motor_Table_EnterCalibration(&p_dev->MOTORS); /* Enter Calibration State for all motors */

    p_mc->LockOpStatus = 0U;

    MotorController_BeepShort(p_dev);
}

static void Lock_Proc(MotorController_T * p_dev)
{
    (void)p_dev;
}

/* Lock SubState/Cmd by passed value */
/* alternatively StateMachine_TransitionCmd_T replace lockId */
static State_T * Lock_InputLockOp_Blocking(MotorController_T * p_dev, state_value_t lockId)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    State_T * p_nextState = NULL;
    MotorController_LockOpStatus_T opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR;

    /* From Top state only. no sub state active. */
    if (StateMachine_IsLeafState(p_dev->STATE_MACHINE.P_ACTIVE, &MC_STATE_LOCK)) /* Blcoks sub state inheritance  */
    {
        switch ((MotorController_LockId_T)lockId)
        {
            case MOTOR_CONTROLLER_LOCK_ENTER:
                opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                break;

            case MOTOR_CONTROLLER_LOCK_EXIT:
                if (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_CALIBRATION) || (Motor_Table_IsEveryState(&p_dev->MOTORS, MOTOR_STATE_ID_DISABLED)))
                {
                    Motor_Table_ForEachApply(&p_dev->MOTORS, Motor_Calibration_Exit);  /* exit calibration */
                    opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                    p_nextState = AppParkState(p_dev); /* Check AppParkState enable, optionally enter Park or Main */
                }
                else
                {
                    opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_ERROR;
                }
                break;

            /* todo check start from top state only substate == current state */
            case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
                p_mc->NvmStatus = MotNvm_SaveConfigAll_Blocking(&p_dev->MOT_NVM); /* NvM function will block + disable interrupts */

                Motor_Table_ForEachApply(&p_dev->MOTORS, Motor_Reinit); /* Reinit from config, which may have been updated by NvM save */
                opStatus = 0;
                break;

            case MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG:
                break;

            case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC: /* alternatively split */
                Motor_Table_EnterCalibrateAdc(&p_dev->MOTORS); /* Motor handles it own state */
                // opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_PROCESSING;
                opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                p_nextState = &MC_STATE_LOCK_CALIBRATE_ADC; /* Enter Calibration SubState */
                break;

                /* Generic select or call motor function */
            // case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:  StartCalibrateSensor(p_dev);    break;

            /* No return */
            case MOTOR_CONTROLLER_LOCK_REBOOT:
                HAL_Reboot();  // optionally deinit clock select
                break;

            case MOTOR_CONTROLLER_LOCK_MOTOR_CMD_MODE: /* keep available for pid tunning */
                Motor_Table_ForEachApply(&p_dev->MOTORS, Motor_Calibration_Exit);  /* exit calibration */
                opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                p_nextState = &MC_STATE_MAIN_MOTOR_CMD; /* */
                break;

            case MOTOR_CONTROLLER_LOCK_MOTOR_TUNING_MODE: /* keep available for pid tunning */
                Motor_Table_ForEachApply(&p_dev->MOTORS, Motor_Calibration_Exit);  /* exit calibration */
                opStatus = MOTOR_CONTROLLER_LOCK_OP_STATUS_OK;
                p_nextState = &MC_STATE_MAIN_TUNING; /* */
                break;

            // case MOTOR_CONTROLLER_LOCK_MOTOR_TUNING_MODE:       break;
            // case MOTOR_CONTROLLER_NVM_BOOT:                  p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_dev);       break;
            // case MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_WriteOnce_Blocking(p_dev);         break;

        }
    }

    p_mc->LockOpStatus = opStatus;
    return p_nextState;
}

// static State_T * Lock_InputStateCmd(MotorController_T * p_dev, state_value_t cmd)
// {
//     (void)p_dev; (void)cmd;
//     return NULL;
// }

static const State_Input_T LOCK_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT]          = (State_Input_T)TransitionFault,
    [MC_STATE_INPUT_LOCK]           = (State_Input_T)Lock_InputLockOp_Blocking,
    // [MC_STATE_INPUT_STATE_CMD]      = (State_Input_T)Lock_InputStateCmd,
};

const State_T MC_STATE_LOCK =
{
    .ID                 = MC_STATE_ID_LOCK,
    .ENTRY              = (State_Action_T)Lock_Entry,
    .LOOP               = (State_Action_T)Lock_Proc,
    .P_TRANSITION_TABLE = &LOCK_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief CalibrateAdc SubState
*/
/******************************************************************************/
void StartCalibrateAdc(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    p_mc->StateCounter = 0U;
    Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    Analog_Conversion_ClearResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
    Analog_Conversion_ClearResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    Accumulator_Init(&p_mc->AvgBuffer0);
    Accumulator_Init(&p_mc->AvgBuffer1);
    // Motor_Table_EnterCalibrateAdc(&p_dev->MOTORS); /* Motor handles it own state */
}

/* Proc Per ms */
void ProcCalibrateAdc(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    if (p_mc->StateCounter != 0U) /* skip first time */
    {
        Accumulator_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE));
        Accumulator_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE));
        Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE);
        Analog_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE);
    }

    p_mc->StateCounter++;
}

static State_T * EndCalibrateAdc(MotorController_T * p_dev)
{
    const uint32_t TIME = 2000U; /* > Motor calibrate adc time */

    MotorController_State_T * p_mc = p_dev->P_MC;
    State_T * p_nextState = NULL;

    if (p_mc->StateCounter > TIME)
    {
        MotAnalogUser_SetThrottleZero(&p_dev->ANALOG_USER, Accumulator_Avg(&p_mc->AvgBuffer0, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.THROTTLE)));
        MotAnalogUser_SetBrakeZero(&p_dev->ANALOG_USER, Accumulator_Avg(&p_mc->AvgBuffer1, Analog_Conversion_GetResult(&p_dev->ANALOG_USER_CONVERSIONS.BRAKE)));
        p_mc->LockOpStatus = 0; /* success */

        p_nextState = &MC_STATE_LOCK; /* return to lock state */
    }

    return p_nextState;
}

static const State_T MC_STATE_LOCK_CALIBRATE_ADC =
{
    .ID = MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC, // valid during subsstae only
    .P_TOP = &MC_STATE_LOCK,
    .P_PARENT = &MC_STATE_LOCK,
    .DEPTH = 1U,
    .ENTRY = (State_Action_T)StartCalibrateAdc,
    .LOOP = (State_Action_T)ProcCalibrateAdc,
    .NEXT = (State_Input0_T)EndCalibrateAdc,
};




/******************************************************************************/
/*!
    @brief Fault State
*/
/******************************************************************************/
static void Fault_Entry(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    Motor_Table_ForceDisableControl(&p_dev->MOTORS); /* Force disable control for all motors */
// #if defined(MOTOR_CONTROLLER_DEBUG_ENABLE)
//     memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
// #endif
    MotorController_BeepPeriodic(p_dev);
}

static void Fault_Proc(MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    Motor_Table_ForceDisableControl(&p_dev->MOTORS); /* Force disable control for all motors */

    // switch (p_mc->Config.InputMode)
    // {
    //     /* Protocol Rx Lost use auto recover, without user input */
    //     case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    p_mc->FaultFlags.RxLost = MotorController_PollRxLost(p_dev); break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_CAN:       break;
    //     case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    break;
    // }

    if (p_mc->FaultFlags.Value == 0U) { Blinky_Stop(&p_dev->BUZZER); }
}

/* Fault State: Set accumulates (latches), Clear side-effects then removes flags */
static State_T * Fault_InputFault(MotorController_T * p_dev, state_value_t faultCmd)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    MotorController_FaultCmd_T cmd = { .Value = faultCmd };

    if (cmd.FaultClear != 0U)
    {
        p_mc->FaultFlags.Value &= ~cmd.FaultClear;
        if (Phase_Calibration_IsValid() == false) { p_mc->FaultFlags.InitCheck = 1U; } /* or remove from clear */
        for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { Motor_StateMachine_TryClearFaultAll(&p_dev->MOTORS.P_DEVS[iMotor]); }
        MotorController_PollFaultFlags(p_dev); /* Re-verify conditions resolved before allowing exit */
        Blinky_Stop(&p_dev->BUZZER);
    }
    if (cmd.FaultSet != 0U)
    {
        p_mc->FaultFlags.Value |= cmd.FaultSet;
    }

    return (p_mc->FaultFlags.Value == 0U) ? AppParkState(p_dev) : NULL;
}

static State_T * Fault_InputLockSaveConfig_Blocking(MotorController_T * p_dev, state_value_t lockId)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    switch ((MotorController_LockId_T)lockId)
    {
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
            p_mc->NvmStatus = MotNvm_SaveConfigAll_Blocking(&p_dev->MOT_NVM);
            p_mc->LockOpStatus = p_mc->NvmStatus;
            break;
        default: break;
    }

    return NULL;
}

static const State_Input_T FAULT_TRANSITION_TABLE[MC_TRANSITION_TABLE_LENGTH] =
{
    [MC_STATE_INPUT_FAULT]  = (State_Input_T)Fault_InputFault,
    [MC_STATE_INPUT_LOCK]   = (State_Input_T)Fault_InputLockSaveConfig_Blocking,
};

static const State_T STATE_FAULT =
{
    .ID                 = MC_STATE_ID_FAULT,
    .ENTRY              = (State_Action_T)Fault_Entry,
    .LOOP               = (State_Action_T)Fault_Proc,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*
    Fault Interface Functions
*/
/******************************************************************************/
/* Pass only delta flags — handler applies OR FaultSet / AND-NOT FaultClear atomically */
// void MotorController_SetFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags)
// {
//     StateMachine_Tree_InputAsyncTransition(&p_dev->STATE_MACHINE, MC_STATE_INPUT_FAULT, (MotorController_FaultCmd_T) { .FaultSet = faultFlags.Value }.Value);
// }

// void MotorController_ClearFault(MotorController_T * p_dev, MotorController_FaultFlags_T faultFlags)
// {
//     StateMachine_Tree_InputAsyncTransition(&p_dev->STATE_MACHINE, MC_STATE_INPUT_FAULT, (MotorController_FaultCmd_T) { .FaultClear = faultFlags.Value }.Value);
// }

// bool MotorController_TryClearFaultAll(MotorController_T * p_dev)
// {
//     MotorController_ClearFault(p_dev, (MotorController_FaultFlags_T){ .Value = UINT16_MAX });
//     return !MotorController_IsFault(p_dev);
// }


