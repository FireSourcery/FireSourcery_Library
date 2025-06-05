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

#include "MotDrive/MotDrive_StateMachine.h"

#include "Utility/StateMachine/StateMachine.h"

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Reboot.h)

static const State_T STATE_INIT;
static const State_T STATE_MAIN;
static const State_T STATE_LOCK;
static const State_T STATE_FAULT;


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
const StateMachine_Machine_T MCSM_MACHINE =
{
    .P_STATE_INITIAL = &STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MCSM_TRANSITION_TABLE_LENGTH,
};

static State_T * TransitionFault(const MotorController_T * p_context, state_input_value_t _void) { (void)_void; return &STATE_FAULT; }


/* Main thread only sets [FaultFlags]. call to check clear. via results of Monitor State */
void MotorController_ClearFaultFlags(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    p_mc->FaultFlags.VSourceLimit = RangeMonitor_IsAnyFault(p_context->V_SOURCE.P_STATE);
    p_mc->FaultFlags.VAccsLimit = RangeMonitor_IsAnyFault(p_context->V_ACCESSORIES.P_STATE);
    p_mc->FaultFlags.VAnalogLimit = RangeMonitor_IsAnyFault(p_context->V_ANALOG.P_STATE);
    p_mc->FaultFlags.PcbOverheat = (Monitor_GetStatus(p_context->HEAT_PCB.P_STATE) == MONITOR_STATUS_FAULT);
    p_mc->FaultFlags.MosfetsOverheat = (HeatMonitor_Group_GetStatus(&p_context->HEAT_MOSFETS) == MONITOR_STATUS_FAULT);
}

/******************************************************************************/
/*!
    @brief Init State

    Init State does not transistion to fault, wait for ADC
*/
/******************************************************************************/
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
    MotorController_State_T * p_mc = p_context->P_ACTIVE;


}

static State_T * Init_Next(const MotorController_T * p_context)
{
    State_T * p_nextState = NULL;

    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    /* Wait for initial ADC readings and clear fault flags set by sensor polling in Main thread */
    // wait for every motor exit init
    if (SysTime_GetMillis() > MOTOR_STATE_MACHINE_INIT_WAIT + 50U)
    {
        MotorController_ClearFaultFlags(p_context);
        if (MotorAnalogRef_IsLoaded() == false) { p_mc->FaultFlags.InitCheck = 1U; } /*  */

        MotorController_CaptureVSourceActiveRef(p_context); /* Set Motors VSupplyRef using ADC reading */

        if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == false) { p_mc->FaultFlags.Motors = 1U; }

        /* In the case of boot into motor spinning state. Go to fault state disable output */
        if (p_mc->FaultFlags.Value == 0U)
        {
            MotorController_BeepShort(p_context);
            p_nextState = &STATE_MAIN; // GetInitialState(p_context);
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
    [MCSM_INPUT_FAULT] = NULL, /* MotorController_StateMachine_EnterFault is disabled for INIT_STATE */
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
    @brief Main App State
*/
/******************************************************************************/
static void Main_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch (p_mc->Config.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
        // MotDrive_DisableVarInput(&p_context->MOT_DRIVE);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN:
            break;
        default:  break;
    }
}

// static void Main_Exit(const MotorController_T * p_context)
// {
//     MotorController_BeepShort(p_context);
// }

static void Main_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    // StateMachine_ProcState(&p_mc->MotDriveStateMachine); /* Proc Sub MAchine */
    // MotDrive_Proc_Thread(&p_context->MOT_DRIVE);
}

static State_T * Main_Next(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    State_T * p_nextState = NULL;
    return p_nextState;
}

static State_T * Main_InputLock(const MotorController_T * p_context, state_input_value_t lockId)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    State_T * p_nextState = NULL;

    if ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER)
    {
        //switch on app machine
        if (MotDrive_StateMachine_GetDirection(&p_context->MOT_DRIVE) == MOT_DRIVE_DIRECTION_PARK)
        {
            p_nextState = &STATE_LOCK;
        }
    }
    return p_nextState;
    // return ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER) ? &STATE_LOCK : NULL;
}

static const State_Input_T MAIN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT] = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK] = (State_Input_T)Main_InputLock,
};

static const State_T STATE_MAIN =
{
    .ID     = MCSM_STATE_ID_MAIN,
    .ENTRY  = (State_Action_T)Main_Entry,
    .LOOP   = (State_Action_T)Main_Proc,
    .NEXT   = (State_InputVoid_T)Main_Next,
    .P_TRANSITION_TABLE = &MAIN_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Cmd Passthrough State
*/
/******************************************************************************/
static void Cmd_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch (p_mc->Config.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
            // MotMotors_ForEach(&p_context->MOTORS, Motor_VarInput_Disable);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
            // MotMotors_ForEach(&p_context->MOTORS, Motor_VarInput_Enable);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN:
            break;
        default:  break;
    }
}

// static void Cmd_Exit(const MotorController_T * p_context)
// {
//     MotorController_BeepShort(p_context);
// }

static void Cmd_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

}

static State_T * Cmd_Next(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    State_T * p_nextState = NULL;
    return p_nextState;
}

// static State_T * Cmd_InputLock(MotorController_T * p_context, state_input_value_t lockId)
// {
//     MotorController_State_T * p_mc = p_context->P_ACTIVE;

//     // return ((MotorController_LockId_T)lockId == MOTOR_CONTROLLER_LOCK_ENTER) ? &STATE_LOCK : NULL;
// }

static State_T * Cmd_InputFeedbackMode(const MotDrive_T * p_motDrive, state_input_value_t feedbackMode)
{
    MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_SetFeedbackMode_Cast, feedbackMode);
    return NULL;
}


static State_T * Drive_InputCmd(const MotDrive_T * p_motDrive, state_input_value_t value)
{
    int16_t cmdValue = (int16_t)value;

    // switch (p_this->DriveSubState)
    // {
    //     case MOT_DRIVE_CMD : /* for a non polling input */
    //         if (cmdValue == 0) { MotDrive_StartDriveZero(p_this); p_this->DriveSubState = MOT_DRIVE_CMD_RELEASE; }
    //         break;
    //     case MOT_DRIVE_CMD_RELEASE:
    //         if (cmdValue != 0) { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ActivateControl); p_this->DriveSubState = MOT_DRIVE_CMD_CMD; }
    //         break;
    // }

    // if (cmdValue == 0) { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_Release); }
    //     // break;
    // if (cmdValue != 0) { MotMotors_ForEach(&p_motDrive->MOTORS, Motor_User_ActivateControl); }
    //     // break;

    MotMotors_ForEachSet(&p_motDrive->MOTORS, Motor_User_SetActiveCmdValue_ScalarCast, cmdValue);
    // p_this->UserCmdValue = cmdValue;
    return NULL;
}

static const State_Input_T CMD_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT] = (State_Input_T)TransitionFault,
    [MCSM_INPUT_CMD] = (State_Input_T)Drive_InputCmd,
    // [MCSM_INPUT_LOCK] = (State_Input_T)Cmd_InputLock,
    // [MCSM_INPUT_FEEDBACK_MODE] = (State_Input_T)Cmd_InputFeedbackMode,
};

static const State_T STATE_CMD =
{
    .ID = MCSM_STATE_ID_CMD,
    .ENTRY = (State_Action_T)Cmd_Entry,
    .LOOP = (State_Action_T)Cmd_Proc,
    .NEXT = (State_InputVoid_T)Cmd_Next,
    .P_TRANSITION_TABLE = &CMD_TRANSITION_TABLE[0U],
};



/******************************************************************************/
/*!
    @brief Blocking and alike functions
        True thread blocking functions, and extended async operations
        Nvm functions block.
        Calibration routines set status id upon completion.
*/
/******************************************************************************/
static void Lock_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_STOP) == false) { p_mc->FaultFlags.Motors = true; }
    // MotMotors_ForEach(&p_context->MOTORS, Motor_EnableVarAccess);
    MotMotors_ForEach(&p_context->MOTORS, Motor_Calibration_Enter);
    if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }

    p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER;
    p_mc->LockOpStatus = 0U;
}

static void Lock_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    if (MotMotors_IsEveryValue(&p_context->MOTORS, Motor_StateMachine_IsState, MSM_STATE_ID_CALIBRATION) == false) { p_mc->FaultFlags.Motors = true; }

    switch(p_mc->LockSubState)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER:
            // if (MotorController_IsEveryMotorStopState(p_context) == false) { p_mc->FaultFlags.Motors = true; } // poll motor
            break;
        case MOTOR_CONTROLLER_LOCK_EXIT:                break;
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     break;
        /* Motor Calibration State transistion may start next pwm cycle */
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:
            // if (ProcCalibrateSensor(p_context) == true)
            // {
            //     p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER;
            //     p_mc->LockOpStatus = 0U;
            // }
            // else if (MotorController_IsAnyMotorFault(p_context) == true) { p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER; p_mc->LockOpStatus = 1U; }
            break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:
            // if (MotorController_Analog_ProcCalibrate(p_context) == true)
            // {
            //     p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER;
            //     p_mc->LockOpStatus = 0U;
            // }
            break;
    }
}

static State_T * Lock_Next(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    if (p_mc->FaultFlags.Value != 0U) { return &STATE_FAULT; }
    return NULL;
}

/* Lock SubState by passed value */
static State_T * Lock_InputLockOp_Blocking(const MotorController_T * p_context, state_input_value_t lockId)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    State_T * p_nextState = NULL;

    p_mc->LockSubState = lockId;

    switch ((MotorController_LockId_T)lockId)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER: break;
        case MOTOR_CONTROLLER_LOCK_EXIT:
            MotMotors_ForEach(&p_context->MOTORS, Motor_Calibration_Exit);
            p_nextState = &STATE_MAIN;
            break;
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
            p_mc->NvmStatus = MotorController_SaveConfig_Blocking(p_context); /* NvM function will block + disable interrupts */
            p_mc->LockOpStatus = p_mc->NvmStatus;
            p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER;
            break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC: /* alternatively split */
            // MotorController_Analog_StartCalibrate(p_context);
            MotMotors_ForEach(&p_context->MOTORS, Motor_Analog_Calibrate); /* Motor handles it own state */
            // Motor_Analog_Calibrate(p_context);
            break;
            // case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:  StartCalibrateSensor(p_context);    break;
        case MOTOR_CONTROLLER_LOCK_REBOOT:  HAL_Reboot(); break; /* No return */ // optionally deinit clock select
        // case MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_WriteOnce_Blocking(p_context);         break;
        // case MOTOR_CONTROLLER_NVM_BOOT:                  p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_context);       break;
    }

    return p_nextState;
}

static const State_Input_T LOCK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]  = (State_Input_T)TransitionFault,
    [MCSM_INPUT_LOCK]   = (State_Input_T)Lock_InputLockOp_Blocking,
};

static const State_T STATE_LOCK =
{
    .ID                 = MCSM_STATE_ID_LOCK,
    .ENTRY              = (State_Action_T)Lock_Entry,
    .LOOP               = (State_Action_T)Lock_Proc,
    .NEXT               = (State_InputVoid_T)Lock_Next,
    .P_TRANSITION_TABLE = &LOCK_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    MotMotors_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */

// #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
//     memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
// #endif
    MotorController_BeepPeriodic(p_context);
}

static void Fault_Proc(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    // MotMotors_ForEach(&p_context->MOTORS, Motor_User_ForceDisableControl);
    MotMotors_ForceDisableControl(&p_context->MOTORS); /* Force disable control for all motors */

    switch (p_mc->Config.InputMode)
    {
        /* Protocol Rx Lost use auto recover, without user input */
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:    p_mc->FaultFlags.RxLost = MotorController_PollRxLost(p_context); break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN:       break;
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:    break;
    }

    if (p_mc->FaultFlags.Value == 0U)
    {
        Blinky_Stop(&p_context->BUZZER);
        _StateMachine_Transition(p_context->STATE_MACHINE.P_ACTIVE, p_context, &STATE_MAIN);
    }
}

/* Fault State Input Fault Checks Fault */
/* Sensor faults only clear on user input */
static State_T * Fault_InputClearFault(const MotorController_T * p_context, state_input_value_t faultFlags)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    // p_mc->FaultFlags.Value &= ~faultFlags;
    // p_mc->FaultFlags.Value = faultFlags;
    // p_mc->FaultFlags.Motors = 0U; /* updated by [MotorController_Main_Thread] */
    MotMotors_ForEvery(&p_context->MOTORS, Motor_StateMachine_ExitFault);
    // p_mc->FaultFlags.User = 0U;
    p_mc->FaultFlags.Value = 0U;
    MotorController_ClearFaultFlags(p_context);
    return NULL;
}

static State_T * Fault_InputLockSaveConfig_Blocking(const MotorController_T * p_context, state_input_value_t lockId)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    // p_mc->LockSubState = lockId;
    // p_mc->LockOpStatus = -1;
    switch ((MotorController_LockId_T)lockId)
    {
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:
            p_mc->NvmStatus = MotorController_SaveConfig_Blocking(p_context);
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
/* Fault Interface Functions */
/******************************************************************************/
bool MotorController_StateMachine_IsFault(const MotorController_State_T * p_active) { return (StateMachine_GetActiveStateId(&p_active->StateMachine) == MCSM_STATE_ID_FAULT); }

/* todo thread safe without lock */
void MotorController_StateMachine_EnterFault(const MotorController_T * p_context)
{
    if (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == false) { StateMachine_ProcInput(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, 0U); }
}

bool MotorController_StateMachine_ExitFault(const MotorController_T * p_context)
{
    if (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == true) { StateMachine_ProcInput(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, 0U); }
    return (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == false);
}

// ((const MotorController_FaultFlags_T) { .VAccsLimit = 1U }).Value
void MotorController_StateMachine_SetFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    // p_context->FaultFlags.Value |= faultFlags;
    if (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == false) { StateMachine_ProcInput(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, faultFlags); }
}

// alternatively
// (MotorController_FaultFlags_T){ .Value = }
/* todo faultFlags as cleared state */
bool MotorController_StateMachine_ClearFault(const MotorController_T * p_context, uint16_t faultFlags)
{
    if (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == true) { StateMachine_ProcInput(&p_context->STATE_MACHINE, MCSM_INPUT_FAULT, faultFlags); }
    return (MotorController_StateMachine_IsFault(p_context->P_ACTIVE) == false);
}



