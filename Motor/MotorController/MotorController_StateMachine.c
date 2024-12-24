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
            States for input mode, User perspective
            Input acceptance using MotorController input and Motor_StateMachine
    @version V0
*/
/******************************************************************************/
#include "MotorController_StateMachine.h"
#include "MotorController_Analog.h"
#include "Utility/StateMachine/StateMachine.h"
#include "System/SysTime/SysTime.h"
#include "System/Reboot/Reboot.h"
#include <string.h>

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_PARK;
static const StateMachine_State_T STATE_DRIVE;
static const StateMachine_State_T STATE_NEUTRAL;
static const StateMachine_State_T STATE_LOCK;
static const StateMachine_State_T STATE_FAULT;
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static const StateMachine_State_T STATE_SERVO;
#endif

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

static StateMachine_State_T * TransitionFault(MotorController_T * p_mc, statemachine_input_value_t _void) { (void)_void; return &STATE_FAULT; }

#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static StateMachine_State_T * TransitionServo(MotorController_T * p_mc, statemachine_input_value_t servoMode)
{
    StateMachine_State_T * p_nextState = NULL;
    switch ((MotorController_ServoMode_T)servoMode)
    {
        case MOTOR_CONTROLLER_SERVO_MODE_ENTER:  p_nextState = &STATE_SERVO;  break;
        default: break;
    }
    return p_nextState;
}
#endif

/* Main thread only sets fault flags, does not clear, call to check clear */
void MotorController_PollAdcFaultFlags(MotorController_T * p_mc)
{
    p_mc->FaultFlags.VSenseLimit = VMonitor_IsFault(&p_mc->VMonitorSense);
    p_mc->FaultFlags.VAccsLimit = VMonitor_IsFault(&p_mc->VMonitorAccs);
    p_mc->FaultFlags.VSourceLimit = VMonitor_IsFault(&p_mc->VMonitorSource);
    p_mc->FaultFlags.PcbOverheat = Thermistor_IsFault(&p_mc->ThermistorPcb);

    for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
    {
        p_mc->FaultFlags.MosfetsOverheat = Thermistor_IsFault(&p_mc->MosfetsThermistors[iMosfets]);
        if (p_mc->FaultFlags.MosfetsOverheat == true) { break; }
    }
}


/******************************************************************************/
/*!
    @brief Init State

    Init State does not transistion to fault, wait for ADC
*/
/******************************************************************************/
static void Init_Entry(MotorController_T * p_mc)
{
    (void)p_mc;
}

// static void Init_Exit(MotorController_T * p_mc)
// {
//     MotorController_BeepShort(p_mc);
// }

static void Init_Proc(MotorController_T * p_mc)
{
    bool wait = true;

    /* Wait for initial ADC readings and clear fault flags set by sensor polling in Main thread */
    if (SysTime_GetMillis() > MOTOR_STATIC.INIT_WAIT + 5U)
    {
        wait = false;
        MotorController_PollAdcFaultFlags(p_mc);
        // MotorController_ResetVSourceActiveRef(p_mc); /* Set Motors VSourceRef using ADC reading */
    }

    if (wait == false)
    {
        if ((p_mc->FaultFlags.Value != 0U) || MotorController_IsAnyMotorFault(p_mc) == true)
        {
            _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_FAULT);
        }
        else
        {
            MotorController_BeepShort(p_mc);
            if (p_mc->Config.InitMode == MOTOR_CONTROLLER_MAIN_MODE_SERVO)
            {
                _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_SERVO);
            }
            else
            {
                /* In the case of boot into motor spinning state. Do not apply sudden hold of Park state. */
                if (MotorController_IsEveryMotorStopState(p_mc) == true)    { _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_PARK); }
                else                                                        { _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_NEUTRAL); }
            }
        }
    }
}

static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT] = NULL, /* MotorController_StateMachine_EnterFault is disabled for INIT_STATE */
};

static const StateMachine_State_T STATE_INIT =
{
    .ID                 = MCSM_STATE_ID_INIT,
    .ENTRY              = (StateMachine_Function_T)Init_Entry,
    .LOOP               = (StateMachine_Function_T)Init_Proc,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
    // .P_TRANSITION_TABLE = (StateMachine_Transition_T[MCSM_TRANSITION_TABLE_LENGTH])
    // {
    //     [MCSM_INPUT_FAULT] = (StateMachine_Transition_T)NULL,
    // },
};

/******************************************************************************/
/*!
    @brief
    Motors in Stop State.
    May enter from Neutral State or Drive State
*/
/******************************************************************************/
static void Park_Entry(MotorController_T * p_mc)
{
    if (MotorController_IsEveryMotorStopState(p_mc) == true) { MotorController_TryHoldAll(p_mc); }
    // p_mc->StateFlags.IsStopped = 1U;
    // p_mc->DriveDirection = MOTOR_CONTROLLER_DIRECTION_PARK;
}

static void Park_Proc(MotorController_T * p_mc) { (void)p_mc; }

static StateMachine_State_T * Park_InputBlocking(MotorController_T * p_mc, statemachine_input_value_t blockingId)
{
    return (blockingId == MOTOR_CONTROLLER_LOCK_ENTER) ? &STATE_LOCK : NULL;
}

static StateMachine_State_T * Park_InputDirection(MotorController_T * p_mc, statemachine_input_value_t direction)
{
    StateMachine_State_T * p_nextState = NULL;
    switch((MotorController_Direction_T)direction)
    {
        case MOTOR_CONTROLLER_DIRECTION_PARK:       p_nextState = NULL; break;
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD:    p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE:    p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
        default: break;
    }
    return p_nextState;
}

static const StateMachine_Transition_T PARK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_LOCK]       = (StateMachine_Transition_T)Park_InputBlocking,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Park_InputDirection,
    // [MCSM_INPUT_USER_MODE]  = (StateMachine_Transition_T)Park_InputUser,
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
    [MCSM_INPUT_SERVO] = (StateMachine_Transition_T)TransitionServo,
#endif
};

static const StateMachine_State_T STATE_PARK =
{
    .ID                 = MCSM_STATE_ID_PARK,
    .P_TRANSITION_TABLE = &PARK_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Park_Entry,
    .LOOP               = (StateMachine_Function_T)Park_Proc,
};


/******************************************************************************/
/*!
    @brief Set Common
*/
/******************************************************************************/
static void SetBrake(MotorController_T * p_mc, uint32_t cmdValue)
{
    if (p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_BRAKE)
    {
        MotorController_SetBrakeValue(p_mc, cmdValue);
    }
    else /* overwrite throttle */
    {
        // Brake has been released, and reapplied. Drive case need check for 0 speed for transition to park
        if (p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_ZERO)
        {
            if (MotorController_IsEveryMotorStopState(p_mc) == true) { MotorController_TryHoldAll(p_mc); }
            // p_mc->StateFlags.IsStopped = 1U;
        }

        MotorController_StartBrakeMode(p_mc);
        MotorController_SetBrakeValue(p_mc, cmdValue);
        p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_BRAKE;
        // if (MotorController_IsEveryMotorRunState(p_mc) == true) { p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_BRAKE; }
        // else { p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_ZERO; }
    }
}

static void SetThrottle(MotorController_T * p_mc, uint32_t cmdValue)
{
    if (p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_THROTTLE)
    {
        MotorController_SetThrottleValue(p_mc, cmdValue);
    }
    else if (p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_ZERO || p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_CMD) /* do not overwrite brake */
    {
        p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_THROTTLE;
        MotorController_StartThrottleMode(p_mc);
        MotorController_SetThrottleValue(p_mc, cmdValue);
    }
}

/* (cmdValue == 0U) do nothing. case where 0 cmd simultaneous with throttle/brake is permited without error */
/* Non-zero cmd simultaneous with throttle/brake results in error */
static void SetDriveZero(MotorController_T * p_mc, MotorController_DriveId_T driveSubState)
{
    if (p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_ZERO)
    {
        MotorController_ProcDriveZero(p_mc);
    }
    else if (p_mc->DriveSubState == driveSubState) /* Only override is prev input mode matches. Do not overwrite other mode on 0 */
    {
        p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_ZERO;
        MotorController_StartDriveZero(p_mc);
    }
}

/******************************************************************************/
/*!
    @brief Drive State
    SubStates: Fwd/Rev,
    Motor States: Run, Freewheel, Stop
    Accepted Inputs: Throttle, Brake

    neutral and drive share stop via motor stop state.
    DriveZero must release control, to transition into Park State
*/
/******************************************************************************/
static void Drive_Entry(MotorController_T * p_mc)
{
    // MotorController_ActivateAll(p_mc);
    // p_mc->StateFlags.IsStopped = 0U;
    p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_ZERO;
}

static void Drive_Proc(MotorController_T * p_mc)
{
    (void)p_mc;
}

static StateMachine_State_T * Drive_InputDirection(MotorController_T * p_mc, statemachine_input_value_t direction)
{
    StateMachine_State_T * p_nextState = NULL;
    switch((MotorController_Direction_T)direction)
    {
        case MOTOR_CONTROLLER_DIRECTION_PARK:       p_nextState = MotorController_IsEveryMotorStopState(p_mc) ? &STATE_PARK : NULL; break;
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD:    p_nextState = NULL; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE:    p_nextState = NULL; break;
        // case MOTOR_CONTROLLER_DIRECTION_FORWARD: p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
        // case MOTOR_CONTROLLER_DIRECTION_REVERSE: p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
        default: break;
    }

    return p_nextState;
}

/* either set drive 0 first, Motor set control twice. or motor directly transition */
static StateMachine_State_T * Drive_InputThrottle(MotorController_T * p_mc, statemachine_input_value_t cmdValue)
{
    if (cmdValue == 0U) { SetDriveZero(p_mc, MOTOR_CONTROLLER_DRIVE_THROTTLE); } else { SetThrottle(p_mc, cmdValue); }
    return NULL;
}

static StateMachine_State_T * Drive_InputBrake(MotorController_T * p_mc, statemachine_input_value_t cmdValue)
{
    if (cmdValue == 0U) { SetDriveZero(p_mc, MOTOR_CONTROLLER_DRIVE_BRAKE); } else { SetBrake(p_mc, cmdValue); }
    return NULL;
}

static const StateMachine_Transition_T DRIVE_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Drive_InputDirection,
    [MCSM_INPUT_THROTTLE]   = (StateMachine_Transition_T)Drive_InputThrottle,
    [MCSM_INPUT_BRAKE]      = (StateMachine_Transition_T)Drive_InputBrake,
    // [MCSM_INPUT_CMD]        = (StateMachine_Transition_T)Drive_InputCmd,
};

static const StateMachine_State_T STATE_DRIVE =
{
    .ID                 = MCSM_STATE_ID_DRIVE,
    .ENTRY              = (StateMachine_Function_T)Drive_Entry,
    .LOOP               = (StateMachine_Function_T)Drive_Proc,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/*!
    @brief  Neutral State
    Drive extension
    Motor States: Drive, Freewheel, Stop
        Motor maybe in Drive when Braking
        Transition between Freewheel and Stop on 0 Speed
        MCSM remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(MotorController_T * p_mc)
{
    if(p_mc->DriveSubState != MOTOR_CONTROLLER_DRIVE_BRAKE) { MotorController_TryReleaseAll(p_mc); }  /* If enter neutral while braking, handle discontinuity */
    // MotorController_TryReleaseAll(p_mc);
    // p_mc->DriveDirection = MOTOR_CONTROLLER_DIRECTION_NEUTRAL;
}

static void Neutral_Proc(MotorController_T * p_mc)
{
    (void)p_mc;
}

static StateMachine_State_T * Neutral_InputDirection(MotorController_T * p_mc, statemachine_input_value_t direction)
{
    StateMachine_State_T * p_nextState = NULL;
    switch((MotorController_Direction_T)direction)
    {
        case MOTOR_CONTROLLER_DIRECTION_PARK:       p_nextState = MotorController_IsEveryMotorStopState(p_mc) ? &STATE_PARK : 0U; break; // release on low speed
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:    p_nextState = NULL; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD:    p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE:    p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        default: break;
    }

    return p_nextState;
}

static StateMachine_State_T * Neutral_InputBrake(MotorController_T * p_mc, statemachine_input_value_t cmdValue)
{
    if(cmdValue == 0U)
    {
        if(p_mc->DriveSubState != MOTOR_CONTROLLER_DRIVE_ZERO)
        {
            p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_ZERO;
            MotorController_TryReleaseAll(p_mc);
        }
    }
    else
    {
        SetBrake(p_mc, cmdValue);
    }

    return NULL;
}

static const StateMachine_Transition_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Neutral_InputDirection,
    [MCSM_INPUT_BRAKE]      = (StateMachine_Transition_T)Neutral_InputBrake,
    // [MCSM_INPUT_DRIVE]      = (StateMachine_Transition_T)Neutral_InputDrive,
};

static const StateMachine_State_T STATE_NEUTRAL =
{
    .ID                 = MCSM_STATE_ID_NEUTRAL,
    .ENTRY              = (StateMachine_Function_T)Neutral_Entry,
    .LOOP               = (StateMachine_Function_T)Neutral_Proc,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Blocking and alike functions
        True thread blocking functions, and extended async operations
        Nvm functions block.
        Calibration routines set status id upon completion.
*/
/******************************************************************************/
static void Lock_Entry(MotorController_T * p_mc)
{
    p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER;
}

static void Lock_Proc(MotorController_T * p_mc)
{
    switch(p_mc->LockSubState)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER:               break;
        case MOTOR_CONTROLLER_LOCK_EXIT:                break;
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:
            if (Motor_User_IsStopState(MotorController_GetPtrMotor(p_mc, 0U)) == true) { p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER; p_mc->CalibrationStatus = 0U; }
            break; // todo check calibration complete
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:
            if (Motor_User_IsStopState(MotorController_GetPtrMotor(p_mc, 0U)) == true) { p_mc->LockSubState = MOTOR_CONTROLLER_LOCK_ENTER; p_mc->CalibrationStatus = 0U; }
            break;
        // case MOTOR_CONTROLLER_BLOCKING_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_WriteOnce_Blocking(p_mc);           break;
        // case MOTOR_CONTROLLER_NVM_BOOT:                  p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_mc);       break;
        // case MOTOR_CONTROLLER_BLOCKING_END:            Protocol_Send  break; // todo send end response
        default: break;
    }
}

/* Lock SubState by passed value */
static StateMachine_State_T * Locking_InputBlocking_Blocking(MotorController_T * p_mc, statemachine_input_value_t blockingId)
{
    StateMachine_State_T * p_nextState = NULL;
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U); // todo set all

    p_mc->LockSubState = blockingId;
    p_mc->CalibrationStatus = -1;

    switch (blockingId)
    {
        case MOTOR_CONTROLLER_LOCK_ENTER: break;
        case MOTOR_CONTROLLER_LOCK_EXIT:                p_nextState = &STATE_PARK;                                        break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:    Motor_User_CalibrateSensor(p_motor);                              break;
        case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:       MotorController_CalibrateAdc(p_mc);                               break;
            /* NvM function will block + disable interrupts */
        case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     p_mc->NvmStatus = MotorController_SaveConfig_Blocking(p_mc);      break;
        case MOTOR_CONTROLLER_LOCK_REBOOT:              Reboot(); break;
        // case MOTOR_CONTROLLER_LOCK_ENTER_SERVO:         p_nextState = &STATE_SERVO; break;
        default: break;
    }

    return p_nextState;
}

static const StateMachine_Transition_T BLOCKING_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_LOCK]       = (StateMachine_Transition_T)Locking_InputBlocking_Blocking,
    [MCSM_INPUT_SERVO]      = (StateMachine_Transition_T)TransitionServo,
};

static const StateMachine_State_T STATE_LOCK =
{
    .ID                 = MCSM_STATE_ID_LOCK,
    .ENTRY              = (StateMachine_Function_T)Lock_Entry,
    .LOOP               = (StateMachine_Function_T)Lock_Proc,
    .P_TRANSITION_TABLE = &BLOCKING_TRANSITION_TABLE[0U],
};


/******************************************************************************/
/*!
    @brief Servo State
    User State, Motor_User interface
*/
/******************************************************************************/
/*
    Handle per motor functions
    multi parameter input into StateMachine. No transition
*/
/*! @param[in] cmdValue int16  */
// void MotorController_ServoState_InputCmd(MotorController_T * p_mc, uint8_t motorId, int16_t cmdValue)
// {
//     if (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO)
//     {
//     #if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
//         MotorController_ServoExtern_SetMotorCmd(p_mc, motorId, cmdValue);
//     #else
//         Motor_User_SetActiveCmdValue(MotorController_GetPtrMotor(p_mc, motorId), cmdValue);
//     #endif
//     }
// }


#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static void Servo_Entry(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Start(p_mc);
#else
    // MotorController_Servo_Start(p_mc);
    // MotorController_StartCmdMode(p_mc, p_mc->Config.DefaultCmdMode);
    // MotorController_ActivateAll(p_mc);
#endif
}

static void Servo_Proc(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Proc(p_mc);
#else
    // MotorController_Servo_Proc(p_mc);
#endif
}

static StateMachine_State_T * Servo_InputExit(MotorController_T * p_mc, statemachine_input_value_t _void)
{
    (void)_void;
    MotorController_TryReleaseAll(p_mc);
    return &STATE_PARK;
}

/*! @param[in] cmdValue int16  */
static StateMachine_State_T * Servo_InputCmd(MotorController_T * p_mc, statemachine_input_value_t cmdValue)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_SetCmd(p_mc, cmdValue);
#else
    // MotorController_Servo_SetCmd(p_mc, userCmd);
    MotorController_SetCmdValueAll(p_mc, cmdValue);
#endif
    //     // if(cmdValue != 0U)
    //     // {
    //     //     if(p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_CMD)
    //     //     {
    //     //         MotorController_SetCmdModeValue(p_mc, cmdValue);
    //     //     }
    //     //     else if(p_mc->DriveSubState == MOTOR_CONTROLLER_DRIVE_ZERO)
    //     //     {
    //     //         MotorController_StartCmdModeDefault(p_mc);
    //     //         MotorController_SetCmdModeValue(p_mc, cmdValue);
    //     //         p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_CMD;
    //     //     }
    //     // }
    //     // else
    //     // {
    //     //     p_mc->DriveSubState = MOTOR_CONTROLLER_DRIVE_ZERO;
    //     // }
    return NULL;
}

// static StateMachine_State_T * Servo_InputDirection(MotorController_T * p_mc, statemachine_input_value_t direction)
// {
//     StateMachine_State_T * p_nextState = NULL;
//     switch((MotorController_Direction_T)direction)
//     {
//         case MOTOR_CONTROLLER_DIRECTION_PARK:       p_nextState = MotorController_IsEveryMotorStopState(p_mc) ? &STATE_PARK : NULL; break;
//         case MOTOR_CONTROLLER_DIRECTION_NEUTRAL:    p_nextState = &STATE_NEUTRAL; break;
//         case MOTOR_CONTROLLER_DIRECTION_FORWARD:    p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
//         case MOTOR_CONTROLLER_DIRECTION_REVERSE:    p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : NULL; break;
//         default: break;
//     }
//     return p_nextState;
// }

static StateMachine_State_T * Servo_InputServo(MotorController_T * p_mc, statemachine_input_value_t servoMode)
{
    StateMachine_State_T * p_nextState = NULL;
    switch ((MotorController_ServoMode_T)servoMode)
    {
        case MOTOR_CONTROLLER_SERVO_MODE_EXIT:
            if (MotorController_IsEveryMotorStopState(p_mc) == true) { p_nextState = &STATE_PARK; }
            break;
        default: break;
    }
    return p_nextState;
}

static const StateMachine_Transition_T SERVO_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)TransitionFault,
    // [MCSM_INPUT_DIRECTION]   = (StateMachine_Transition_T)Servo_InputDirection,
    [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)Servo_InputCmd,
    [MCSM_INPUT_SERVO]          = (StateMachine_Transition_T)Servo_InputServo,
};

static const StateMachine_State_T STATE_SERVO =
{
    .ID                 = MCSM_STATE_ID_SERVO,
    .ENTRY              = (StateMachine_Function_T)Servo_Entry,
    .LOOP               = (StateMachine_Function_T)Servo_Proc,
    .P_TRANSITION_TABLE = &SERVO_TRANSITION_TABLE[0U],
};
#endif

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(MotorController_T * p_mc)
{
    MotorController_ForceDisableAll(p_mc);
// #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
//     memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
// #endif
    Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U);
}

static void Fault_Proc(MotorController_T * p_mc)
{
    MotorController_ForceDisableAll(p_mc);

    switch(p_mc->Config.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_SERIAL: /* Protocol Rx Lost use auto recover, without user input */
            p_mc->FaultFlags.RxLost = Protocol_IsRxLost(&p_mc->CONST.P_PROTOCOLS[0U]);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
        case MOTOR_CONTROLLER_INPUT_MODE_ANALOG: break;
        default:  break;
    }

    if(p_mc->FaultFlags.Value == 0U)
    {
        Blinky_Stop(&p_mc->Buzzer);
        _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_PARK);
    }
}

/* Fault State Input Fault Checks Fault */
/* Sensor faults only clear on user input */
static StateMachine_State_T * Fault_InputClearFault(MotorController_T * p_mc, statemachine_input_value_t faultFlags)
{
    // p_mc->FaultFlags.Value &= ~faultFlags;
    p_mc->FaultFlags.Value = 0U;
    // p_mc->FaultFlags.Motors = 0U; /* updated by [MotorController_Main_Thread] */
    MotorController_ForEveryMotorExitFault(p_mc);
    // MotorController_PollAdcFaultFlags(p_mc);
    // p_mc->FaultFlags.User = 0U;
    return NULL;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)Fault_InputClearFault,
};

static const StateMachine_State_T STATE_FAULT =
{
    .ID                 = MCSM_STATE_ID_FAULT,
    .ENTRY              = (StateMachine_Function_T)Fault_Entry,
    .LOOP               = (StateMachine_Function_T)Fault_Proc,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
};

/******************************************************************************/
/* Fault Interface Functions */
/******************************************************************************/
bool MotorController_StateMachine_IsFault(const MotorController_T * p_mc) { return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_FAULT); }

void MotorController_StateMachine_EnterFault(MotorController_T * p_mc)
{
    if (MotorController_StateMachine_IsFault(p_mc) == false) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, 0U); }
}

bool MotorController_StateMachine_ExitFault(MotorController_T * p_mc)
{
    if (MotorController_StateMachine_IsFault(p_mc) == true) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, 0U); }
    return (MotorController_StateMachine_IsFault(p_mc) == false);
}

void MotorController_StateMachine_SetFault(MotorController_T * p_mc, uint16_t faultFlags)
{
    p_mc->FaultFlags.Value |= faultFlags;
    if (MotorController_StateMachine_IsFault(p_mc) == false) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, faultFlags); }
}

//alternatively
// (MotorController_FaultFlags_T){ .Value = }
bool MotorController_StateMachine_ClearFault(MotorController_T * p_mc, uint16_t faultFlags)
{
    if (MotorController_StateMachine_IsFault(p_mc) == true) { StateMachine_ProcInput(&p_mc->StateMachine, MCSM_INPUT_FAULT, faultFlags); }
    return (MotorController_StateMachine_IsFault(p_mc) == false);
}




/*! @param[in] driveCmd MotorController_DriveId_T */
// static StateMachine_State_T * _Drive_InputDrive(MotorController_T * p_mc, MotorController_DriveId_T driveCmdId, uint32 _driveCmdValue)
// {
//     MotorController_DriveId_T driveCmdSet = (p_mc->UserCmdValue == 0) ? MOTOR_CONTROLLER_DRIVE_ZERO : (MotorController_DriveId_T)driveCmd;
//     StateMachine_State_T * p_nextState = NULL;

//     //switch on input
//     // switch(driveCmdSet)
//     // {
//     //     case MOTOR_CONTROLLER_DRIVE_BRAKE: DriveNeutral_SetBrake(p_mc); break;

//     //     case MOTOR_CONTROLLER_DRIVE_THROTTLE:
//     //         if(p_mc->DriveSubState != MOTOR_CONTROLLER_DRIVE_THROTTLE) { MotorController_StartThrottleMode(p_mc); } //todo motor in stop state, activate
//     //         MotorController_SetThrottleValue(p_mc, p_mc->UserCmdValue);
//     //         break;

//     //     case MOTOR_CONTROLLER_DRIVE_ZERO:
//     //         if(p_mc->DriveSubState != MOTOR_CONTROLLER_DRIVE_ZERO) { MotorController_StartDriveZero(p_mc); }
//     //         MotorController_ProcDriveZero(p_mc);
//     //         break;
//     //     default: break;
//     // }

//     // switch on current state
//     switch(p_mc->DriveSubState)
//     {
//         case MOTOR_CONTROLLER_DRIVE_BRAKE:
//             switch((MotorController_DriveId_T)driveCmd)
//             {
//                 case MOTOR_CONTROLLER_DRIVE_BRAKE: MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue); break;
//                 case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_StartThrottleMode(p_mc); break;
//                 case MOTOR_CONTROLLER_DRIVE_ZERO: MotorController_StartDriveZero(p_mc); break;
//                 default: break;
//             }
//         case MOTOR_CONTROLLER_DRIVE_THROTTLE:
//             switch((MotorController_DriveId_T)driveCmd)
//             {
//                 case MOTOR_CONTROLLER_DRIVE_BRAKE: MotorController_StartBrakeMode(p_mc); break;
//                 case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_SetThrottleValue(p_mc, p_mc->UserCmdValue); break;
//                 case MOTOR_CONTROLLER_DRIVE_ZERO: Drive_InputZero_Start(p_mc); break;
//                 default: break;
//             }
//             break;
//         case MOTOR_CONTROLLER_DRIVE_ZERO:
//             switch((MotorController_DriveId_T)driveCmd)
//             {
//                 case MOTOR_CONTROLLER_DRIVE_BRAKE:
//                     if(MotorController_IsEveryMotorStopState(p_mc) == true) { MotorController_TryHoldAll(p_mc); p_nextState = &STATE_PARK; }
//                     else { MotorController_StartBrakeMode(p_mc); }
//                     break;
//                 case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_StartThrottleMode(p_mc); break;
//                 case MOTOR_CONTROLLER_DRIVE_ZERO: Drive_InputZero_Proc(p_mc); break;
//                 case MOTOR_CONTROLLER_DRIVE_CMD:
//                 default: break;
//             }
//             break;
//         default: break;
//     }

//     p_mc->DriveSubState = _driveCmd;

//     return p_nextState;
// }