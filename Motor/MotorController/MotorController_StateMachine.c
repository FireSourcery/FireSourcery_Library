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
#include "Utility/StateMachine/StateMachine.h"
#include "System/SysTime/SysTime.h"
#include <string.h>

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_PARK;
static const StateMachine_State_T STATE_DRIVE;
static const StateMachine_State_T STATE_NEUTRAL;
static const StateMachine_State_T STATE_BLOCKING;
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
    .P_STATE_INITIAL         = &STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MCSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(MotorControllerPtr_T p_mc, statemachine_inputvalue_t voidVar)          { (void)p_mc; (void)voidVar; return &STATE_FAULT; }
static StateMachine_State_T * TransitionBlocking(MotorControllerPtr_T p_mc, statemachine_inputvalue_t blockingId)    { (void)p_mc; return (blockingId == MOTOR_CONTROLLER_BLOCKING_ENTER) ? &STATE_BLOCKING : 0U; }

/******************************************************************************/
/*!
    @brief Init State

    Init State does not transistion to fault, wait for ADC
*/
/******************************************************************************/
static void Init_Entry(MotorControllerPtr_T p_mc) { (void)p_mc; }

static void Init_Exit(MotorControllerPtr_T p_mc)
{
    p_mc->FaultFlags.Word = 0U; /* Clear initial ADC readings */
    MotorController_BeepShort(p_mc);
}

static void Init_Proc(MotorControllerPtr_T p_mc)
{
    if(SysTime_GetMillis() > GLOBAL_MOTOR.INIT_WAIT  )
    {
        //verify params
        if(p_mc->InitFlags.Word == 0U)   //indirectly poll inputs
        {
            if(p_mc->Parameters.InitMode == MOTOR_CONTROLLER_INIT_MODE_SERVO) { _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_SERVO); }
            else { _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_PARK); }
        }
    }
}
// static StateMachine_State_T * Init_InputThrottle(MotorControllerPtr_T p_mc)
// {
//     if((p_mc->Parameters.BuzzerFlagsEnable.ThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.ThrottleOnInit == 0U))
//     {
//         p_mc->BuzzerFlagsActive.ThrottleOnInit = 1U;
//         MotorController_BeepShort(p_mc);
//     }
//     return 0U;
// }

static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT] = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_INIT =
{
    .ID                 = MCSM_STATE_ID_INIT,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Init_Entry,
    .EXIT               = (StateMachine_Function_T)Init_Exit,
    .LOOP               = (StateMachine_Function_T)Init_Proc,
};

/******************************************************************************/
/*!
    @brief
    Motors in Stop State.
    May enter from Neutral State or Drive State
*/
/******************************************************************************/
static void Park_Entry(MotorControllerPtr_T p_mc)
{
    MotorController_TryHoldAll(p_mc);
    p_mc->StatusFlags.IsStopped = 1U;
    // p_mc->DriveDirection = MOTOR_CONTROLLER_DIRECTION_PARK;
}

static void Park_Proc(MotorControllerPtr_T p_mc) { (void)p_mc; }

static StateMachine_State_T * Park_InputBlocking(MotorControllerPtr_T p_mc, statemachine_inputvalue_t blockingId)
{
    return (blockingId == MOTOR_CONTROLLER_BLOCKING_ENTER) ? &STATE_BLOCKING : 0U;
}

static StateMachine_State_T * Park_InputDirection(MotorControllerPtr_T p_mc, statemachine_inputvalue_t direction)
{
    StateMachine_State_T * volatile p_nextState = 0U;
    p_mc->UserCmdValue = 0U; // non polling modes enter on brake
    switch((MotorController_Direction_T)direction)
    {
        case MOTOR_CONTROLLER_DIRECTION_PARK: p_nextState = 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL: p_nextState = &STATE_NEUTRAL; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD: p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE: p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        default: break;
    }
    if(p_nextState == 0U) { MotorController_BeepDouble(p_mc); }

    return p_nextState;
}

#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static StateMachine_State_T * Park_InputServo(MotorControllerPtr_T p_mc, statemachine_inputvalue_t voidVar)
{
    (void)voidVar;
    return &STATE_SERVO;
}
#endif

static const StateMachine_Transition_T PARK_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_BLOCKING]   = (StateMachine_Transition_T)Park_InputBlocking,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Park_InputDirection,
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
    [MCSM_INPUT_SERVO]      = (StateMachine_Transition_T)Park_InputServo,
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
    @brief SetBrake Common
*/
/******************************************************************************/
static void DriveNeutral_SetBrake(MotorControllerPtr_T p_mc)
{
    // if(p_mc->DriveState == MOTOR_CONTROLLER_DRIVE_BRAKE) { MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue); }
    // Brake has been released, and reapplied. Drive case need check for 0 speed for transition to park
    if((p_mc->DriveState == MOTOR_CONTROLLER_DRIVE_ZERO) && (MotorController_CheckStopAll(p_mc) == true)) //check for 0 speed, motor run does nto transition
    {
        MotorController_TryHoldAll(p_mc);
        p_mc->StatusFlags.IsStopped = 1U;
    }
    else
    {
        if(p_mc->DriveState != MOTOR_CONTROLLER_DRIVE_BRAKE) { MotorController_SetBrakeMode(p_mc); }
        MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue);
    }


    switch(p_mc->DriveState)
    {
        case MOTOR_CONTROLLER_DRIVE_BRAKE:
            MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue);
            break;

        case MOTOR_CONTROLLER_DRIVE_THROTTLE:
            MotorController_SetBrakeMode(p_mc);
            MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue);
            break;

        case MOTOR_CONTROLLER_DRIVE_ZERO:

            break;
        default: break;
    }
}

/******************************************************************************/
/*!
    @brief Drive State
    Analogous to in Fwd/Rev,
    Motor States: Run, Freewheel, Stop
    Accepted Inputs: Throttle, Brake


    neutral and drive share stop via motor stop state.
*/
/******************************************************************************/
static void Drive_Entry(MotorControllerPtr_T p_mc)
{
    MotorController_ActivateAll(p_mc);
    p_mc->StatusFlags.IsStopped = 0U;
}

static void Drive_Proc(MotorControllerPtr_T p_mc)
{
    (void)p_mc;
}

static StateMachine_State_T * Drive_InputDirection(MotorControllerPtr_T p_mc, statemachine_inputvalue_t direction)
{
    StateMachine_State_T * p_nextState = 0U;
    switch((MotorController_Direction_T)direction)
    {
        //try hold
        case MOTOR_CONTROLLER_DIRECTION_PARK: p_nextState = MotorController_CheckStopAll(p_mc) ? &STATE_PARK : 0U; break; // release on low speed
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL: p_nextState = &STATE_NEUTRAL; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD: p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE: p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        default: break;
    }
    if(p_nextState == 0U) { MotorController_BeepDouble(p_mc); }
    return p_nextState;
}

/*! @param[in] driveCmd MotorController_DriveId_T */
static StateMachine_State_T * Drive_InputDrive(MotorControllerPtr_T p_mc, statemachine_inputvalue_t driveCmd)
{
    volatile MotorController_DriveId_T _driveCmd = (p_mc->UserCmdValue == 0) ? MOTOR_CONTROLLER_DRIVE_ZERO : (MotorController_DriveId_T)driveCmd;
    StateMachine_State_T * p_nextState = 0U;
    switch(_driveCmd)
    {
        case MOTOR_CONTROLLER_DRIVE_BRAKE: DriveNeutral_SetBrake(p_mc); break;
        case MOTOR_CONTROLLER_DRIVE_THROTTLE:
            if(p_mc->DriveState != MOTOR_CONTROLLER_DRIVE_THROTTLE) { MotorController_SetThrottleMode(p_mc); } //todo motor in stop state, activate
            MotorController_SetThrottleValue(p_mc, p_mc->UserCmdValue);
            break;
        case MOTOR_CONTROLLER_DRIVE_ZERO:
            if(p_mc->DriveState != MOTOR_CONTROLLER_DRIVE_ZERO) { MotorController_StartInputZero(p_mc); }
            MotorController_ProcInputZero(p_mc);
            break;
        default: break;
    }
    p_mc->DriveState = _driveCmd;
    return p_nextState;

}
/* todo activate default cmdMode */
static StateMachine_State_T * Drive_InputCmd(MotorControllerPtr_T p_mc, statemachine_inputvalue_t voidVar)
{
    return Drive_InputDrive(p_mc, MOTOR_CONTROLLER_DRIVE_THROTTLE);
}

static const StateMachine_Transition_T DRIVE_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Drive_InputDirection,
    [MCSM_INPUT_DRIVE]      = (StateMachine_Transition_T)Drive_InputDrive,
    [MCSM_INPUT_CMD]        = (StateMachine_Transition_T)Drive_InputCmd,
};

static const StateMachine_State_T STATE_DRIVE =
{
    .ID                 = MCSM_STATE_ID_DRIVE,
    .P_TRANSITION_TABLE = &DRIVE_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Drive_Entry,
    .LOOP               = (StateMachine_Function_T)Drive_Proc,
};

/******************************************************************************/
/*!
    @brief  Neutral State
    Motor States: Drive, Freewheel, Stop
        Motor maybe in Drive when Braking
        Transition between Freewheel and Stop on 0 Speed
        MCSM remains in Neutral state
    Accepted Inputs: Brake only, Throttle no effect.
    Motor Direction unchanged upon entering MC Neutral
*/
/******************************************************************************/
static void Neutral_Entry(MotorControllerPtr_T p_mc)
{
    // if(p_mc->DriveState != MOTOR_CONTROLLER_DRIVE_BRAKE) { MotorController_ReleaseAll(p_mc); }  /* If enter neutral while braking, handle discontinuity */
    MotorController_ReleaseAll(p_mc);
    // p_mc->DriveDirection == MOTOR_CONTROLLER_DIRECTION_NEUTRAL;
}
static void Neutral_Proc(MotorControllerPtr_T p_mc) { (void)p_mc; }

static StateMachine_State_T * Neutral_InputDirection(MotorControllerPtr_T p_mc, statemachine_inputvalue_t direction)
{
    StateMachine_State_T * p_nextState = 0U;
    switch((MotorController_Direction_T)direction)
    {
        case MOTOR_CONTROLLER_DIRECTION_PARK: p_nextState = MotorController_CheckStopAll(p_mc) ? &STATE_PARK : 0U; break; // release on low speed
        case MOTOR_CONTROLLER_DIRECTION_NEUTRAL: p_nextState = 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_FORWARD: p_nextState = MotorController_TryDirectionForwardAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        case MOTOR_CONTROLLER_DIRECTION_REVERSE: p_nextState = MotorController_TryDirectionReverseAll(p_mc, direction) ? &STATE_DRIVE : 0U; break;
        default: break;
    }
    if(p_nextState == 0U) { MotorController_BeepDouble(p_mc); }
    return p_nextState;
}

static StateMachine_State_T * Neutral_InputDrive(MotorControllerPtr_T p_mc, statemachine_inputvalue_t driveCmd)
{
    MotorController_DriveId_T _driveCmd = (p_mc->UserCmdValue == 0) ? MOTOR_CONTROLLER_DRIVE_ZERO : (MotorController_DriveId_T)driveCmd;
    StateMachine_State_T * p_nextState = 0U;
    switch(_driveCmd)
    {
        case MOTOR_CONTROLLER_DRIVE_BRAKE: DriveNeutral_SetBrake(p_mc); break;
        case MOTOR_CONTROLLER_DRIVE_ZERO: if(p_mc->DriveState != MOTOR_CONTROLLER_DRIVE_ZERO) { MotorController_ReleaseAll(p_mc); } break;
        case MOTOR_CONTROLLER_DRIVE_THROTTLE: break;
        default: break;
    }
    p_mc->DriveState = _driveCmd;
    return p_nextState;
}

static const StateMachine_Transition_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]      = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_DIRECTION]  = (StateMachine_Transition_T)Neutral_InputDirection,
    [MCSM_INPUT_DRIVE]      = (StateMachine_Transition_T)Neutral_InputDrive,
};

static const StateMachine_State_T STATE_NEUTRAL =
{
    .ID                 = MCSM_STATE_ID_NEUTRAL,
    .P_TRANSITION_TABLE = &NEUTRAL_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Neutral_Entry,
    .LOOP               = (StateMachine_Function_T)Neutral_Proc,
};


/******************************************************************************/
/*!
    @brief  Blocking Op
*/
/******************************************************************************/
static void Blocking_Entry(MotorControllerPtr_T p_mc) { (void)p_mc; }

static void Blocking_Proc(MotorControllerPtr_T p_mc) { (void)p_mc; }

static StateMachine_State_T * Blocking_InputBlocking_Blocking(MotorControllerPtr_T p_mc, statemachine_inputvalue_t blockingId)
{
    StateMachine_State_T * p_nextState = 0U;
    MotorPtr_T p_motor = MotorController_GetPtrMotor(p_mc, 0U); //toodo set all
    volatile int debug = 0;

    p_mc->BlockingState = blockingId;
    switch(blockingId)
    {
        case MOTOR_CONTROLLER_BLOCKING_ENTER:   break;
        case MOTOR_CONTROLLER_BLOCKING_EXIT:    p_nextState = &STATE_PARK;  break;
        case MOTOR_CONTROLLER_BLOCKING_NVM_SAVE_PARAMS:   p_mc->NvmStatus = MotorController_SaveParameters_Blocking(p_mc);    break;     /* Flash Write will disable interrupts */
        case MOTOR_CONTROLLER_BLOCKING_CALIBRATE_SENSOR:  Motor_User_CalibrateSensor(p_motor);                                break;
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
        // case MOTOR_CONTROLLER_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_SaveOnce_Blocking(p_mc);          break;
// #endif
        // case MOTOR_CONTROLLER_NVM_BOOT:          p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_mc);       break;
        // case MOTOR_CONTROLLER_SET_PARAMS_START:  p_mc->StatusFlags.SetParams = 1U;       break;
        // case MOTOR_CONTROLLER_SET_PARAMS_END:    p_mc->StatusFlags.SetParams = 0U;       break;
        default: break;
    }

    return p_nextState;
}

static const StateMachine_Transition_T BLOCKING_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_BLOCKING]       = (StateMachine_Transition_T)Blocking_InputBlocking_Blocking,
};

static const StateMachine_State_T STATE_BLOCKING =
{
    .ID                 = MCSM_STATE_ID_BLOCKING,
    .P_TRANSITION_TABLE = &BLOCKING_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Blocking_Entry,
    .LOOP               = (StateMachine_Function_T)Blocking_Proc,
};


/******************************************************************************/
/*!
    @brief Servo State
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static void Servo_Entry(MotorControllerPtr_T p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Start(p_mc);
#else
    MotorController_Servo_Start(p_mc);
    MotorController_SetCmdMode(p_mc, p_mc->Parameters.DefaultCmdMode);
    MotorController_ActivateAll(p_mc);
#endif
}

static void Servo_Proc(MotorControllerPtr_T p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Proc(p_mc);
#else
    MotorController_Servo_Proc(p_mc);
#endif
}

static StateMachine_State_T * Servo_InputExit(MotorControllerPtr_T p_mc, statemachine_inputvalue_t voidVar)
{
    (void)voidVar;
    MotorController_ReleaseAll(p_mc);
    return &STATE_PARK;
}

static StateMachine_State_T * Servo_InputCmd(MotorControllerPtr_T p_mc, statemachine_inputvalue_t userCmd)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_SetCmd(p_mc, cmd);
#else
    MotorController_Servo_SetCmd(p_mc, p_mc->UserCmdValue);
    MotorController_SetCmdModeValue(p_mc, p_mc->UserCmdValue);
#endif
    return 0U;
}

static StateMachine_State_T * Servo_InputServo(MotorControllerPtr_T p_mc, statemachine_inputvalue_t servoId)
{
    switch(servoId)
    {
        case 0:  break;

        default:   break;
    }
}

static const StateMachine_Transition_T SERVO_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_DIRECTION]      = (StateMachine_Transition_T)Servo_InputExit,
    [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)Servo_InputCmd,
    [MCSM_INPUT_SERVO]          = (StateMachine_Transition_T)Servo_InputServo,
    [MCSM_INPUT_BLOCKING]       = (StateMachine_Transition_T)TransitionBlocking,
};

static const StateMachine_State_T STATE_SERVO =
{
    .ID                 = MCSM_STATE_ID_SERVO,
    .P_TRANSITION_TABLE = &SERVO_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Servo_Entry,
    .LOOP               = (StateMachine_Function_T)Servo_Proc,
};
#endif


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(MotorControllerPtr_T p_mc)
{
    MotorController_DisableAll(p_mc);
#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
    memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
#endif
    Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U);
}

static void Fault_Proc(MotorControllerPtr_T p_mc)
{
    MotorController_DisableAll(p_mc);

    switch(p_mc->Parameters.InputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL: /* Protocol Rx Lost use auto recover, without user input */
            p_mc->FaultFlags.RxLost = Protocol_CheckRxLost(&p_mc->CONFIG.P_PROTOCOLS[0U]);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
        default:  break;
    }

    if(p_mc->FaultFlags.Word == 0U)
    {
        Blinky_Stop(&p_mc->Buzzer);
        _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_PARK);
    }
}

/* Fault State Input Fault Checks Fault */
/* Sensor faults only clear on user input */
static StateMachine_State_T * Fault_InputFault(MotorControllerPtr_T p_mc, statemachine_inputvalue_t voidVar)
{
    (void)voidVar;
    p_mc->FaultFlags.Motors         = (MotorController_ClearFaultAll(p_mc) == false);
    p_mc->FaultFlags.VSenseLimit    = VMonitor_GetIsFault(&p_mc->VMonitorSense);
    p_mc->FaultFlags.VAccsLimit     = VMonitor_GetIsFault(&p_mc->VMonitorAccs);
    p_mc->FaultFlags.VSourceLimit   = VMonitor_GetIsFault(&p_mc->VMonitorSource);
    p_mc->FaultFlags.PcbOverHeat    = Thermistor_GetIsFault(&p_mc->ThermistorPcb);
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    p_mc->FaultFlags.MosfetsTopOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsTop);
    p_mc->FaultFlags.MosfetsBotOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfetsBot);
#else
    p_mc->FaultFlags.MosfetsOverHeat = Thermistor_GetIsFault(&p_mc->ThermistorMosfets);
#endif

    p_mc->FaultFlags.User = 0U;
    return 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)Fault_InputFault,
    [MCSM_INPUT_BLOCKING]       = (StateMachine_Transition_T)TransitionBlocking,
    // [MCSM_INPUT_DIRECTION]      = (StateMachine_Transition_T)0U,
    // [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_FAULT =
{
    .ID                 = MCSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Fault_Entry,
    .LOOP               = (StateMachine_Function_T)Fault_Proc,
};

    // switch(p_mc->DriveState)
    // {
    //     case MOTOR_CONTROLLER_DRIVE_BRAKE:
    //         switch((MotorController_DriveId_T)driveCmd)
    //         {
    //             case MOTOR_CONTROLLER_DRIVE_BRAKE: MotorController_SetBrakeValue(p_mc, p_mc->UserCmdValue); break;
    //             case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_SetThrottleMode(p_mc); break;
    //             case MOTOR_CONTROLLER_DRIVE_ZERO: Drive_InputZero_Start(p_mc); break;
    //             default: break;
    //         }
    //     case MOTOR_CONTROLLER_DRIVE_THROTTLE:
    //         switch((MotorController_DriveId_T)driveCmd)
    //         {
    //             case MOTOR_CONTROLLER_DRIVE_BRAKE: MotorController_SetBrakeMode(p_mc); break;
    //             case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_SetThrottleValue(p_mc, p_mc->UserCmdValue); break;
    //             case MOTOR_CONTROLLER_DRIVE_ZERO: Drive_InputZero_Start(p_mc); break;
    //             default: break;
    //         }
    //         break;
    //     case MOTOR_CONTROLLER_DRIVE_ZERO:
    //         switch((MotorController_DriveId_T)driveCmd)
    //         {
    //             case MOTOR_CONTROLLER_DRIVE_BRAKE:
    //                 if(MotorController_CheckStopAll(p_mc) == true) { MotorController_TryHoldAll(p_mc); p_nextState = &STATE_PARK; }
    //                 else { MotorController_SetBrakeMode(p_mc); }
    //                 break;
    //             case MOTOR_CONTROLLER_DRIVE_THROTTLE: MotorController_SetThrottleMode(p_mc); break;
    //             case MOTOR_CONTROLLER_DRIVE_ZERO: Drive_InputZero_Proc(p_mc); break;
    //             default: break;
    //         }
    //         break;
    //     default: break;
    // }