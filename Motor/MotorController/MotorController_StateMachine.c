/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     MotorController_StateMachine.c
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
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_RUN;
static const StateMachine_State_T STATE_NEUTRAL;
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static const StateMachine_State_T STATE_SERVO;
#endif
static const StateMachine_State_T STATE_FAULT;

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

static StateMachine_State_T * TransitionFault(MotorController_T * p_mc, statemachine_inputext_t voidVar) { (void)p_mc; (void)voidVar; return &STATE_FAULT; }

/******************************************************************************/
/*!
    @brief Init State

    Init State does not transistion to fault, wait for ADC
*/
/******************************************************************************/
static void Init_Entry(MotorController_T * p_mc) { (void)p_mc; }

static void Init_Exit(MotorController_T * p_mc)
{
    p_mc->FaultFlags.State = 0U; /* Clear initial ADC readings */
    MotorController_BeepShort(p_mc);
}

static void Init_Proc(MotorController_T * p_mc)
{
//sample and set vsource    //set vsource if vsource less than max

    if(SysTime_GetMillis() > GLOBAL_MOTOR.INIT_WAIT) { _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_STOP); }
    /* todo And throttle, init conditionals */
}

// static StateMachine_State_T * Init_InputDirection(MotorController_T * p_mc)
// {
//     MotorController_ProcUserDirection(p_mc);
//     return 0U;
// }

// static StateMachine_State_T * Init_InputThrottle(MotorController_T * p_mc)
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
    // [MCSM_INPUT_SET_DIRECTION]         = (StateMachine_Transition_T)Init_InputDirection,
    // [MCSM_INPUT_PROC_ZERO]             = (StateMachine_Transition_T)TransitionStop, /* Zero throttle, Fwd/Rev. Change to neutral? */
    // [MCSM_INPUT_THROTTLE]             = (StateMachine_Transition_T)Init_InputThrottle,
};

static const StateMachine_State_T STATE_INIT =
{
    .ID                 = MCSM_STATE_ID_INIT,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Output_T)Init_Entry,
    .EXIT               = (StateMachine_Output_T)Init_Exit,
    .OUTPUT             = (StateMachine_Output_T)Init_Proc,
};

/******************************************************************************/
/*!
    @brief  Stop State
    Motors in Stop State. Enters upon all motors enter motor stop state
    May enter from Neutral State or Run State
*/
/******************************************************************************/
/*
    Entry upon detection of 0 speed, maybe upon active braking, or coast
*/
static void Stop_Entry(MotorController_T * p_mc)
{
    (void)p_mc;
    // if (p_mc->Parameters.StopMode == MOTOR_CONTROLLER_STOP_MODE_GROUND)
    // {
    //     MotorController_GroundAll(p_mc);
    // }
    // else
    // {
    //     MotorController_DisableAll(p_mc);
    // }
}

static void Stop_Proc(MotorController_T * p_mc) { (void)p_mc; }

static StateMachine_State_T * Stop_InputDirection(MotorController_T * p_mc, statemachine_inputext_t inputDirection)
{
    StateMachine_State_T * p_nextState;
    /*
        Motor Freewheel check stop should be atomic relative to this function
        this runs before motor freewheel checks speed => goto fault state.
    */
    if(MotorController_ProcUserDirection(p_mc, inputDirection) == true)
    {
        p_mc->ActiveDirection = inputDirection;
        p_nextState = (p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_NEUTRAL) ? &STATE_NEUTRAL : 0U;
    }
    else
    {
        p_mc->FaultFlags.DirectionSync = 1U;
        p_nextState = &STATE_FAULT;
    }

    return p_nextState;
}

/* */
static StateMachine_State_T * Stop_InputCmd(MotorController_T * p_mc, statemachine_inputext_t userCmd)
{
    // StateMachine_State_T * p_nextState = 0U;
    // p_mc->UserCmd = (int16_t)userCmd;
    // if((p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD) || (p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE))
    // {
    //     MotorController_SetCmd(p_mc, userCmd);
    //     p_nextState = &STATE_RUN;
    // }
    // return p_nextState;
}

static StateMachine_State_T * Stop_InputThrottle(MotorController_T * p_mc, statemachine_inputext_t userCmdThrottle)
{
    StateMachine_State_T * p_nextState = 0U;

    p_mc->UserCmd = (uint16_t)userCmdThrottle;
    if((p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD) || (p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE))
    {
        MotorController_SetThrottle(p_mc, userCmdThrottle); /* or non polling modes have to input throttle twice */
        p_nextState = &STATE_RUN;
    }

    return p_nextState;
}

static StateMachine_State_T * Stop_InputBrake(MotorController_T * p_mc, statemachine_inputext_t userCmdBrake)
{
    p_mc->UserCmd = (uint16_t)userCmdBrake;
    MotorController_GroundAll(p_mc); /* repeat set is okay for brake */
    return 0U;
}

#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static StateMachine_State_T * Stop_InputServo(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    return &STATE_SERVO;
}
#endif

/* This function is blocking */
static StateMachine_State_T * Stop_InputSaveParams_Blocking(MotorController_T * p_mc, statemachine_inputext_t substateId)
{
    /* Flash Write will enter critical */
    p_mc->NvmStatus = NV_MEMORY_STATUS_PROCESSING;
    switch(substateId)
    {
        case MOTOR_CONTROLLER_NVM_PARAMS_ALL:   p_mc->NvmStatus = MotorController_SaveParameters_Blocking(p_mc);    break;
        case MOTOR_CONTROLLER_NVM_BOOT:         p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_mc);       break;
#if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
        case MOTOR_CONTROLLER_NVM_WRITE_ONCE:   p_mc->NvmStatus = MotorController_SaveOnce_Blocking(p_mc);          break;
#endif
        // case MOTOR_CONTROLLER_TOGGLE_USER_INPUT_MODE:
        //     if (p_mc->Parameters.UserInputMode == MOTOR_CONTROLLER_INPUT_MODE_ANALOG) {p_mc->Parameters.UserInputMode = MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL; }
        //     else if (p_mc->Parameters.UserInputMode == MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL) {p_mc->Parameters.UserInputMode = MOTOR_CONTROLLER_INPUT_MODE_ANALOG; }
        default: break;
    }

    return 0U;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]         = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_SET_DIRECTION] = (StateMachine_Transition_T)Stop_InputDirection,
    [MCSM_INPUT_CMD]           = (StateMachine_Transition_T)Stop_InputCmd,
    [MCSM_INPUT_THROTTLE]      = (StateMachine_Transition_T)Stop_InputThrottle,
    [MCSM_INPUT_BRAKE]         = (StateMachine_Transition_T)Stop_InputBrake,
    [MCSM_INPUT_ZERO]          = (StateMachine_Transition_T)0U,
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
    [MCSM_INPUT_SERVO]          = (StateMachine_Transition_T)Stop_InputServo,
#endif
    [MCSM_INPUT_CALIBRATION]    = (StateMachine_Transition_T)Stop_InputSaveParams_Blocking,
};

static const StateMachine_State_T STATE_STOP =
{
    .ID                 = MCSM_STATE_ID_STOP,
    .P_TRANSITION_TABLE = &STOP_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Output_T)Stop_Entry,
    .OUTPUT             = (StateMachine_Output_T)Stop_Proc,
};

/******************************************************************************/
/*!
    @brief  Stopping State?
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief Run State
    Motors may be in Run or Freewheel.
    Accepts both Throttle/Brake inputs.
    Analog UserInput - release/edge inputs implicitly tracks previous state.
*/
/******************************************************************************/
static void Run_Entry(MotorController_T * p_mc) { (void)p_mc; }
static void Run_Proc(MotorController_T * p_mc) { (void)p_mc; }

static StateMachine_State_T * Run_InputDirection(MotorController_T * p_mc, statemachine_inputext_t inputDirection)
{
    StateMachine_State_T * p_nextState;

    if(inputDirection == MOTOR_CONTROLLER_DIRECTION_NEUTRAL)
    {
        p_mc->ActiveDirection = MOTOR_CONTROLLER_DIRECTION_NEUTRAL;
        p_nextState = &STATE_NEUTRAL;
    }
    else /* Not applicable for AnalogUser, when transistion through neutral state. Includes setting same/active direction */
    {
        MotorController_BeepShort(p_mc);
        p_nextState = 0U;
    }

    return p_nextState;
}

/*
    Check Stop / Zero Throttle
*/
static StateMachine_State_T * _Run_InputProcZero(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    switch(p_mc->Parameters.ZeroCmdMode)
    {
        case MOTOR_CONTROLLER_ZERO_CMD_MODE_FLOAT: break;
        case MOTOR_CONTROLLER_ZERO_CMD_MODE_REGEN: /* MotorController_ProcRegenMotorAll(p_mc); */ break;
        // case MOTOR_CONTROLLER_ZERO_CMD_MODE_CRUISE: break;
        default: break;
    }
    return (MotorController_CheckStopMotorAll(p_mc) == true) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * _Run_InputSetZero(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    switch(p_mc->Parameters.ZeroCmdMode)
    {
        case MOTOR_CONTROLLER_ZERO_CMD_MODE_FLOAT: MotorController_ReleaseAll(p_mc); break;
        case MOTOR_CONTROLLER_ZERO_CMD_MODE_REGEN: /* MotorController_SetRegenMotorAll(p_mc); */ break;
        // case MOTOR_CONTROLLER_ZERO_CMD_MODE_CRUISE: MotorController_SetCruiseMotorAll(p_mc); break;
        default: break;
    }
    return 0U;
}

static StateMachine_State_T * Run_InputZero(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    StateMachine_State_T * p_nextState = 0U;
    // volatile int32_t stop = 0;
    // volatile int32_t stop2 = stop;

    if(p_mc->UserCmd == 0U) { p_nextState = _Run_InputProcZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    else                    { p_mc->UserCmd = 0U; p_nextState = _Run_InputSetZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    return p_nextState;
}

static StateMachine_State_T * Run_InputCmd(MotorController_T * p_mc, statemachine_inputext_t userCmd)
{
    // if(userCmd == 0U)   { Run_InputZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    // else                { p_mc->UserCmd = (int16_t)userCmd; MotorController_SetCmd(p_mc, userCmd); };
    return 0U;
}

static StateMachine_State_T * Run_InputThrottle(MotorController_T * p_mc, statemachine_inputext_t userCmdThrottle)
{
    StateMachine_State_T * p_nextState = 0U;
    if(userCmdThrottle == 0U)   { p_nextState = Run_InputZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    else                        { p_mc->UserCmd = (uint16_t)userCmdThrottle; MotorController_SetThrottle(p_mc, userCmdThrottle); };
    return p_nextState;
}

static StateMachine_State_T * Run_InputBrake(MotorController_T * p_mc, statemachine_inputext_t userCmdBrake)
{
    StateMachine_State_T * p_nextState = 0U;
    if(userCmdBrake == 0U)  { p_nextState = Run_InputZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    else                    { p_mc->UserCmd = (uint16_t)userCmdBrake; MotorController_SetBrake(p_mc, userCmdBrake); };
    return p_nextState;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_SET_DIRECTION]  = (StateMachine_Transition_T)Run_InputDirection,
    [MCSM_INPUT_ZERO]           = (StateMachine_Transition_T)Run_InputZero,
    [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)Run_InputCmd,
    [MCSM_INPUT_THROTTLE]       = (StateMachine_Transition_T)Run_InputThrottle,
    [MCSM_INPUT_BRAKE]          = (StateMachine_Transition_T)Run_InputBrake,
    [MCSM_INPUT_CALIBRATION]    = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_RUN =
{
    .ID                     = MCSM_STATE_ID_RUN,
    .P_TRANSITION_TABLE     = &RUN_TRANSITION_TABLE[0U],
    .ENTRY                  = (StateMachine_Output_T)Run_Entry,
    .OUTPUT                 = (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief  Neutral State
    Brake effective. Throttle no effect.
    Motors in Freewheel or Stop state, or Run when braking.
    Motor may transition between Motor Freewheel and Stop, on 0 speed
    but MCSM remains in Neutral state, motor floating, due to input
*/
/******************************************************************************/
/* If entry while braking, will experience brief discontinuity */ /* todo handles ignore while braking */
static void Neutral_Entry(MotorController_T * p_mc) { MotorController_ReleaseAll(p_mc); }
static void Neutral_Proc(MotorController_T * p_mc) { (void)p_mc; }

static StateMachine_State_T * Neutral_InputDirection(MotorController_T * p_mc, statemachine_inputext_t inputDirection)
{
    StateMachine_State_T * p_nextState = 0U;

    if(MotorController_ProcUserDirection(p_mc, inputDirection) == true)
    {
        p_mc->ActiveDirection = inputDirection;
        if((p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_FORWARD) || (p_mc->ActiveDirection == MOTOR_CONTROLLER_DIRECTION_REVERSE))
        {
            p_nextState = &STATE_RUN;
        }
    }
    else /* Motors layer return error */
    {
        MotorController_BeepShort(p_mc);
    }

    return p_nextState;
}

/*
    Waits for user to input brake before transitioning to Stop
*/
static StateMachine_State_T * Neutral_InputZero(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    if(p_mc->UserCmd != 0U) { p_mc->UserCmd = 0U; MotorController_ReleaseAll(p_mc);}
    return 0U;
}

static StateMachine_State_T * Neutral_InputBrake(MotorController_T * p_mc, statemachine_inputext_t userCmdBrake)
{
    if(userCmdBrake == 0U)  { Neutral_InputZero(p_mc, STATE_MACHINE_INPUT_VALUE_NULL); }
    else                    { p_mc->UserCmd = (uint16_t)userCmdBrake; MotorController_SetBrake(p_mc, userCmdBrake); };

    return (MotorController_CheckStopMotorAll(p_mc) == true) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * Neutral_InputCmd(MotorController_T * p_mc, statemachine_inputext_t userCmd)
{

}

static const StateMachine_Transition_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_SET_DIRECTION]  = (StateMachine_Transition_T)Neutral_InputDirection,
    [MCSM_INPUT_ZERO]           = (StateMachine_Transition_T)Neutral_InputZero,
    [MCSM_INPUT_BRAKE]          = (StateMachine_Transition_T)Neutral_InputBrake,
    [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)Neutral_InputCmd,
    [MCSM_INPUT_THROTTLE]       = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_CALIBRATION]    = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_NEUTRAL =
{
    .P_TRANSITION_TABLE     = &NEUTRAL_TRANSITION_TABLE[0U],
    .ENTRY                  = (StateMachine_Output_T)Neutral_Entry,
    .OUTPUT                 = (StateMachine_Output_T)Neutral_Proc,
};


/******************************************************************************/
/*!
    @brief Servo State
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static void Servo_Entry(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Start(p_mc);
#else
    MotorController_Servo_Start(p_mc);
#endif
}

static void Servo_Proc(MotorController_T * p_mc)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_Proc(p_mc);
#else
    MotorController_Servo_Proc(p_mc);
#endif
}

static StateMachine_State_T * Servo_InputExit(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    MotorController_ReleaseAll(p_mc);
    return &STATE_STOP;
}

static StateMachine_State_T * Servo_InputCmd(MotorController_T * p_mc, statemachine_inputext_t cmd)
{
#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
    MotorController_ServoExtern_SetCmd(p_mc, cmd);
#else
    MotorController_Servo_SetCmd(p_mc);
#endif
    return 0U;
}

static const StateMachine_Transition_T SERVO_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]         = (StateMachine_Transition_T)TransitionFault,
    [MCSM_INPUT_SET_DIRECTION] = (StateMachine_Transition_T)Servo_InputExit,
    [MCSM_INPUT_CMD]           = (StateMachine_Transition_T)Servo_InputCmd,
    [MCSM_INPUT_THROTTLE]      = (StateMachine_Transition_T)Servo_InputCmd,
    [MCSM_INPUT_BRAKE]         = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_ZERO]          = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_SERVO]         = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_CALIBRATION]   = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_SERVO =
{
    .ID                 = MCSM_STATE_ID_SERVO,
    .P_TRANSITION_TABLE = &SERVO_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Output_T)Servo_Entry,
    .OUTPUT             = (StateMachine_Output_T)Servo_Proc,
};
#endif

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(MotorController_T * p_mc)
{
    MotorController_DisableAll(p_mc);
#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
    memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
#endif
    Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U);
}

static void Fault_Proc(MotorController_T * p_mc)
{
    MotorController_DisableAll(p_mc);

    switch(p_mc->Parameters.UserInputMode)
    {
        case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL: /* Protocol Rx Lost use auto recover, without user input */
            p_mc->FaultFlags.RxLost = Protocol_CheckRxLost(&p_mc->CONFIG.P_PROTOCOLS[0U]);
            break;
        case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
        default:  break;
    }

    if(p_mc->FaultFlags.State == 0U)
    {
        Blinky_Stop(&p_mc->Buzzer);
        _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_STOP);
    }
}

/* Fault State Input Fault Checks Fault */
/* Sensor faults only clear on user input */
static StateMachine_State_T * Fault_InputFault(MotorController_T * p_mc, statemachine_inputext_t voidVar)
{
    (void)voidVar;
    p_mc->FaultFlags.Motors       = (MotorController_ClearFaultMotorAll(p_mc) == false);
    p_mc->FaultFlags.VSenseLimit  = VMonitor_GetIsStatusLimit(&p_mc->VMonitorSense);
    p_mc->FaultFlags.VAccLimit    = VMonitor_GetIsStatusLimit(&p_mc->VMonitorAcc);
    p_mc->FaultFlags.VSourceLimit = VMonitor_GetIsStatusLimit(&p_mc->VMonitorSource);
    p_mc->FaultFlags.PcbOverHeat  = Thermistor_GetIsShutdown(&p_mc->ThermistorPcb);
#if defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
    p_mc->FaultFlags.MosfetsTopOverHeat = Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsTop);
    p_mc->FaultFlags.MosfetsBotOverHeat = Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsBot);
#else
    p_mc->FaultFlags.MosfetsOverHeat = Thermistor_GetIsShutdown(&p_mc->ThermistorMosfets);
#endif

    p_mc->FaultFlags.User = 0U;
    return 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
    [MCSM_INPUT_FAULT]          = (StateMachine_Transition_T)Fault_InputFault,
    [MCSM_INPUT_CALIBRATION]    = (StateMachine_Transition_T)Stop_InputSaveParams_Blocking,
    [MCSM_INPUT_SET_DIRECTION]  = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_THROTTLE]       = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_BRAKE]          = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_ZERO]           = (StateMachine_Transition_T)0U,
    [MCSM_INPUT_CMD]            = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_FAULT =
{
    .ID                 = MCSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Output_T)Fault_Entry,
    .OUTPUT             = (StateMachine_Output_T)Fault_Proc,
};
