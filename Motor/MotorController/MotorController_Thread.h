#pragma once

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
    @file   MotorController_Thread.h
    @author FireSourcery
    @brief  MotorController module functions to be placed into corresponding user app threads.
*/
/******************************************************************************/
#include "MotorController_User.h"
#include "Motor/Motor/Motor_Thread.h"

#include "MotDrive/MotDrive_StateMachine.h"
#include "MotDrive/MotDrive_User.h"

#include "Peripheral/Analog/Analog_ADC_Thread.h"

static inline bool IsDividerAlign(uint32_t counter, uint32_t align) { return ((counter & align) == 0UL); }

/******************************************************************************/
/*
    Mapped to Thread - Proc in All States
*/
/******************************************************************************/
static inline void _MotorController_ProcAnalogUser(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    MotAnalogUser_Cmd_T cmd;

    MotAnalogUser_CaptureInput(&p_context->ANALOG_USER, MotAnalogUser_Conversion_GetThrottle(&p_context->ANALOG_USER_CONVERSIONS), MotAnalogUser_Conversion_GetBrake(&p_context->ANALOG_USER_CONVERSIONS));
    // cmd = MotAnalogUser_PollCmd(&p_context->ANALOG_USER);

    // switch (cmd)
    // {
    //     // case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));          break;
    //     // case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_User_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));    break;
    //     //                                                     // MotDrive_SetThrottleValue(&p_mc->MotDrive, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
    //     // case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:         MotorController_User_SetCmdBrake(p_mc, 0U);                                                 break;
    //     // case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:      MotorController_User_SetCmdThrottle(p_mc, 0U);                                              break;
    //     // case MOT_ANALOG_USER_CMD_PROC_ZERO:                 MotorController_User_SetCmdDriveZero(p_mc);                                                 break;
    //     // case MOT_ANALOG_USER_CMD_SET_NEUTRAL:               MotorController_User_SetDirection(p_mc, MOT_DRIVE_DIRECTION_NEUTRAL);                       break;
    //     case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:
    //         // MotorController_User_ExitLockState(p_mc);
    //         // MotorController_User_SetDirection(p_mc, MOT_DRIVE_DIRECTION_FORWARD);
    //         break;
    //     case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:
    //     // MotorController_User_ExitLockState(p_mc);
    //         // MotorController_User_SetDirection(p_mc, MOT_DRIVE_DIRECTION_REVERSE);
    //         break;
    //     case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:  break;
    //     default: break;
    // }


    // if drive mode
    // switch on app macchine
    switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    {
        case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotDrive_User_SetDirection(&p_context->MOT_DRIVE, MOT_DRIVE_DIRECTION_FORWARD);   break;
        case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotDrive_User_SetDirection(&p_context->MOT_DRIVE, MOT_DRIVE_DIRECTION_REVERSE);   break;
        case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotDrive_User_SetDirection(&p_context->MOT_DRIVE, MOT_DRIVE_DIRECTION_NEUTRAL);   break;
        default: break;
    }

    MotDrive_User_SetThrottle(p_context->MOT_DRIVE.P_ACTIVE, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER));
    MotDrive_User_SetBrake(p_context->MOT_DRIVE.P_ACTIVE, MotAnalogUser_GetBrake(&p_context->ANALOG_USER));

    // else
    // switch (MotAnalogUser_GetDirectionEdge(&p_context->ANALOG_USER))
    // {
    //     case MOT_ANALOG_USER_DIRECTION_FORWARD_EDGE:  MotorController_User_SetDirection(&p_context->MOT_DRIVE, 1);   break;
    //     case MOT_ANALOG_USER_DIRECTION_REVERSE_EDGE:  MotorController_User_SetDirection(&p_context->MOT_DRIVE, -1);   break;
    //     case MOT_ANALOG_USER_DIRECTION_NEUTRAL_EDGE:  MotorController_User_SetDirection(&p_context->MOT_DRIVE, 0);   break;
    //     default: break;
    // }

    // if (MotAnalogUser_IsAnyBrakeOn(&p_context->ANALOG_USER) == true) { MotorController_User_SetCmdValue(p_context, 0U); }
    // else { MotorController_User_SetCmdValue(p_context, MotAnalogUser_GetThrottle(&p_context->ANALOG_USER) / 2U); }

    if (IsDividerAlign(p_mc->MainDividerCounter, p_context->ANALOG_USER_DIVIDER) == true) { MotAnalogUser_Conversion_Mark(&p_context->ANALOG_USER_CONVERSIONS); }
}


/*
    Optional Din
*/
static inline void _MotorController_ProcOptDin(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    uint8_t dinStatus = 0U;

    if (p_mc->Config.OptDinMode != MOTOR_CONTROLLER_OPT_DIN_DISABLE)
    {
        UserDIn_PollEdge(&p_context->OPT_DIN);

        switch (p_mc->Config.OptDinMode)
        {
            case MOTOR_CONTROLLER_OPT_DIN_DISABLE: break;
            case MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT:
                switch (UserDIn_GetEdge(&p_context->OPT_DIN))
                {
                    case USER_DIN_EDGE_RISING:  MotorController_User_SetSpeedLimitAll(p_context, p_mc->Config.OptSpeedLimit_Fract16); break;
                    case USER_DIN_EDGE_FALLING: MotorController_User_ClearSpeedLimitAll(p_context); break;
                    default: break;
                }
                break;
            // #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
            // case MOTOR_CONTROLLER_OPT_DIN_SERVO:
            //     switch (Debounce_GetEdge(&p_mc->OptDin))
            //     {
            //         case DEBOUNCE_EDGE_RISING:  MotorController_User_EnterServoMode(p_mc);  break;
            //         case DEBOUNCE_EDGE_FALLING: MotorController_User_ExitServoMode(p_mc);   break;
            //         default: break;
            //     }
            //     break;
            // #endif
            default: break;
        }
    }
}

/*
    Monitor Threads only set fault flags. Do not clear until user input clear
*/
/******************************************************************************/
/*
    Heat Monitor Thread
*/
/******************************************************************************/
static inline void _MotorController_HeatMonitor_Thread(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    HeatMonitor_Status_T status;

    /* Poll PCB Temperature Monitor */
    switch (HeatMonitor_Poll(&p_context->HEAT_PCB))
    {
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            p_mc->FaultFlags.PcbOverheat = 1U;
            MotorController_StateMachine_EnterFault(p_context);
            break;
        case HEAT_MONITOR_STATUS_WARNING_HIGH: break;
        case HEAT_MONITOR_STATUS_NORMAL: break;
        default: break;
    }

    /* Poll MOSFET Temperature Monitors Group */
    switch (HeatMonitor_Group_PollAll(&p_context->HEAT_MOSFETS))
    {
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            p_mc->FaultFlags.MosfetsOverheat = 1U;
            MotorController_StateMachine_EnterFault(p_context);
            break;

        case HEAT_MONITOR_STATUS_WARNING_HIGH:
            /*
                Apply thermal current limiting based on hottest MOSFET
                Does not check for edge trigger
                Thermistor Adcu is roughly linear in Warning region
                Increasing Limit only, reset on warning clear.
            */
            LimitArray_SetEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_HEAT_MC, HeatMonitor_Group_GetScalarLimit_Percent16(&p_context->HEAT_MOSFETS) / 2U);
            MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS);
            // if (Monitor_IsWarningTriggering(p_context->HEAT_MOSFETS.P_STATE)) {MotorController_BeepMonitorTrigger(p_context); }
            break;

        case HEAT_MONITOR_STATUS_NORMAL:
            if (Monitor_IsWarningClearing(p_context->HEAT_MOSFETS.P_STATE))
            {
                LimitArray_ClearEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_HEAT_MC);
                MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS);
            }
            break;

        default:
            break;
    }


    if (Monitor_IsWarningTriggering(p_context->HEAT_PCB.P_STATE) || Monitor_IsWarningTriggering(p_context->HEAT_MOSFETS.P_STATE))
    {
        MotorController_BeepMonitorTrigger(p_context);
    }


    /* Mark analog conversions for next cycle */
    HeatMonitor_Group_MarkEach(&p_context->HEAT_MOSFETS);
    HeatMonitor_MarkConversion(&p_context->HEAT_PCB);

    // /* Process individual motor heat monitoring */
    // for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_Heat_Thread(&p_context->P_MOTOR_CONSTS[iMotor]); }
}

/******************************************************************************/
/*
    VSource Monitor Thread
*/
/******************************************************************************/
static inline void _MotorController_VSourceMonitor_Thread(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    switch (RangeMonitor_Poll(p_context->V_SOURCE.P_STATE, Analog_Conversion_GetResult(&p_context->V_SOURCE.ANALOG_CONVERSION)))
    {
        //todo
        /* No sync protection, if overwritten, main will check fault flags and enter, or on next poll */
        /* if the signal is not acquired, main will check fault flags and enter */
        case VMONITOR_STATUS_FAULT_OVERVOLTAGE: p_mc->FaultFlags.VSourceLimit = 1U; MotorController_StateMachine_EnterFault(p_context); break;
        case VMONITOR_STATUS_FAULT_UNDERVOLTAGE: p_mc->FaultFlags.VSourceLimit = 1U; MotorController_StateMachine_EnterFault(p_context); break;
        case VMONITOR_STATUS_WARNING_HIGH:
            break;
        case VMONITOR_STATUS_WARNING_LOW:
            if (RangeMonitor_IsTriggeringEdge(p_context->V_SOURCE.P_STATE) == true)
            {
                MotorController_CaptureVSource(p_context);
                LimitArray_SetEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_V_LOW, p_mc->Config.VLowILimit_Fract16);
            }
            break;
        case VMONITOR_STATUS_NORMAL:
            if (RangeMonitor_IsClearingEdge(p_context->V_SOURCE.P_STATE) == true)
            {
                LimitArray_ClearEntry(&p_context->MOT_I_LIMITS, MOT_I_LIMIT_V_LOW);
            }
            break;
        default: break;
    }

    if (RangeMonitor_IsTriggeringEdge(p_context->V_SOURCE.P_STATE) == true) { MotorController_BeepMonitorTrigger(p_context); }

    Analog_Conversion_MarkConversion(&p_context->V_SOURCE.ANALOG_CONVERSION);

#endif
}

/******************************************************************************/
/*
   VAux Monitor Thread
*/
/******************************************************************************/
static inline void _MotorController_VMonitorBoard_Thread(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    RangeMonitor_Poll(p_context->V_ACCESSORIES.P_STATE, Analog_Conversion_GetResult(&p_context->V_ACCESSORIES.ANALOG_CONVERSION));
    RangeMonitor_Poll(p_context->V_ANALOG.P_STATE, Analog_Conversion_GetResult(&p_context->V_ANALOG.ANALOG_CONVERSION));

    if (RangeMonitor_IsAnyFault(p_context->V_ACCESSORIES.P_STATE) == true) { p_mc->FaultFlags.VAccsLimit = 1U; MotorController_StateMachine_EnterFault(p_context); }
    if (RangeMonitor_IsAnyFault(p_context->V_ANALOG.P_STATE) == true) { p_mc->FaultFlags.VAnalogLimit = 1U; MotorController_StateMachine_EnterFault(p_context); }

    if (p_mc->FaultFlags.Value != 0U) { MotorController_StateMachine_EnterFault(p_context); }

    Analog_Conversion_MarkConversion(&p_context->V_ACCESSORIES.ANALOG_CONVERSION);
    Analog_Conversion_MarkConversion(&p_context->V_ANALOG.ANALOG_CONVERSION);
}


/******************************************************************************/
/*
    Main
    High Freq, Low Priority,
*/
/******************************************************************************/
static inline void MotorController_Main_Thread(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    /* High Freq, Low Priority */
    if (Timer_Periodic_Poll(&p_mc->TimerMillis) == true)
    {
        /*
            Med Freq, Low Priority, 1 ms
        */
        /* SubStates update on proc, at least once Motor_StateMachine will have processed */
        /* Handle Inputs as they are received */
        // _StateMachine_ProcSyncOutput(&p_mc->StateMachine, p_mc); // maybe change this to signal if enter fault is on 1ms thread
        _StateMachine_ProcSyncOutput(p_context->STATE_MACHINE.P_ACTIVE, p_mc);

        /* Proc app Machine */ /* let compiler optimize */
        // if (StateMachine_IsActiveStateId(p_context->STATE_MACHINE.P_ACTIVE, MCSM_STATE_ID_MAIN) == true) { MotDrive_Proc_Thread(&p_context->MOT_DRIVE); }

        // switch (p_mc->StateMachine.State)
        // {
        //     case MOTOR_CONTROLLER_STATE_INIT:   break;
        //     case MOTOR_CONTROLLER_STATE_MAIN:    break;
        //     case MOTOR_CONTROLLER_STATE_LOCK:   break;
        //     case MOTOR_CONTROLLER_STATE_FAULT:  break;
        //     default: break;
        // }

        for (uint8_t iProtocol = 0U; iProtocol < p_context->PROTOCOL_COUNT; iProtocol++) { Protocol_Proc(&p_context->P_PROTOCOLS[iProtocol]); }

    #ifdef CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE
        if (p_mc->Config.IsCanEnable == true) { CanBus_ProcServices(p_context->P_CAN_BUS); }
    #endif

        switch (p_mc->Config.InputMode)
        {
            // case MOTOR_CONTROLLER_INPUT_MODE_DISABLE: break;
            case MOTOR_CONTROLLER_INPUT_MODE_ANALOG: _MotorController_ProcAnalogUser(p_context);  break;
            case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
                // if (MotAnalogUser_PollBrakePins(&p_context->ANALOG_USER) == true) { MotorController_User_ForceDisableControl(p_mc); }
                /* Only active when Serial is selected as primary input */
                if (MotorController_PollRxLost(p_context) == true)
                {
                    MotorController_User_ForceDisableControl(p_context);
                    MotorController_StateMachine_EnterFault(p_context);
                }
                break;
            case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
            default:  break;
        }

        /* As polling */
        // for(uint8_t iSerial = 0U; iSerial < p_context->SERIAL_COUNT; iSerial++)
        // {
        //     Serial_PollRxData(&p_context->P_SERIALS[iSerial]);
        //     Serial_PollTxData(&p_context->P_SERIALS[iSerial]);
        // }

        /*
            Low Freq, Low Priority, ~10ms ~16ms, 100Hz
        */
        if (IsDividerAlign(p_mc->MainDividerCounter, p_context->MAIN_DIVIDER_10) == true)
        {
        #ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
            Shell_Proc(&p_mc->Shell);
        #endif
            Blinky_Proc(&p_context->BUZZER);
            Blinky_Proc(&p_context->METER);
        }

        /*
            Low Freq, Low Priority, ~1s ~1024ms
        */
        if (IsDividerAlign(p_mc->MainDividerCounter, p_context->MAIN_DIVIDER_1000) == true)
        {
            /* In case of Serial Rx Overflow Timeout */
            for (uint8_t iSerial = 0U; iSerial < p_context->SERIAL_COUNT; iSerial++) { Serial_PollRestartRxIsr(&p_context->P_SERIALS[iSerial]); }

            _MotorController_ProcOptDin(p_context);
            _MotorController_VMonitorBoard_Thread(p_context); /* Except VSupply */
            _MotorController_HeatMonitor_Thread(p_context);

            /* Can use low priority check, as motor is already in fault state */
            if (MotMotors_IsAnyState(&p_context->MOTORS, MSM_STATE_ID_FAULT) == true) { p_mc->FaultFlags.Motors = 1U; }

            if (p_mc->FaultFlags.Value != 0U) { MotorController_StateMachine_EnterFault(p_context); }

            MotorController_CaptureVSource(p_context); /* update vout ratios */

        #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)

        #endif
        }

        p_mc->MainDividerCounter++;
    }
    // todo transient recorder proc
}

/*
    Med Freq, Med-High Priority
*/
static inline void MotorController_Timer1Ms_Thread(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;
    _MotorController_VSourceMonitor_Thread(p_context);

    // BrakeThread(p_mc);
    // if (p_mc->Config.InputMode != MOTOR_CONTROLLER_INPUT_MODE_ANALOG)
    // {
    //     if (MotAnalogUser_PollBrakePins(&p_context->ANALOG_USER) == true) { MotorController_User_ForceDisableControl(p_mc); }
    // }

#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    // _Blinky_Toggle(&p_mc->Meter);
#endif
    p_mc->TimerDividerCounter++;
}



/*
    High Freq, High Priority
*/
/* Alternatively these can be placed directly user main if the compiler does not optimize */
static inline void MotorController_PWM_Thread(const MotorController_T * p_context)
{
    // for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_MarkAnalog_Thread(&p_context->MOTORS.P_CONTEXTS[iMotor]); }

    // if (Motor_IsAnalogCycle(&p_context->MOTORS.P_CONTEXTS[0U]) == true) /* todo common timer */
    // {
    //     for (uint8_t iAdc = 0U; iAdc < p_context->ADC_COUNT; iAdc++) { Analog_ADC_ProcMarked(&p_context->P_ANALOG_ADCS[iAdc]); }
    // }

    if (MotorTimeRef_IsAnalogCycle(p_context->P_ACTIVE->ControlCounter) == true)
    {
        for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { _Motor_MarkAnalog_Thread(&p_context->MOTORS.P_CONTEXTS[iMotor]); }
    }

    // if (MotorTimeRef_IsAnalogCycle(p_context->P_ACTIVE->ControlCounter) == true) /* removable */
    for (uint8_t iAdc = 0U; iAdc < p_context->ADC_COUNT; iAdc++) { Analog_ADC_ProcMarked(&p_context->P_ANALOG_ADCS[iAdc]); }


    for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_PWM_Thread(&p_context->MOTORS.P_CONTEXTS[iMotor]); }

    // HAL_PWM_ClearInterrupt(p_context->HAL_PWM); /* BOARD_PWM_HAL */
    // Motor_ClearInterrupt(&p_context->MOTORS.P_ARRAY[0U]);
    p_context->P_ACTIVE->ControlCounter++;
}
