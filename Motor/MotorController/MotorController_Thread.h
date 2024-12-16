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
    @file   MotorController_Thread.h
    @author FireSourcery
    @brief  MotorController module functions must be placed into corresponding user app threads.
            Most outer functions to call from MCU app.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_THREAD_H
#define MOTOR_CONTROLLER_THREAD_H

#include "MotorController_User.h"
#include "MotorController_Analog.h"
#include "Motor/Motor/Motor_Thread.h"

static inline bool CheckDividerMask(uint32_t num, uint32_t align) { return ((num & align) == 0UL); }

/******************************************************************************/
/*
    Mapped to Thread - Proc in All States
*/
/******************************************************************************/
static inline void _MotorController_ProcAnalogUser(MotorController_T * p_mc)
{
    MotAnalogUser_Cmd_T cmd = MotAnalogUser_PollCmd(&p_mc->AnalogUser);
    MotAnalogUser_CaptureInput(&p_mc->AnalogUser, MotorController_Analog_GetThrottle(p_mc), MotorController_Analog_GetBrake(p_mc));

    /* MotAnalog user implements edge detect implemented and cmd priority, but handled by state machine */
    switch(cmd)
    {
        case MOT_ANALOG_USER_CMD_SET_BRAKE:                 MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrake(&p_mc->AnalogUser));          break;
        case MOT_ANALOG_USER_CMD_SET_THROTTLE:              MotorController_User_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottle(&p_mc->AnalogUser));    break;
        case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:         MotorController_User_SetCmdBrake(p_mc, 0U);                                                 break;
        case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:      MotorController_User_SetCmdThrottle(p_mc, 0U);                                              break;
        case MOT_ANALOG_USER_CMD_PROC_ZERO:                 MotorController_User_SetCmdDriveZero(p_mc);                                                 break;
        case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:     MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);                break;
        case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:     MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);                break;
        case MOT_ANALOG_USER_CMD_SET_NEUTRAL:               MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_NEUTRAL);                break;
        case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:  break;
        default: break;
    }

    if(CheckDividerMask(p_mc->MainDividerCounter, p_mc->CONST.ANALOG_USER_DIVIDER) == true)
    {
        Analog_MarkConversion(&p_mc->CONST.CONVERSION_THROTTLE);
        Analog_MarkConversion(&p_mc->CONST.CONVERSION_BRAKE);
    }
}

/*
    Optional Din
*/
static inline void _MotorController_ProcOptDin(MotorController_T * p_mc)
{
    uint8_t dinStatus = 0U;

    if(p_mc->Config.OptDinMode != MOTOR_CONTROLLER_OPT_DIN_DISABLE)
    {
        Debounce_CaptureState(&p_mc->OptDin);

        switch(p_mc->Config.OptDinMode)
        {
            case MOTOR_CONTROLLER_OPT_DIN_DISABLE: break;
            case MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT:
                switch(Debounce_GetEdge(&p_mc->OptDin))
                {
                    case DEBOUNCE_EDGE_RISING:  MotorController_User_SetSpeedLimitAll(p_mc, p_mc->Config.OptSpeedLimit_Percent16); break;
                    case DEBOUNCE_EDGE_FALLING: MotorController_User_ClearSpeedLimitAll(p_mc); break;
                    default: break;
                }
                break;
            #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
            case MOTOR_CONTROLLER_OPT_DIN_SERVO:
                switch(Debounce_GetEdge(&p_mc->OptDin))
                {
                    case DEBOUNCE_EDGE_RISING:  MotorController_User_EnterServoMode(p_mc);  break;
                    case DEBOUNCE_EDGE_FALLING: MotorController_User_ExitServoMode(p_mc);   break;
                    default: break;
                }
                break;
            #endif
            default: break;
        }
    }
}

/* Monitor Threads only set fault flags. Do not clear until user input clear */
static inline void _MotorController_ProcHeatMonitor(MotorController_T * p_mc)
{
    bool isFault = false;
    bool isWarning = false;

    Analog_MarkConversion(&p_mc->CONST.CONVERSION_HEAT_PCB);
    for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
        { Analog_MarkConversion(&p_mc->CONST.HEAT_MOSFETS_CONVERSIONS[iMosfets]); }

    Thermistor_PollMonitor(&p_mc->ThermistorPcb, MotorController_Analog_GetHeatPcb(p_mc));
    if (Thermistor_IsFault(&p_mc->ThermistorPcb) == true) { p_mc->FaultFlags.PcbOverheat = 1U; isFault = true; }

    for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
    {
        Thermistor_PollMonitor(&p_mc->MosfetsThermistors[iMosfets], MotorController_Analog_GetHeatMosfets(p_mc, iMosfets));
        if (Thermistor_IsFault(&p_mc->MosfetsThermistors[iMosfets]) == true) { p_mc->FaultFlags.MosfetsOverheat = 1U; isFault = true; }
    }

    if(isFault == true)
    {
        MotorController_StateMachine_EnterFault(p_mc); /* Shutdown repeat set ok */
    }
    else
    {
        /* Warning behaviors edge triggered */
        isWarning |= Thermistor_IsWarning(&p_mc->ThermistorPcb);
        for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
            { isWarning |= Thermistor_IsWarning(&p_mc->MosfetsThermistors[iMosfets]); }

        if(isWarning == true)
        {
            /*
                Thermistor Adcu is roughly linear in Warning region
                Assume Heat Mosfets as highest heat
                constantly compares lowest active limit, alternatively, check and restore prev on clear limit
                Increasing Limit only, reset on warning clear.
            */
            MotorController_SetSystemILimitAll(p_mc, Thermistor_GetHeatLimit_Percent16(&p_mc->MosfetsThermistors[0U]));

            // Thermistor_PollWarningRisingEdge(&p_mc->ThermistorMosfets ); // use highest or
            if(p_mc->StateFlags.HeatWarning == false)
            {
                Blinky_BlinkN(&p_mc->Buzzer, 250U, 250U, 1U);
                p_mc->StateFlags.HeatWarning = true;
            }
        }
        else
        {
            // Thermistor_PollWarningFallingEdge(&p_mc->ThermistorMosfets );
            if(p_mc->StateFlags.HeatWarning == true)
            {
                MotorController_ClearSystemILimitAll(p_mc); //clear mosfet
                p_mc->StateFlags.HeatWarning = false;
            }
        }
    }

    for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_Heat_Thread(&p_mc->CONST.P_MOTORS[iMotor]); }
}

static inline void _MotorController_ProcVoltageMonitor(MotorController_T * p_mc)
{
    bool isFault = false;

    Analog_MarkConversion(&p_mc->CONST.CONVERSION_VACCS);
    Analog_MarkConversion(&p_mc->CONST.CONVERSION_VSENSE);

    VMonitor_PollStatus(&p_mc->VMonitorSense,  MotorController_Analog_GetVSense(p_mc));
    VMonitor_PollStatus(&p_mc->VMonitorAccs,  MotorController_Analog_GetVAccs(p_mc));
    if (VMonitor_IsFault(&p_mc->VMonitorSense) == true) { p_mc->FaultFlags.VSenseLimit = 1U; isFault = true; }
    if (VMonitor_IsFault(&p_mc->VMonitorAccs) == true) { p_mc->FaultFlags.VAccsLimit = 1U; isFault = true; }

    if (isFault == true) { MotorController_StateMachine_EnterFault(p_mc); } /* Sensors checks fault only */
}

/*
    Main
    High Freq, Low Priority,
*/
static inline void MotorController_Main_Thread(MotorController_T * p_mc)
{
    /* High Freq, Low Priority */

    /* Med Freq, Low Priority, 1 ms */
    if (Timer_Periodic_Poll(&p_mc->TimerMillis) == true)
    {
        StateMachine_Async_ProcState(&p_mc->StateMachine);

        for (uint8_t iProtocol = 0U; iProtocol < p_mc->CONST.PROTOCOL_COUNT; iProtocol++) { Protocol_Proc(&p_mc->CONST.P_PROTOCOLS[iProtocol]); }
    #ifdef CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE
        if (p_mc->Config.IsCanEnable == true) { CanBus_ProcServices(p_mc->CONST.P_CAN_BUS); }
    #endif

        /* Real-Time, continuous polling, inputs require arbitration */
        switch (p_mc->Config.InputMode)
        {
            case MOTOR_CONTROLLER_INPUT_MODE_ANALOG: _MotorController_ProcAnalogUser(p_mc);  break;
            case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
                /* MotorController_Var_Set voluntarily checks InputMode for proc */
                if (MotAnalogUser_PollBrakePins(&p_mc->AnalogUser) == true) { MotorController_User_ForceDisableControl(p_mc); }

                if (Protocol_IsRxLost(&p_mc->CONST.P_PROTOCOLS[p_mc->CONST.USER_PROTOCOL_INDEX]) == true)
                {
                    MotorController_User_ForceDisableControl(p_mc);
                    p_mc->FaultFlags.RxLost = 1U;
                    MotorController_StateMachine_EnterFault(p_mc);
                }
                break;
                // case MOTOR_CONTROLLER_INPUT_MODE_DISABLE: break;
            case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
            default:  break;
        }

        /* Bypass Isr */
        // for(uint8_t iSerial = 0U; iSerial < p_mc->CONST.SERIAL_COUNT; iSerial++)
        // {
        //     Serial_PollRxData(&p_mc->CONST.P_SERIALS[iSerial]);
        //     Serial_PollTxData(&p_mc->CONST.P_SERIALS[iSerial]);
        // }

        /* Low Freq, Low Priority, ~10ms ~16ms, 100Hz */
        if (CheckDividerMask(p_mc->MainDividerCounter, p_mc->CONST.MAIN_DIVIDER_10) == true)
        {
        #ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
            Shell_Proc(&p_mc->Shell);
        #endif
            // if (p_mc->StateFlags.BuzzerEnable) { Blinky_Proc(&p_mc->Buzzer); }
            Blinky_Proc(&p_mc->Buzzer);
            Blinky_Proc(&p_mc->Meter);
        }

        /* Low Freq, Low Priority, ~1s ~1024ms */
        if (CheckDividerMask(p_mc->MainDividerCounter, p_mc->CONST.MAIN_DIVIDER_1000) == true)
        {
            /* In case of Serial Rx Overflow Timeout */
            for (uint8_t iSerial = 0U; iSerial < p_mc->CONST.SERIAL_COUNT; iSerial++) { Serial_PollRestartRxIsr(&p_mc->CONST.P_SERIALS[iSerial]); }

            _MotorController_ProcOptDin(p_mc);
            _MotorController_ProcVoltageMonitor(p_mc); /* Except VSupply */
            _MotorController_ProcHeatMonitor(p_mc);
            /* Can use low priority check, as motor is already in fault state */
            if (MotorController_IsAnyMotorFault(p_mc) == true) { p_mc->FaultFlags.Motors = 1U; MotorController_StateMachine_EnterFault(p_mc); }
            // for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_Heat_Thread(&p_mc->CONST.P_MOTORS[iMotor]); }

        #if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
            // _Blinky_Toggle(&p_mc->Meter);
            // _Blinky_Toggle(&p_mc->Buzzer);
            // volatile uint32_t test = VMonitor_ChargeLevelOfAdcu_Percent16(&p_mc->VMonitorSource, p_mc->AnalogResults.VSource_Adcu);
        #endif
        }

        p_mc->MainDividerCounter++; //Timer_GetBase(p_timer)
    }
}

/*
    Med Freq, Med-High Priority

    Thread Safety: Async Set Fault, is okay?
        Set ILimit - possible mix match sentinel value to id?
*/
static inline void MotorController_Timer1Ms_Thread(MotorController_T * p_mc)
{
    p_mc->TimerDividerCounter++;
    //    BrakeThread(p_mc);
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    VMonitor_Status_T vStatus = VMonitor_PollStatus(&p_mc->VMonitorSource, MotorController_Analog_GetVSource(p_mc)); // todo include edge

    switch (vStatus)
    {
        case VMONITOR_FAULT_UPPER: p_mc->FaultFlags.VSourceLimit = 1U; MotorController_StateMachine_EnterFault(p_mc); break;
        case VMONITOR_FAULT_LOWER: p_mc->FaultFlags.VSourceLimit = 1U; MotorController_StateMachine_EnterFault(p_mc); break;
        case VMONITOR_WARNING_UPPER:
            break;
        case VMONITOR_WARNING_LOWER:
            if (p_mc->StateFlags.VLow == 0U)
            {
                p_mc->StateFlags.VLow = 1U;
                // MotorController_SetSystemILimitAll(p_mc, p_mc->Config.VLowILimit_Percent16);
                Blinky_BlinkN(&p_mc->Buzzer, 500U, 250U, 2U);
            }
            break;
        case VMONITOR_STATUS_OK:
            if (p_mc->StateFlags.VLow == 1U)
            {
                p_mc->StateFlags.VLow = 0U;
                // MotorController_ClearSystemILimitAll(p_mc);
            }
            break;
        default: break;
    }

    Analog_MarkConversion(&p_mc->CONST.CONVERSION_VSOURCE);
#endif

    // if (p_mc->Config.InputMode != MOTOR_CONTROLLER_INPUT_MODE_ANALOG)
    // {
    //     if (MotAnalogUser_PollBrakePins(&p_mc->AnalogUser) == true) { MotorController_User_ForceDisableControl(p_mc); }
    // }

    // if(CheckDividerMask(p_mc->TimerDividerCounter, p_mc->CONST.TIMER_DIVIDER_1000) == true)
    // {
    //     _MotorController_ProcVoltageMonitor(p_mc); /* Except voltage supply */
    //     _MotorController_ProcHeatMonitor(p_mc);
    //     for(uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_Heat_Thread(&p_mc->CONST.P_MOTORS[iMotor]); }
    // }
#if defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    // _Blinky_Toggle(&p_mc->Meter);
#endif
}

/*
    High Freq, High Priority
*/
static inline void MotorController_PWM_Thread(MotorController_T * p_mc)
{
    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_MarkAnalog_Thread(&p_mc->CONST.P_MOTORS[iMotor]); }

    if ((Motor_IsAnalogCycle(&p_mc->CONST.P_MOTORS[0U]) == true)) /* todo common timer */
    {
        Analog_StartConversions(p_mc->CONST.P_ANALOG);
    }

    for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTOR_COUNT; iMotor++) { Motor_PWM_Thread(&p_mc->CONST.P_MOTORS[iMotor]); }
    Motor_ClearInterrupt(&p_mc->CONST.P_MOTORS[0U]);
}

#endif
