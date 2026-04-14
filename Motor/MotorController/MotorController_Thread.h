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

#include "Peripheral/Analog/Analog_ADC_Thread.h"

/******************************************************************************/
/*

*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_ANALOG_USER_DIVIDER
#define MOTOR_CONTROLLER_ANALOG_USER_DIVIDER (15U) /* Approx. 10 ms */
#endif

#ifndef MOTOR_CONTROLLER_MAIN_DIVIDER_10
#define MOTOR_CONTROLLER_MAIN_DIVIDER_10 (15U) /* Approx. 10 ms */
#endif

#ifndef MOTOR_CONTROLLER_MAIN_DIVIDER_1000
#define MOTOR_CONTROLLER_MAIN_DIVIDER_1000 (1023U) /* Approx. 1 S */
#endif

// #ifndef MOTOR_CONTROLLER_TIMER_DIVIDER_1000
// #define MOTOR_CONTROLLER_TIMER_DIVIDER_1000 1023U /* Approx. 1 S */
// #endif


/******************************************************************************/
/*
    Mapped to Thread - Proc in All States
*/
/******************************************************************************/
static inline void _MotorController_ProcAnalogUser(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    MotAnalogUser_Cmd_T cmd;

    MotAnalogUser_CaptureInput(&p_dev->ANALOG_USER, MotAnalogUser_Conversion_GetThrottle(&p_dev->ANALOG_USER_CONVERSIONS), MotAnalogUser_Conversion_GetBrake(&p_dev->ANALOG_USER_CONVERSIONS));

    MotorController_App_ProcAnalogUser((MotorController_T *)p_dev);

    if (TimerT_Counter_IsAligned(&p_dev->MILLIS_TIMER, MOTOR_CONTROLLER_ANALOG_USER_DIVIDER) == true)
    {
        MotAnalogUser_Conversion_Mark(&p_dev->ANALOG_USER_CONVERSIONS);
    }
}


/*
    Optional Din
*/
static inline void _MotorController_ProcOptDin(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    uint8_t dinStatus = 0U;

    if (p_mc->Config.OptDinMode != MOTOR_CONTROLLER_OPT_DIN_DISABLE)
    {
        UserDIn_PollEdge(&p_dev->OPT_DIN);

        switch (p_mc->Config.OptDinMode)
        {
            case MOTOR_CONTROLLER_OPT_DIN_DISABLE: break;
            case MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT:
                switch (UserDIn_GetEdge(&p_dev->OPT_DIN))
                {
                    case USER_DIN_EDGE_RISING:  MotorController_SetUserSpeedLimitAll(p_dev, p_mc->Config.OptSpeedLimit_Fract16); break;
                    case USER_DIN_EDGE_FALLING: MotorController_ClearUserSpeedLimitAll(p_dev); break;
                    default: break;
                }
                break;
            // #ifdef MOTOR_CONTROLLER_SERVO_ENABLE
            // case MOTOR_CONTROLLER_OPT_DIN_SERVO:
            //     switch (Debounce_GetEdge(&p_mc->OptDin))
            //     {
            //         case DEBOUNCE_EDGE_RISING:  MotorController_EnterServoMode(p_mc);  break;
            //         case DEBOUNCE_EDGE_FALLING: MotorController_ExitServoMode(p_mc);   break;
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
static inline void _MotorController_HeatMonitor_Thread(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    HeatMonitor_Status_T status;

    /* Poll PCB Temperature Monitor */
    switch (HeatMonitor_Poll(&p_dev->HEAT_PCB, Analog_Conversion_GetResult(&p_dev->HEAT_PCB_CONVERSION)))
    {
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_PCB_OVERHEAT);
            break;
        case HEAT_MONITOR_STATUS_WARNING_HIGH: break;
        case HEAT_MONITOR_STATUS_NORMAL: break;
        default: break;
    }

    /* Poll each MOSFET sensor individually */
    for (uint8_t i = 0U; i < p_dev->HEAT_MOSFETS.COUNT; i++)
    {
        HeatMonitor_Poll(&p_dev->HEAT_MOSFETS.P_MONITORS[i], Analog_Conversion_GetResult(&p_dev->P_HEAT_MOSFET_CONVERSIONS[i]));
    }

    /* Poll MOSFET Temperature Monitors Group Collective */
    switch (HeatMonitor_Group_PollCollective(&p_dev->HEAT_MOSFETS))
    {
        case HEAT_MONITOR_STATUS_FAULT_OVERHEAT:
            MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_MOSFETS_OVERHEAT);
            break;

        case HEAT_MONITOR_STATUS_WARNING_HIGH:
            /*
                Apply thermal current limiting based on hottest MOSFETs
                Does not check for edge trigger
                Thermistor Adcu is roughly linear in Warning region
                Increasing Limit only, reset on warning clear.
            */
           _MotorController_SetILimitAll(p_dev, MOT_I_LIMIT_HEAT_MC, fract16_mul(HeatMonitor_Group_GetScalarLimit_Percent16(&p_dev->HEAT_MOSFETS) / 2, Phase_Calibration_GetIRatedPeak_Fract16()));
            break;

        case HEAT_MONITOR_STATUS_NORMAL:
            if (Monitor_IsWarningClearing(p_dev->HEAT_MOSFETS.P_STATE)) { _MotorController_ClearILimitAll(p_dev, MOT_I_LIMIT_HEAT_MC); }
            break;

        default:
            break;
    }


    if (Monitor_IsWarningTriggering(p_dev->HEAT_PCB.P_STATE) || Monitor_IsWarningTriggering(p_dev->HEAT_MOSFETS.P_STATE))
    {
        MotorController_BeepMonitorTrigger(p_dev);
    }


    /* Mark analog conversions for next cycle */
    for (uint8_t i = 0U; i < p_dev->HEAT_MOSFETS.COUNT; i++) { Analog_Conversion_Mark(&p_dev->P_HEAT_MOSFET_CONVERSIONS[i]); }
    Analog_Conversion_Mark(&p_dev->HEAT_PCB_CONVERSION);

    // /* Process individual motor heat monitoring */
    // for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { Motor_Heat_Thread(&p_dev->P_MOTOR_CONSTS[iMotor]); }
}

/******************************************************************************/
/*
    VSource Monitor Thread
*/
/******************************************************************************/
static void _MotorController_VSourceMonitor_EnterFault(const MotorController_T * p_dev)
{
    Motor_Table_ForceDisableControl(&p_dev->MOTORS);
    /*
        if this interrupts a lower priority transition, that transition may overwrite the fault transition, causing exit from fault.
        if overwritten, main will check fault flags and enter, or on next poll.
        input handlers store a local copy of the function pointer before performing null check. so it should not crash the program.
    */
    MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_VSOURCE_LIMIT);
}

static inline void _MotorController_VSourceMonitor_Thread(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    // alternatively
    // Phase_Analog_CaptureVBus(Analog_Conversion_GetResult(&p_dev->V_SOURCE_CONVERSION));
    // switch (RangeMonitor_Poll(p_dev->V_SOURCE.P_STATE, Phase_VBus_Fract16()))

// #if defined(MOTOR_V_SENSORS_ANALOG)
    switch (RangeMonitor_Poll(p_dev->V_SOURCE.P_STATE, Analog_Conversion_GetResult(&p_dev->V_SOURCE_CONVERSION)))
    {
        case VMONITOR_STATUS_FAULT_OVERVOLTAGE:  _MotorController_VSourceMonitor_EnterFault(p_dev); break;
        case VMONITOR_STATUS_FAULT_UNDERVOLTAGE: _MotorController_VSourceMonitor_EnterFault(p_dev); break;
        case VMONITOR_STATUS_WARNING_HIGH:
            break;
        case VMONITOR_STATUS_WARNING_LOW:
            if (RangeMonitor_IsTriggeringEdge(p_dev->V_SOURCE.P_STATE) == true)
            {
                MotorController_CaptureVSource(p_dev);
                // LimitArray_TrySetEntry(&p_dev->MOT_I_LIMITS, MOT_I_LIMIT_V_LOW, p_mc->Config.VLowILimit_Fract16);
            }
            break;
        case VMONITOR_STATUS_NORMAL:
            if (RangeMonitor_IsClearingEdge(p_dev->V_SOURCE.P_STATE) == true)
            {
                // LimitArray_TryClearEntry(&p_dev->MOT_I_LIMITS, MOT_I_LIMIT_V_LOW);
            }
            break;
        default: break;
    }

    if (RangeMonitor_IsTriggeringEdge(p_dev->V_SOURCE.P_STATE) == true) { MotorController_BeepMonitorTrigger(p_dev); }

    Analog_Conversion_Mark(&p_dev->V_SOURCE_CONVERSION);

// #endif
}

/******************************************************************************/
/*
   VAux Monitor Thread
*/
/******************************************************************************/
static inline void _MotorController_VMonitorBoard_Thread(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    RangeMonitor_Poll(p_dev->V_ACCESSORIES.P_STATE, Analog_Conversion_GetResult(&p_dev->V_ACCESSORIES_CONVERSION));
    RangeMonitor_Poll(p_dev->V_ANALOG.P_STATE, Analog_Conversion_GetResult(&p_dev->V_ANALOG_CONVERSION));

    if (RangeMonitor_IsAnyFault(p_dev->V_ACCESSORIES.P_STATE) == true) { MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_VACCS_LIMIT); }
    if (RangeMonitor_IsAnyFault(p_dev->V_ANALOG.P_STATE) == true) { MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_VANALOG_LIMIT); }

    if (p_mc->FaultFlags.Value != 0U) { MotorController_SetFault(p_dev, (MotorController_FaultFlags_T){ .Value = p_mc->FaultFlags.Value }); }

    Analog_Conversion_Mark(&p_dev->V_ACCESSORIES_CONVERSION);
    Analog_Conversion_Mark(&p_dev->V_ANALOG_CONVERSION);
}


/******************************************************************************/
/*
    Main
    High Freq, Low Priority,
*/
/******************************************************************************/

static inline void MotorController_Main_Thread(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    /* High Freq, Low Priority */
    if (TimerT_Counter_Poll(&p_dev->MILLIS_TIMER) == true)
    {
        /*
            Med Freq, Low Priority, 1 ms
        */
        /* SubStates update on proc, at least once Motor_StateMachine will have processed */
        /* Handle Inputs as they are received */
        // maybe interrupted by enterFault on 1ms thread
        _StateMachine_RootFirst_ProcSyncOutput(p_dev->STATE_MACHINE.P_ACTIVE, (void *)p_dev);

        for (uint8_t iProtocol = 0U; iProtocol < p_dev->PROTOCOL_COUNT; iProtocol++) { Socket_Proc(&p_dev->P_PROTOCOLS[iProtocol]); }

    // #ifdef MOTOR_CONTROLLER_CAN_BUS_ENABLE
    //     if (p_mc->Config.IsCanEnable == true) { CanBus_ProcServices(p_dev->P_CAN_BUS); }
    // #endif
        /* Proc in all State */
        switch (p_mc->Config.InputMode)
        {
            case MOTOR_CONTROLLER_INPUT_MODE_ANALOG:
                _MotorController_ProcAnalogUser(p_dev);
                break;
            case MOTOR_CONTROLLER_INPUT_MODE_SERIAL:
                /* optionally */
                // if (MotAnalogUser_PollBrakePins(&p_dev->ANALOG_USER) == true) { MotorController_ForceDisableControl(p_mc); }
                /* Only active when Serial is selected as primary input */
                // if (MotorController_PollRxLost(p_dev) == true)
                // {
                // Motor_Table_ForEachApply(&p_dev->MOTORS, Motor_ReleaseVZ);
                //     MotorController_ForceDisableControl(p_dev);
                //     MotorController_EnterFault(p_dev);
                // }
                break;
            case MOTOR_CONTROLLER_INPUT_MODE_CAN:
                break;
        }

        /*
            Low Freq, Low Priority, ~10ms ~16ms, 100Hz
        */
        if (TimerT_Counter_IsAligned(&p_dev->MILLIS_TIMER, MOTOR_CONTROLLER_MAIN_DIVIDER_10) == true)
        {
        #ifdef MOTOR_CONTROLLER_SHELL_ENABLE
            Shell_Proc(&p_mc->Shell);
        #endif
            Blinky_Proc(&p_dev->BUZZER);
            Blinky_Proc(&p_dev->METER);
        }

        /*
            Low Freq, Low Priority, ~1s ~1024ms
        */
        if (TimerT_Counter_IsAligned(&p_dev->MILLIS_TIMER, MOTOR_CONTROLLER_MAIN_DIVIDER_1000) == true)
        {
            /* In case of Serial Rx Overflow Timeout */
            for (uint8_t iSerial = 0U; iSerial < p_dev->SERIAL_COUNT; iSerial++) { Serial_PollRestartRxIsr(&p_dev->P_SERIALS[iSerial]); }

            _MotorController_ProcOptDin(p_dev);
            _MotorController_VMonitorBoard_Thread(p_dev); /* Except VSupply */
            _MotorController_HeatMonitor_Thread(p_dev);

            /* Can use low priority check, as motor is already in fault state. */
            if (Motor_Table_IsAnyState(&p_dev->MOTORS, MOTOR_STATE_ID_FAULT) == true) { MotorController_SetFault(p_dev, MOTOR_CONTROLLER_FAULT_MOTORS); }

            if (p_mc->FaultFlags.Value != 0U) { MotorController_SetFault(p_dev, (MotorController_FaultFlags_T){ .Value = p_mc->FaultFlags.Value }); }

            MotorController_CaptureVSource(p_dev); /* update vout ratios  Set Motors VSupplyRef using ADC reading. Low Freq unless in warning region */

        }
    }
    // todo transient recorder proc
}

/*
    Med Freq, Med-High Priority
*/
static inline void MotorController_Timer1Ms_Thread(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;
    _MotorController_VSourceMonitor_Thread(p_dev);

    // if (p_mc->Config.InputMode != MOTOR_CONTROLLER_INPUT_MODE_ANALOG)
    // {
    //     if (MotAnalogUser_PollBrakePins(&p_dev->ANALOG_USER) == true) { MotorController_ForceDisableControl(p_mc); }
    // }

#if defined(MOTOR_CONTROLLER_DEBUG_ENABLE) || defined(MOTOR_DEBUG_ENABLE)
    // _Blinky_Toggle(&p_mc->Meter);
#endif
}



/*
    High Freq, High Priority
*/
/* Alternatively these can be placed directly user main if the compiler does not optimize */
// static inline void MotorController_PWM_Thread(const MotorController_T * p_dev)
// {
//     // p_fields->MicrosRef = SysTime_GetMicros();

//     // for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { Motor_MarkAnalog_Thread(&p_dev->MOTORS.P_MONITORS[iMotor]); }

//     // if (Motor_IsAnalogCycle(&p_dev->MOTORS.P_MONITORS[0U]) == true) /* todo common timer */
//     // {
//     //     for (uint8_t iAdc = 0U; iAdc < p_dev->ADC_COUNT; iAdc++) { Analog_ADC_ProcMarked(&p_dev->P_ANALOG_ADCS[iAdc]); }
//     // }

//     if (_Motor_IsAnalogCycle(p_dev->P_MC->ControlCounter) == true)
//     {
//         for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { _Motor_Analog_Thread(&p_dev->MOTORS.P_MONITORS[iMotor]); }
//     }

//     // if (_Motor_IsAnalogCycle(p_dev->P_MC->ControlCounter) == true) /* removable */
//     for (uint8_t iAdc = 0U; iAdc < p_dev->ADC_COUNT; iAdc++) { Analog_ADC_ProcMarked(&p_dev->P_ANALOG_ADCS[iAdc]); }


//     for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { Motor_PWM_Thread(&p_dev->MOTORS.P_MONITORS[iMotor]); }

//     // timer_counter_wrapped(1000U, p_fields->MicrosRef, SysTime_GetMicros());
//     // HAL_PWM_ClearInterrupt(p_dev->HAL_PWM); /* BOARD_PWM_HAL */
//     // Motor_ClearInterrupt(&p_dev->MOTORS.P_ARRAY[0U]);
//     p_dev->P_MC->ControlCounter++;
// }

