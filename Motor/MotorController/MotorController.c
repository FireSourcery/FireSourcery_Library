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
    @file   MotorController.c
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#include "MotorController.h"
#include <string.h>

#include "MotAnalogUser/OptPin/MotorController_OptPin.h"

void MotorController_Init(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    MotNvm_Init(&p_dev->MOT_NVM);

    if (p_dev->P_NVM_CONFIG != NULL) { p_mc->Config = *p_dev->P_NVM_CONFIG; }
    if (p_dev->MOT_NVM.P_BOOT_REF != NULL) { p_mc->BootRef.Word = p_dev->MOT_NVM.P_BOOT_REF->Word; }

    for (uint8_t iAnalog = 0U; iAnalog < p_dev->ADC_COUNT; iAnalog++) { Analog_ADC_Init(&p_dev->P_ANALOG_ADCS[iAnalog]); }
    for (uint8_t iSerial = 0U; iSerial < p_dev->SERIAL_COUNT; iSerial++) { Serial_Init(&p_dev->P_SERIALS[iSerial]); }

#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if (p_dev->P_CAN_BUS == NULL && p_mc->Config.IsCanEnable == true) { CanBus_Init(p_dev->P_CAN_BUS); }
    if (p_dev->P_CAN_BUS == NULL) { CanBus_Init(p_dev->P_CAN_BUS); }
#endif

    for (uint8_t iProtocol = 0U; iProtocol < p_dev->PROTOCOL_COUNT; iProtocol++) { Socket_Init(&p_dev->P_PROTOCOLS[iProtocol]); }

    MotAnalogUser_Init(&p_dev->ANALOG_USER);

    _MotorController_InitOptDin(p_dev);

    VBus_InitFrom(p_dev->P_VBUS, p_dev->P_VBUS_NVM_CONFIG);

    VMonitor_Init(&p_dev->V_ACCESSORIES);
    VMonitor_Init(&p_dev->V_ANALOG);

    HeatMonitor_Init(&p_dev->HEAT_PCB);
    HeatMonitor_Group_Init(&p_dev->HEAT_MOSFETS);

    for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++) { Motor_Init(&p_dev->MOTORS.P_DEVS[iMotor]); }

    Blinky_Init(&p_dev->BUZZER);
    Blinky_Init(&p_dev->METER);
    Pin_Output_Init(&p_dev->RELAY_PIN);
    UserDIn_Init(&p_dev->OPT_DIN); /* 5-10ms by default */

    TimerT_Periodic_Init(&p_dev->MILLIS_TIMER, 1U);

#ifdef MOTOR_CONTROLLER_SHELL_ENABLE
    Shell_Init(&p_mc->Shell);
#endif

    MotorController_App_Init(p_dev);

    /* Alternatively set nominal on init */
    for (uint8_t i = 0U; i < p_dev->HEAT_MOSFETS.COUNT; i++) { Analog_Conversion_Mark(&p_dev->P_HEAT_MOSFET_CONVERSIONS[i]); }
    Analog_Conversion_Mark(&p_dev->HEAT_PCB_CONVERSION);
    Analog_Conversion_Mark(&p_dev->VBUS_CONVERSION);
    Analog_Conversion_Mark(&p_dev->V_ACCESSORIES_CONVERSION);
    Analog_Conversion_Mark(&p_dev->V_ANALOG_CONVERSION);

    LimitArray_ClearAll(&p_dev->SPEED_LIMIT_SOURCES);
    LimitArray_ClearAll(&p_dev->I_LIMIT_SOURCES);
    LimitArray_ClearAll(&p_dev->I_GEN_LIMIT_SOURCES);

    StateMachine_Init(&p_dev->STATE_MACHINE);
}

/******************************************************************************/
/*!
    Set/Reset
    Runtime only
*/
/******************************************************************************/
void MotorController_ResetBootDefault(MotorController_State_T * p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}



/******************************************************************************/
/*
    Collective Setting Limit
*/
/******************************************************************************/
bool _MotorController_SetSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimitId_T id, limit_t speed_fract16)
{
    if (LimitArray_TestSetUpper(&p_dev->SPEED_LIMIT_SOURCES, id, speed_fract16) == true) { Motor_Table_ApplySpeedLimit(&p_dev->MOTORS, &p_dev->SPEED_LIMIT_SOURCES); return true; }
    return false;
}

bool _MotorController_ClearSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimitId_T id)
{
    if (LimitArray_TestClearEntry(&p_dev->SPEED_LIMIT_SOURCES, id) == true) { Motor_Table_ApplySpeedLimit(&p_dev->MOTORS, &p_dev->SPEED_LIMIT_SOURCES); return true; }
    return false;
}

bool _MotorController_SetILimitAll(const MotorController_T * p_dev, MotILimitId_T id, limit_t i_fract16)
{
    if (LimitArray_TestSetUpper(&p_dev->I_LIMIT_SOURCES, id, i_fract16) == true) { Motor_Table_ApplyILimit(&p_dev->MOTORS, &p_dev->I_LIMIT_SOURCES); return true; }
    return false;
}

bool _MotorController_ClearILimitAll(const MotorController_T * p_dev, MotILimitId_T id)
{
    if (LimitArray_TestClearEntry(&p_dev->I_LIMIT_SOURCES, id) == true) { Motor_Table_ApplyILimit(&p_dev->MOTORS, &p_dev->I_LIMIT_SOURCES); return true; }
    return false;
}

bool _MotorController_SetIGenLimitAll(const MotorController_T * p_dev, MotIGenLimitId_T id, limit_t i_fract16)
{
    if (LimitArray_TestSetUpper(&p_dev->I_GEN_LIMIT_SOURCES, id, i_fract16) == true) { Motor_Table_ApplyIGenLimit(&p_dev->MOTORS, &p_dev->I_GEN_LIMIT_SOURCES); return true; }
    return false;
}

bool _MotorController_ClearIGenLimitAll(const MotorController_T * p_dev, MotIGenLimitId_T id)
{
    if (LimitArray_TestClearEntry(&p_dev->I_GEN_LIMIT_SOURCES, id) == true) { Motor_Table_ApplyIGenLimit(&p_dev->MOTORS, &p_dev->I_GEN_LIMIT_SOURCES); return true; }
    return false;
}


