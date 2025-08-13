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

void MotorController_Init(const MotorController_T * p_context)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    MotNvm_Init(&p_context->MOT_NVM);

    if (p_context->P_NVM_CONFIG != NULL) { p_mc->Config = *p_context->P_NVM_CONFIG; }
    if (p_context->MOT_NVM.P_BOOT_REF != NULL) { p_mc->BootRef.Word = p_context->MOT_NVM.P_BOOT_REF->Word; }

    for (uint8_t iAnalog = 0U; iAnalog < p_context->ADC_COUNT; iAnalog++) { Analog_ADC_Init(&p_context->P_ANALOG_ADCS[iAnalog]); }
    for (uint8_t iSerial = 0U; iSerial < p_context->SERIAL_COUNT; iSerial++) { Serial_Init(&p_context->P_SERIALS[iSerial]); }

#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if(p_mc->Config.IsCanEnable == true) { CanBus_Init(p_context->P_CAN_BUS, p_mc->Config.CanServicesId); }
#endif

    for (uint8_t iProtocol = 0U; iProtocol < p_context->PROTOCOL_COUNT; iProtocol++) { Socket_Init(&p_context->P_PROTOCOLS[iProtocol]); }

    MotAnalogUser_Init(&p_context->ANALOG_USER);

    VMonitor_Init(&p_context->V_SOURCE);
    /* Overwrite */
    VDivider_ToLinear(&(VDivider_T) { .R1 = MOTOR_ANALOG_REFERENCE_BOARD.V_PHASE_R1, .R2 = MOTOR_ANALOG_REFERENCE_BOARD.V_PHASE_R2, }, p_context->V_SOURCE.P_LINEAR);

    VMonitor_Init(&p_context->V_ACCESSORIES);
    VMonitor_Init(&p_context->V_ANALOG);

    HeatMonitor_Init(&p_context->HEAT_PCB);
    HeatMonitor_Group_Init(&p_context->HEAT_MOSFETS);

    MotorAnalog_InitVSource_V(p_mc->Config.VSupplyRef);
    for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++) { Motor_Init(&p_context->MOTORS.P_CONTEXTS[iMotor]); }

    Blinky_Init(&p_context->BUZZER);
    Blinky_Init(&p_context->METER);
    Pin_Output_Init(&p_context->RELAY_PIN);
    UserDIn_Init(&p_context->OPT_DIN); /* 5-10ms by default */

    TimerT_Periodic_Init(&p_context->MILLIS_TIMER, 1U);

#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
    Shell_Init(&p_mc->Shell);
#endif

    MotDrive_Init(&p_context->MOT_DRIVE);

    /* Alternatively set nominal on init */
    // TimerT_Counter_InitTickOnInit(&p_context->MILLIS_TIMER, 1U);
    HeatMonitor_Group_MarkEach(&p_context->HEAT_MOSFETS);
    HeatMonitor_MarkConversion(&p_context->HEAT_PCB);
    Analog_Conversion_Mark(&p_context->V_SOURCE.ANALOG_CONVERSION);
    Analog_Conversion_Mark(&p_context->V_ACCESSORIES.ANALOG_CONVERSION);
    Analog_Conversion_Mark(&p_context->V_ANALOG.ANALOG_CONVERSION);

    StateMachine_Init(&p_context->STATE_MACHINE);
}

/******************************************************************************/
/*!
    Set/Reset
    Runtime only
*/
/******************************************************************************/
/*
    Set runtime Config (RAM copy) via abstraction layer functions (in user units)
    Convenience function over p_mc->Config compile time initializers
    On first time boot up. propagate defaults

    when if (BootRef_IsValid() == false)
*/
void MotorController_LoadConfigDefault(const MotorController_T * p_context)
{
    // RangeMonitor_Enable(p_context->V_SOURCE.P_STATE);
    MotorController_ResetVSourceMonitorDefaults(p_context);

    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);
    // for (uint8_t iMotor = 0U; iMotor < p_context->MOTORS.LENGTH; iMotor++)
}

void MotorController_ResetVSourceMonitorDefaults(const MotorController_T * p_context)
{
    VMonitor_InitLimitsDefault(p_context->V_SOURCE.P_STATE, Linear_Voltage_AdcuOfV(p_context->V_SOURCE.P_LINEAR, p_context->P_ACTIVE->Config.VSupplyRef), 25, 15, 5);
}

void MotorController_ResetBootDefault(MotorController_State_T * p_mc)
{
    static const BootRef_T BOOT_REF_DEFAULT = { .IsValid = BOOT_REF_IS_VALID_01, .FastBoot = 0U, .Beep = 1U, .Blink = 1U, }; /* Overwrite after first time boot */
    p_mc->BootRef.Word = BOOT_REF_DEFAULT.Word;
}

/******************************************************************************/
/*

*/
/******************************************************************************/
void _MotorController_SetVSupplyRef(const MotorController_T * p_context, uint16_t volts)
{
    p_context->P_ACTIVE->Config.VSupplyRef = math_min(volts, MotorAnalogRef_GetVRated_V());
    MotorController_ResetVSourceMonitorDefaults(p_context); /* may overwrite fault/warning if called in the same packet */
}

void MotorController_InitVSupplyAutoValue(const MotorController_T * p_context)
{
    assert(MotorAnalogRef_IsLoaded() == true); /* Must be loaded before */
    _MotorController_SetVSupplyRef(p_context, Linear_Voltage_Of(p_context->V_SOURCE.P_LINEAR, Analog_Conversion_GetResult(&p_context->V_SOURCE.ANALOG_CONVERSION)));
}


/******************************************************************************/
/*
    Collective Setting Limit
*/
/******************************************************************************/
bool MotorController_SetSpeedLimitAll(const MotorController_T * p_context, MotSpeedLimit_Id_T id, limit_t limit_fract16)
{
    if (LimitArray_SetEntry(&p_context->MOT_SPEED_LIMITS, id, limit_fract16) == true) { MotMotors_ApplySpeedLimit(&p_context->MOTORS, &p_context->MOT_SPEED_LIMITS); }
}

bool MotorController_ClearSpeedLimitAll(const MotorController_T * p_context, MotSpeedLimit_Id_T id)
{
    if (LimitArray_ClearEntry(&p_context->MOT_SPEED_LIMITS, id) == true) { MotMotors_ApplySpeedLimit(&p_context->MOTORS, &p_context->MOT_SPEED_LIMITS); }
}

bool MotorController_SetILimitAll(const MotorController_T * p_context, MotILimit_Id_T id, limit_t limit_fract16)
{
    if (LimitArray_SetEntry(&p_context->MOT_I_LIMITS, id, limit_fract16) == true) { MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS); }
}

bool MotorController_ClearILimitAll(const MotorController_T * p_context, MotILimit_Id_T id)
{
    if (LimitArray_ClearEntry(&p_context->MOT_I_LIMITS, id) == true) { MotMotors_ApplyILimit(&p_context->MOTORS, &p_context->MOT_I_LIMITS); }
}



