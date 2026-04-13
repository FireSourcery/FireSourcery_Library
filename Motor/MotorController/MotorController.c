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

void MotorController_Init(const MotorController_T * p_dev)
{
    MotorController_State_T * p_mc = p_dev->P_MC;

    MotNvm_Init(&p_dev->MOT_NVM);

    if (p_dev->P_NVM_CONFIG != NULL) { p_mc->Config = *p_dev->P_NVM_CONFIG; }
    if (p_dev->MOT_NVM.P_BOOT_REF != NULL) { p_mc->BootRef.Word = p_dev->MOT_NVM.P_BOOT_REF->Word; }

    for (uint8_t iAnalog = 0U; iAnalog < p_dev->ADC_COUNT; iAnalog++) { Analog_ADC_Init(&p_dev->P_ANALOG_ADCS[iAnalog]); }
    for (uint8_t iSerial = 0U; iSerial < p_dev->SERIAL_COUNT; iSerial++) { Serial_Init(&p_dev->P_SERIALS[iSerial]); }

#if defined(MOTOR_CONTROLLER_CAN_BUS_ENABLE)
    // if(p_mc->Config.IsCanEnable == true) { CanBus_Init(p_dev->P_CAN_BUS, p_mc->Config.CanServicesId); }
#endif

    for (uint8_t iProtocol = 0U; iProtocol < p_dev->PROTOCOL_COUNT; iProtocol++) { Socket_Init(&p_dev->P_PROTOCOLS[iProtocol]); }

    MotAnalogUser_Init(&p_dev->ANALOG_USER);

    VMonitor_Init(&p_dev->V_SOURCE);
    /* Overwrite */
    VDivider_ToLinear(&(VDivider_T) { .R1 = PHASE_ANALOG_SENSOR_REF.V_PHASE_R1, .R2 = PHASE_ANALOG_SENSOR_REF.V_PHASE_R2, }, p_dev->V_SOURCE.P_LINEAR);

    VMonitor_Init(&p_dev->V_ACCESSORIES);
    VMonitor_Init(&p_dev->V_ANALOG);

    HeatMonitor_Init(&p_dev->HEAT_PCB);
    HeatMonitor_Group_Init(&p_dev->HEAT_MOSFETS);

    Phase_VBus_InitV(p_mc->Config.VSupplyRef);
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
    Analog_Conversion_Mark(&p_dev->V_SOURCE_CONVERSION);
    Analog_Conversion_Mark(&p_dev->V_ACCESSORIES_CONVERSION);
    Analog_Conversion_Mark(&p_dev->V_ANALOG_CONVERSION);

    StateMachine_Init(&p_dev->STATE_MACHINE);
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
void MotorController_LoadConfigDefault(const MotorController_T * p_dev)
{
    RangeMonitor_Enable(p_dev->V_SOURCE.P_STATE);
    MotorController_ResetVSourceMonitorDefaults(p_dev);

    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorAccs);
    // VMonitor_ResetLimitsDefault(&p_mc->VMonitorSense);
    // for (uint8_t iMotor = 0U; iMotor < p_dev->MOTORS.LENGTH; iMotor++)
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
// fault past 57% or vPhaseFract16*vBusInv_Fract32 may overflow
void _MotorController_SetVSupplyRef(const MotorController_T * p_dev, uint16_t volts)
{
    p_dev->P_MC->Config.VSupplyRef = math_min(volts, Phase_Calibration_GetVRated_V());
    MotorController_ResetVSourceMonitorDefaults(p_dev); /* may overwrite fault/warning if called in the same packet */
}

/* auto value using */
void MotorController_InitVSupplyAutoValue(const MotorController_T * p_dev)
{
    assert(Phase_Calibration_IsValid() == true); /* Must be loaded before */
    _MotorController_SetVSupplyRef(p_dev, Linear_Voltage_Of(p_dev->V_SOURCE.P_LINEAR, Analog_Conversion_GetResult(&p_dev->V_SOURCE_CONVERSION)));
}

void MotorController_ResetVSourceMonitorDefaults(const MotorController_T * p_dev)
{
    VMonitor_InitLimitsDefault(p_dev->V_SOURCE.P_STATE, Linear_Voltage_AdcuOfV(p_dev->V_SOURCE.P_LINEAR, p_dev->P_MC->Config.VSupplyRef), 25, 15, 5);
}
// bool MotorController_ValidateVSupplyMonitor(const MotorController_T * p_dev)
// {
//    RangeMonitor_Config_T * p_config = &p_dev->V_SOURCE.P_STATE->Config;
//    uint32_t nominal = Linear_Voltage_AdcuOfV(p_dev->V_SOURCE.P_LINEAR, p_dev->P_MC->Config.VSupplyRef);
// //    math_is_in_range(p_config->FaultOverLimit.Limit, nominal * 70 / 100, nominal * 130 / 100);
// }


/******************************************************************************/
/*
    Collective Setting Limit
*/
/******************************************************************************/
bool _MotorController_SetSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimit_Id_T id, limit_t speed_fract16)
{
    if (LimitArray_TrySetEntry(&p_dev->MOT_SPEED_LIMITS, id, speed_fract16) == true) { Motor_Table_ApplySpeedLimit(&p_dev->MOTORS, &p_dev->MOT_SPEED_LIMITS); }
}

bool _MotorController_ClearSpeedLimitAll(const MotorController_T * p_dev, MotSpeedLimit_Id_T id)
{
    if (LimitArray_TryClearEntry(&p_dev->MOT_SPEED_LIMITS, id) == true) { Motor_Table_ApplySpeedLimit(&p_dev->MOTORS, &p_dev->MOT_SPEED_LIMITS); }
}

bool _MotorController_SetILimitAll(const MotorController_T * p_dev, MotILimit_Id_T id, limit_t i_fract16)
{
    if (LimitArray_TrySetEntry(&p_dev->MOT_I_LIMITS, id, i_fract16) == true) { Motor_Table_ApplyILimit(&p_dev->MOTORS, &p_dev->MOT_I_LIMITS); }
}

bool _MotorController_ClearILimitAll(const MotorController_T * p_dev, MotILimit_Id_T id)
{
    if (LimitArray_TryClearEntry(&p_dev->MOT_I_LIMITS, id) == true) { Motor_Table_ApplyILimit(&p_dev->MOTORS, &p_dev->MOT_I_LIMITS); }
}


