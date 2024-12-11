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
    @file
    @author FireSourcery
    @brief    Text based UI.
            This modules calls Motor_User and MotorController_User functions only.
    @version V0
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
#include "MotorController_Shell.h"
#include "MotorController_User.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//todo User_Wrapper functions
/******************************************************************************/
/*!
    @brief     Cmd Functions
*/
/*! @{ */
/******************************************************************************/
static Cmd_Status_T Cmd_stop(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argc; (void)argv;
    MotorController_User_ForceDisableControl(p_mc);
    p_mc->CONST.P_MOTORS[0].Encoder.Align = ENCODER_ALIGN_NO;
    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_run(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    char * p_end;
    uint32_t value;

    // p_motor->DebugCounter = 0U;
    // p_motor->DebugCounter2 = 0U;
    // p_motor->DebugFlag = false;
    // p_motor->DebugError = 0U;

    if(argc == 1U) /* run */
    {
        MotorController_User_SetCmdThrottle(p_mc, 65536U / 20U);
    }
    else if(argc == 2U) /* run [num] */
    {
        if(strncmp(argv[1U], "ol", 3U) == 0U) /* openloop */
        {

        }
        else
        {
            value = strtoul(argv[1U], &p_end, 10);
            MotorController_User_SetCmdThrottle(p_mc, value);
        }
    }
    else if(argc == 3U) /* run [throttle/brake] [num] */
    {
        value = strtoul(argv[2U], &p_end, 10);
        if      (strncmp(argv[1U], "throttle", 9U) == 0U)   { MotorController_User_SetCmdThrottle(p_mc, value); }
        else if (strncmp(argv[1U], "brake", 6U) == 0U)      { MotorController_User_SetCmdBrake(p_mc, value); }
    }

    return CMD_STATUS_SUCCESS;
}

static void PrintPhase12(Terminal_T * p_term, uint8_t step)
{
    switch(step % 12U)
    {
        case 0U: Terminal_SendString(p_term, "Phase  A: "); break;
        case 1U: Terminal_SendString(p_term, "Phase AC: "); break;
        case 2U: Terminal_SendString(p_term, "Phase -C: "); break;
        case 3U: Terminal_SendString(p_term, "Phase BC: "); break;
        case 4U: Terminal_SendString(p_term, "Phase  B: "); break;
        case 5U: Terminal_SendString(p_term, "Phase BA: "); break;
        case 6U: Terminal_SendString(p_term, "Phase -A: "); break;
        case 7U: Terminal_SendString(p_term, "Phase CA: "); break;
        case 8U: Terminal_SendString(p_term, "Phase  C: "); break;
        case 9U: Terminal_SendString(p_term, "Phase CB: "); break;
        case 10U: Terminal_SendString(p_term, "Phase -B: "); break;
        case 11U: Terminal_SendString(p_term, "Phase AB: "); break;
        default: break;
    }
}

static void PrintPhase(Terminal_T * p_term, uint8_t step)
{
    switch(step % 6U)
    {
        case 0U: Terminal_SendString(p_term, "Phase  A: "); break;
        case 1U: Terminal_SendString(p_term, "Phase -C: "); break;
        case 2U: Terminal_SendString(p_term, "Phase  B: "); break;
        case 3U: Terminal_SendString(p_term, "Phase -A: "); break;
        case 4U: Terminal_SendString(p_term, "Phase  C: "); break;;
        case 5U: Terminal_SendString(p_term, "Phase -B: "); break;
        default: break;
    }
}

static void PrintSensor(Terminal_T * p_term, Motor_T * p_motor)
{
    switch(Motor_User_GetSensorMode(p_motor))
    {
        case MOTOR_SENSOR_MODE_HALL:
            // if(step % 6U == 0U)
            // {
            //     // Terminal_SendString(p_term, "AngularD: ");
            //     // Terminal_SendNum(p_term, Encoder_GetCounterD(&p_motor->Encoder));
            //     Terminal_SendString(p_term, "\r\n");
            //     Terminal_SendString(p_term, "Hall: ");
            // }
            Terminal_SendNum(p_term, Hall_ReadSensors(&p_motor->Hall).Value); Terminal_SendString(p_term, " ");
            break;

        case MOTOR_SENSOR_MODE_ENCODER:
            // Terminal_SendNum(p_term, step);                                                        Terminal_SendString(p_term, " ");
            Terminal_SendNum(p_term, p_motor->Encoder.TotalD);                    Terminal_SendString(p_term, ", ");
            Terminal_SendNum(p_term, Encoder_GetAngle(&p_motor->Encoder));        Terminal_SendString(p_term, ", ");
            // Terminal_SendNum(p_term, Encoder_Motor_GetElectricalTheta(&p_motor->Encoder));        Terminal_SendString(p_term, ", ");
            Terminal_SendNum(p_term, p_motor->Encoder.IndexCount);                Terminal_SendString(p_term, ", ");
            Terminal_SendNum(p_term, p_motor->Encoder.ErrorCount);                Terminal_SendString(p_term, "\r\n");
            break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_SENSOR_MODE_SIN_COS:
            SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
            Terminal_SendString(p_term, "Sin: ");         Terminal_SendNum(p_term, p_motor->AnalogResults.Sin_Adcu);
            Terminal_SendString(p_term, " Cos: ");         Terminal_SendNum(p_term, p_motor->AnalogResults.Cos_Adcu);
            Terminal_SendString(p_term, " Angle: ");     Terminal_SendNum(p_term, SinCos_GetElectricalAngle(&p_motor->SinCos));
            Terminal_SendString(p_term, "\r\n");
            break;
        #endif
        #if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
        case MOTOR_SENSOR_MODE_SENSORLESS:     break;
        #endif
        default:
            break;
    }
}


/* Need stop to release */
static Cmd_Status_T Cmd_phase(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    //    Terminal_T * p_term = &p_mc->Shell.Terminal;
    const uint16_t duty = p_motor->Config.AlignPower_ScalarU16;

    if(argc == 2U)
    {
        if(argv[1U][0U] == 'a' && argv[1U][1U] == 'b') { Phase_Polar_ActivateAB(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'a' && argv[1U][1U] == 'c') { Phase_Polar_ActivateAC(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'a' && argv[1U][1U] == '\0') { Phase_Polar_ActivateA(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'a' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvA(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'b' && argv[1U][1U] == 'a') { Phase_Polar_ActivateBA(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'b' && argv[1U][1U] == 'c') { Phase_Polar_ActivateBC(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'b' && argv[1U][1U] == '\0') { Phase_Polar_ActivateB(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'b' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvB(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'c' && argv[1U][1U] == 'a') { Phase_Polar_ActivateCA(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'c' && argv[1U][1U] == 'b') { Phase_Polar_ActivateCB(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'c' && argv[1U][1U] == '\0') { Phase_Polar_ActivateC(&p_motor->Phase, duty); }
        else if(argv[1U][0U] == 'c' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvC(&p_motor->Phase, duty); }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_jog(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argv;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Cmd_Status_T status;
    if(argc == 1U)
    {
        Motor_Jog6(p_motor);
        status = CMD_STATUS_SUCCESS;
    }
    else if(argc == 2U)
    {
        //        Motor_Jog (p_motor, strtoul(argv[1U], 0U, 10));
        //        return CMD_STATUS_PROCESS_LOOP;
    }

    return status;
}

static Cmd_Status_T Cmd_rev(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argc;    (void)argv;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    p_motor->JogIndex = 0U;
    p_motor->Encoder.TotalD = 0U;
    Motor_ZeroSensor(p_motor);
    return CMD_STATUS_PROCESS_LOOP;
}

static Cmd_Status_T Cmd_rev_Proc(MotorController_T * p_mc)
{
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Cmd_Status_T status;

    if(p_motor->JogIndex < Motor_User_GetPolePairs(p_motor) * 6U + 1U)
    {
        if(p_motor->JogIndex == 0U) { Motor_ZeroSensor(p_motor); }
        // Terminal_SendNum(p_term, p_motor->JogIndex);
        // PrintPhase(p_term, p_motor, p_motor->JogIndex);
        Terminal_SendNum(p_term, p_motor->JogIndex); Terminal_SendString(p_term, " ");
        PrintSensor(p_term, p_motor);
        Motor_Jog6(p_motor);
        status = CMD_STATUS_SUCCESS;
    }
    else
    {
        Terminal_SendString(p_term, "\r\n");
        MotorController_User_ForceDisableControl(p_mc);
        status = CMD_STATUS_PROCESS_END;
    }

    return status;
}

static Cmd_Status_T Cmd_monitor(MotorController_T * p_mc, int argc, char ** argv)
{
    if(argc == 1U)
    {
        p_mc->ShellSubstate = 0U;
    }
    else if(argc == 2U)
    {
        if      (strncmp(argv[1U], "sensor", 6U) == 0U) { p_mc->ShellSubstate = 1U; }
        else if (strncmp(argv[1U], "power", 5U) == 0U)  { p_mc->ShellSubstate = 2U; }
        else if (strncmp(argv[1U], "bemf", 4U) == 0U)   { p_mc->ShellSubstate = 3U; }
        else if (strncmp(argv[1U], "exp", 3U) == 0U)    { p_mc->ShellSubstate = 4U; }
    }
    return CMD_STATUS_PROCESS_LOOP;
}


static Cmd_Status_T Cmd_monitor_Proc(MotorController_T * p_mc)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    switch(p_mc->ShellSubstate)
    {
        case 0U:
            Terminal_SendString(p_term, "Speed: "); Terminal_SendNum(p_term, Motor_User_GetSpeed_Rpm(p_motor)); Terminal_SendString(p_term, " RPM, ");
            Terminal_SendNum(p_term, Motor_User_GetSpeed_Frac16(p_motor)); Terminal_SendString(p_term, " Frac16\r\n");

            Terminal_SendString(p_term, "Encoder Speed: ");
            Terminal_SendNum(p_term, Encoder_ModeDT_GetRotationalVelocity_RPM(&p_motor->Encoder)); Terminal_SendString(p_term, " RPM, ");
            Terminal_SendNum(p_term, Encoder_ModeDT_GetScalarVelocity(&p_motor->Encoder)); Terminal_SendString(p_term, " Frac16\r\n");

            // Terminal_SendString(p_term, "SpeedControl: "); Terminal_SendNum(p_term, PID_GetOutput(&p_motor->PidSpeed)); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "IdReq: "); Terminal_SendNum(p_term, p_motor->Foc.ReqQ); Terminal_SendString(p_term, "\r\n");

            Terminal_SendString(p_term, "RampOut: "); Terminal_SendNum(p_term, Linear_Ramp_GetOutput(&p_motor->Ramp)); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "RampTarget: "); Terminal_SendNum(p_term, Linear_Ramp_GetTarget(&p_motor->Ramp)); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DeltaD: "); Terminal_SendNum(p_term, p_motor->Encoder.DeltaD); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DeltaT: "); Terminal_SendNum(p_term, p_motor->Encoder.DeltaT); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DeltaTh: "); Terminal_SendNum(p_term, p_motor->Encoder.DeltaTh); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "FreqD: "); Terminal_SendNum(p_term, p_motor->Encoder.FreqD); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "TotalD: "); Terminal_SendNum(p_term, p_motor->Encoder.TotalD); Terminal_SendString(p_term, "\r\n");

            Terminal_SendString(p_term, "Iq: "); Terminal_SendNum(p_term, p_motor->Foc.Iq); Terminal_SendString(p_term, ", Vq: "); Terminal_SendNum(p_term, p_motor->Foc.Vq); Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Id: "); Terminal_SendNum(p_term, p_motor->Foc.Id); Terminal_SendString(p_term, ", Vd: "); Terminal_SendNum(p_term, p_motor->Foc.Vd); Terminal_SendString(p_term, " Frac16\r\n");

            // Terminal_SendString(p_term, "Throttle: ");     Terminal_SendNum(p_term, MotAnalogUser_GetThrottle(&p_mc->AnalogUser)); Terminal_SendString(p_term, " Frac16\r\n");
            // Terminal_SendString(p_term, "Brake: ");     Terminal_SendNum(p_term, MotAnalogUser_GetBrake(&p_mc->AnalogUser)); Terminal_SendString(p_term, " Frac16\r\n");

            // Terminal_SendString(p_term, "AnalogUserCmd: ");
            // switch(p_mc->AnalogUser.Cmd)
            // {
            //     case MOT_ANALOG_USER_CMD_SET_BRAKE:                    Terminal_SendString(p_term, "brake");            break;
            //     case MOT_ANALOG_USER_CMD_SET_THROTTLE:                Terminal_SendString(p_term, "throttle");        break;
            //     // case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:        Terminal_SendString(p_term, "brake rel");        break;
            //     // case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:            Terminal_SendString(p_term, "throttle rel");    break;
            //     // case MOT_ANALOG_USER_CMD_SET_NEUTRAL:                Terminal_SendString(p_term, "set neutral");        break;
            //     case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:                Terminal_SendString(p_term, "neutral");            break;
            //     // case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD:     Terminal_SendString(p_term, "set forward");        break;
            //     // case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE:     Terminal_SendString(p_term, "set reverse");        break;
            //     case MOT_ANALOG_USER_CMD_PROC_ZERO:                Terminal_SendString(p_term, "release");            break;
            //     default: break;
            // }
            // Terminal_SendString(p_term, "\r\n");

            Terminal_SendString(p_term, "MSM: ");
            switch(Motor_User_GetStateId(p_motor))
            {
                case MSM_STATE_ID_INIT:         Terminal_SendString(p_term, "Init");        break;
                case MSM_STATE_ID_STOP:         Terminal_SendString(p_term, "Stop");        break;
                case MSM_STATE_ID_OPEN_LOOP:    Terminal_SendString(p_term, "OpenLoop");    break;
                case MSM_STATE_ID_RUN:          Terminal_SendString(p_term, "Run");         break;
                case MSM_STATE_ID_FREEWHEEL:    Terminal_SendString(p_term, "Freewheel");   break;
                case MSM_STATE_ID_CALIBRATION:  Terminal_SendString(p_term, "Calib");       break;
                case MSM_STATE_ID_FAULT:        Terminal_SendString(p_term, "Fault");       break;
                default: break;
            }
            Terminal_SendString(p_term, "\r\n");

            Terminal_SendString(p_term, "MCSM: ");
            switch(MotorController_User_GetStateId(p_mc))
            {
                case MCSM_STATE_ID_INIT:    Terminal_SendString(p_term, "Init");    break;
                case MCSM_STATE_ID_STOP:    Terminal_SendString(p_term, "Stop");    break;
                #ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
                case MCSM_STATE_ID_SERVO:   Terminal_SendString(p_term, "Servo");   break;
                #endif
                case MCSM_STATE_ID_RUN:     Terminal_SendString(p_term, "Run");     break;
                case MCSM_STATE_ID_FAULT:   Terminal_SendString(p_term, "Fault");   break;
                default: break;
            }
            Terminal_SendString(p_term, "\r\n");

            // Terminal_SendString(p_term, "ILimitActiveId: "); Terminal_SendNum(p_term, p_motor->ILimitActiveId); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "ILimitActive: "); Terminal_SendNum(p_term, p_motor->ILimitActiveSentinel_ScalarU16); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "ILimitMotoring_ScalarU16: "); Terminal_SendNum(p_term, p_motor->ILimitMotoring_ScalarU16); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "IsILimitActive: "); Terminal_SendNum(p_term, p_motor->StateFlags.ILimitActive); Terminal_SendString(p_term, "\r\n");

            // Terminal_SendString(p_term, "SpeedPid Limit: "); Terminal_SendNum(p_term, p_motor->PidSpeed.OutputMax); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "ElecAngle: "); Terminal_SendNum(p_term, Motor_User_GetElectricalAngle(p_motor)); Terminal_SendString(p_term, " Deg16\r\n");
            // Terminal_SendString(p_term, "MechAngle: "); Terminal_SendNum(p_term, Motor_User_GetMechanicalAngle(p_motor)); Terminal_SendString(p_term, " Deg16\r\n");

            break;
        case 1U:
            PrintSensor(p_term, p_motor);
            break;

        case 2U:
            Terminal_SendString(p_term, "Speed: "); Terminal_SendNum(p_term, Motor_User_GetSpeed_Rpm(p_motor)); Terminal_SendString(p_term, " RPM\r\n");

            Terminal_SendString(p_term, "Iq: "); Terminal_SendNum(p_term, p_motor->Foc.Iq); Terminal_SendString(p_term, ", Vq: "); Terminal_SendNum(p_term, p_motor->Foc.Vq); Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Id: "); Terminal_SendNum(p_term, p_motor->Foc.Id); Terminal_SendString(p_term, ", Vd: "); Terminal_SendNum(p_term, p_motor->Foc.Vd); Terminal_SendString(p_term, " Frac16\r\n");

            Terminal_SendString(p_term, "Ideal Power: ");
            Terminal_SendNum(p_term, Motor_User_GetElectricalPower_VA(p_motor));        Terminal_SendString(p_term, " Volt-Amps, ");
            Terminal_SendNum(p_term, Motor_User_GetElectricalPower_UFrac16(p_motor));   Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "VPhase: ");
            Terminal_SendNum(p_term, Motor_User_GetVPhase_Volts(p_motor));      Terminal_SendString(p_term, " Volts, ");
            Terminal_SendNum(p_term, Motor_User_GetVPhase_UFrac16(p_motor));    Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "IPhase: ");
            Terminal_SendNum(p_term, Motor_User_GetIPhase_Amps(p_motor));       Terminal_SendString(p_term, " Amps, ");
            Terminal_SendNum(p_term, Motor_User_GetIPhase_UFrac16(p_motor));    Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "IPeakAdcu: "); Terminal_SendNum(p_term, Linear_ADC_AdcuOfFrac16(&p_motor->UnitsIa, FOC_GetIMagnitude(&p_motor->Foc)) - p_motor->Config.IaZeroRef_Adcu); Terminal_SendString(p_term, " ADCU\r\n");
            // Terminal_SendString(p_term, "Capture: ");             Terminal_SendNum(p_term, p_motor->IPhasePeak_Adcu);             Terminal_SendString(p_term, " ADCU\r\n");
            // Terminal_SendString(p_term, "Ia: ");     Terminal_SendNum(p_term, p_motor->Foc.Ia); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "Ib: ");     Terminal_SendNum(p_term, p_motor->Foc.Ib); Terminal_SendString(p_term, "\r\n");
            // Terminal_SendString(p_term, "Ic: ");     Terminal_SendNum(p_term, p_motor->Foc.Ic); Terminal_SendString(p_term, "\r\n");
            break;

        case 3U:
            // Terminal_SendString(p_term, "Bemf Peak: ");      Terminal_SendNum(p_term, Motor_User_GetVPhase_UFrac16(p_motor)); Terminal_SendString(p_term, " Frac16\r\n");
            // Terminal_SendString(p_term, "Capture: ");        Terminal_SendNum(p_term, p_motor->VBemfPeak_Adcu); Terminal_SendString(p_term, " ADCU\r\n");
            // Terminal_SendString(p_term, "FOC: ");            Terminal_SendNum(p_term, Linear_ADC_AdcuOfFrac16(&p_motor->UnitsVa, FOC_GetIMagnitude(&p_motor->Foc)) - p_motor->Config.IaZeroRef_Adcu); Terminal_SendString(p_term, " ADCU\r\n");
            // Terminal_SendString(p_term, "FOC Clarke: ");     Terminal_SendNum(p_term, Linear_ADC_AdcuOfFrac16(&p_motor->UnitsVa, FOC_GetIMagnitude_Clarke(&p_motor->Foc)) - p_motor->Config.IaZeroRef_Adcu); Terminal_SendString(p_term, " ADCU\r\n");

        case 4U:
            // Terminal_SendString(p_term, "VectorMagnitude: "); Terminal_SendNum(p_term, VectorMagnitude); Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Throttle: ");  Terminal_SendNum(p_term, MotAnalogUser_GetThrottle(&p_mc->AnalogUser));     Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Brake: ");     Terminal_SendNum(p_term, MotAnalogUser_GetBrake(&p_mc->AnalogUser));        Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Speed: ");     Terminal_SendNum(p_term, Motor_User_GetSpeed_Rpm(p_motor)); Terminal_SendString(p_term, " RPM\r\n");
            Terminal_SendString(p_term, "Iq: "); Terminal_SendNum(p_term, p_motor->Foc.Iq); Terminal_SendString(p_term, ", Vq: "); Terminal_SendNum(p_term, p_motor->Foc.Vq); Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Id: "); Terminal_SendNum(p_term, p_motor->Foc.Id); Terminal_SendString(p_term, ", Vd: "); Terminal_SendNum(p_term, p_motor->Foc.Vd); Terminal_SendString(p_term, " Frac16\r\n");

            Terminal_SendString(p_term, "VPhase: ");
            Terminal_SendNum(p_term, Motor_User_GetVPhase_Volts(p_motor));      Terminal_SendString(p_term, " Volts, ");
            Terminal_SendNum(p_term, Motor_User_GetVPhase_UFrac16(p_motor));    Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "IPhase: ");
            Terminal_SendNum(p_term, Motor_User_GetIPhase_Amps(p_motor));       Terminal_SendString(p_term, " Amps, ");
            Terminal_SendNum(p_term, Motor_User_GetIPhase_UFrac16(p_motor));    Terminal_SendString(p_term, " Frac16\r\n");
            Terminal_SendString(p_term, "Ideal Power: ");
            Terminal_SendNum(p_term, Motor_User_GetElectricalPower_VA(p_motor));        Terminal_SendString(p_term, " Volt-Amps, ");
            Terminal_SendNum(p_term, Motor_User_GetElectricalPower_UFrac16(p_motor));   Terminal_SendString(p_term, " Frac16\r\n");

            break;

        default: break;
    }

    Terminal_SendString(p_term, "\r\n");

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_mode(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 1U) /* display modes */
    {
        Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Motor:\r\n");

        Terminal_SendString(p_term, "CommutationMode: ");
        switch(Motor_User_GetCommutationMode(p_motor))
        {
            case MOTOR_COMMUTATION_MODE_SIX_STEP:   Terminal_SendString(p_term, "SIX_STEP");    break;
            case MOTOR_COMMUTATION_MODE_FOC:        Terminal_SendString(p_term, "FOC");         break;
            default: break;
        }
        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "SensorMode: ");
        switch(Motor_User_GetSensorMode(p_motor))
        {
            case MOTOR_SENSOR_MODE_HALL:    Terminal_SendString(p_term, "HALL");    break;
            case MOTOR_SENSOR_MODE_ENCODER: Terminal_SendString(p_term, "ENCODER"); break;
            default: break;
        }
        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "FeedbackMode: ");
        switch(Motor_User_GetDefaultFeedbackMode(p_motor))
        {
            case MOTOR_FEEDBACK_MODE_OPEN_LOOP_SCALAR:                 Terminal_SendString(p_term, "OPEN_LOOP");       break;
            case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:          Terminal_SendString(p_term, "VOLTAGE");         break;
            case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:          Terminal_SendString(p_term, "CURRENT");         break;
            case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:    Terminal_SendString(p_term, "SPEED_VOLTAGE");   break;
            case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:    Terminal_SendString(p_term, "SPEED_CURRENT");   break;
            default: break;
        }
        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "DirectionForward: ");
        switch(Motor_User_GetDirectionCalibration(p_motor))
        {
            case MOTOR_FORWARD_IS_CW:   Terminal_SendString(p_term, "CW");  break;
            case MOTOR_FORWARD_IS_CCW:  Terminal_SendString(p_term, "CCW"); break;
            default: break;
        }
        Terminal_SendString(p_term, "\r\n");
    }
    else if(argc == 2U) /* sets mode */
    {
        if      (strncmp(argv[1U], "foc", 4U) == 0U)        { Motor_User_SetCommutationMode(p_motor, MOTOR_COMMUTATION_MODE_FOC); }
        else if (strncmp(argv[1U], "sixstep", 8U) == 0U)    { Motor_User_SetCommutationMode(p_motor, MOTOR_COMMUTATION_MODE_SIX_STEP); }
        else if (strncmp(argv[1U], "voltage", 8U) == 0U)    { Motor_User_SetDefaultFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE); }
        else if (strncmp(argv[1U], "current", 8U) == 0U)    { Motor_User_SetDefaultFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT); }
        else if (strncmp(argv[1U], "speedv", 7U) == 0U)     { Motor_User_SetDefaultFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE); }
        else if (strncmp(argv[1U], "speedi", 7U) == 0U)     { Motor_User_SetDefaultFeedbackMode(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT); }
        else if (strncmp(argv[1U], "fwdccw", 7U) == 0U)     { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW); }
        else if (strncmp(argv[1U], "fwdcw", 6U) == 0U)      { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW); }
        else if (strncmp(argv[1U], "protocol", 9U) == 0U)
        {
            Shell_DisableOnInit(&p_mc->Shell);
            Protocol_EnableOnInit(&p_mc->CONST.P_PROTOCOLS[0U]);
        }
        else if(strncmp(argv[1U], "brakevf", 7U) == 0U)
        {
            MotorController_User_SetBrakeMode(p_mc, MOTOR_CONTROLLER_BRAKE_MODE_VOLTAGE);
            Terminal_SendString(p_term, "VFreq Brake\r\n");
        }
        else if(strncmp(argv[1U], "brakei", 7U) == 0U)
        {
            MotorController_User_SetBrakeMode(p_mc, MOTOR_CONTROLLER_BRAKE_MODE_TORQUE);
            Terminal_SendString(p_term, "Torque Brake\r\n");
        }
    }

    return CMD_STATUS_SUCCESS;
}


static Cmd_Status_T Cmd_calibrate(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Cmd_Status_T status = CMD_STATUS_INVALID_ARGS;

    if(argc == 2U)
    {
        if      (strncmp(argv[1U], "encoder", 8U) == 0U)    { Motor_User_ActivateCalibrationEncoder(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
        else if (strncmp(argv[1U], "hall", 5U) == 0U)       { Motor_User_ActivateCalibrationHall(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
        else if (strncmp(argv[1U], "adc", 4U) == 0U)        { Motor_User_ActivateCalibrationAdc(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
    #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        else if (strncmp(argv[1U], "sincos", 7U) == 0U) { Motor_User_ActivateCalibrationSinCos(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
    #endif
    }

    return status;
}

//todo to string functions
static Cmd_Status_T Cmd_calibrate_Proc(MotorController_T * p_mc)
{
    Cmd_Status_T status = CMD_STATUS_PROCESS_LOOP;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(Motor_User_GetStateId(p_motor) == MSM_STATE_ID_STOP)
    {
        status = CMD_STATUS_PROCESS_END;

        if(p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_HALL)
        {
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[0U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[1U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[2U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[3U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[4U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[5U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[6U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendNum(p_term, p_motor->Hall.Config.SensorsTable[7U]); Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "Boundary Count "); Terminal_SendNum(p_term, p_motor->Hall.Config.BoundaryType); Terminal_SendString(p_term, "\r\n");
        }
    #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        else if(p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_SIN_COS)
        {
            Terminal_SendString(p_term, "AngleOffset: ");         Terminal_SendNum(p_term, p_motor->SinCos.Config.AngleOffet);         Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "IsCcwPositive: ");     Terminal_SendNum(p_term, p_motor->SinCos.Config.IsCcwPositive);     Terminal_SendString(p_term, "\r\n");

            Terminal_SendString(p_term, "DebugAPreMech: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugAPre);         Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DebugBPreMech: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugBPre);         Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DebugAPostMech: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugAPostMech);     Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DebugBPostMech: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugBPostMech);     Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DebugAPostElec: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugAPostElec);     Terminal_SendString(p_term, "\r\n");
            Terminal_SendString(p_term, "DebugBPostElec: ");     Terminal_SendNum(p_term, p_motor->SinCos.DebugBPostElec);     Terminal_SendString(p_term, "\r\n");

            //            Terminal_SendString(p_term, "Zero_Adcu: ");     Terminal_SendNum(p_term, p_motor->SinCos.Config.Zero_Adcu);     Terminal_SendString(p_term, "\r\n");
            //            Terminal_SendString(p_term, "Max_Adcu: ");         Terminal_SendNum(p_term, p_motor->SinCos.Config.Max_Adcu);         Terminal_SendString(p_term, "\r\n");
            //            Terminal_SendString(p_term, "Max_MilliV: ");     Terminal_SendNum(p_term, p_motor->SinCos.Config.Max_MilliV);     Terminal_SendString(p_term, "\r\n");
            //            Terminal_SendString(p_term, "Phase 0: "); Terminal_SendNum(p_term, p_motor->Debug[9U]); Terminal_SendString(p_term, "Deg16\r\n");
            //            Terminal_SendString(p_term, "Phase A: "); Terminal_SendNum(p_term, p_motor->Debug[0U]); Terminal_SendString(p_term, "Deg16\r\n");
            //            Terminal_SendString(p_term, "Sin: "); Terminal_SendNum(p_term, p_motor->Debug[3U]);
            //            Terminal_SendString(p_term, "Cos: "); Terminal_SendNum(p_term, p_motor->Debug[4U]);
            //            Terminal_SendString(p_term, "\r\n");
            //            Terminal_SendString(p_term, "Phase B: "); Terminal_SendNum(p_term, p_motor->Debug[1U]); Terminal_SendString(p_term, "Deg16\r\n");
            //            Terminal_SendString(p_term, "Sin: "); Terminal_SendNum(p_term, p_motor->Debug[5U]);
            //            Terminal_SendString(p_term, "Cos: "); Terminal_SendNum(p_term, p_motor->Debug[6U]);
            //            Terminal_SendString(p_term, "\r\n");
            //            Terminal_SendString(p_term, "Phase C: "); Terminal_SendNum(p_term, p_motor->Debug[2U]); Terminal_SendString(p_term, "Deg16\r\n");
            //            Terminal_SendString(p_term, "Sin: "); Terminal_SendNum(p_term, p_motor->Debug[7U]);
            //            Terminal_SendString(p_term, "Cos: "); Terminal_SendNum(p_term, p_motor->Debug[8U]);
            Terminal_SendString(p_term, "\r\n");

        }
    #endif
        Terminal_SendString(p_term, "\r\n");
    }

    return status;
}



static Cmd_Status_T Cmd_hall(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argv;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 1U)
    {
        Hall_CaptureSensors_ISR(&p_motor->Hall);
        Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Hall Sensors: "); Terminal_SendNum(p_term, Hall_GetSensorsValue(&p_motor->Hall)); Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Electrical Angle: "); Terminal_SendNum(p_term, Hall_GetAngle16(&p_motor->Hall)); Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "\r\n");
    }
    // else if(argc == 2U)
    // {
    //     if(strncmp(argv[1U], "table", 5U) == 0U) {} //tostroing
    // }

    return CMD_STATUS_SUCCESS;
}

void PrintThermistorLimit(Terminal_T * p_term, Thermistor_T * p_thermistor)
{
    Terminal_SendString(p_term, "Shutdown: ");
    Terminal_SendNum(p_term, Thermistor_GetFaultTrigger_Adcu(p_thermistor)); Terminal_SendString(p_term, " ADCU, ");
    Terminal_SendNum(p_term, Thermistor_GetFault_DegCScalar(p_thermistor, 1U)); Terminal_SendString(p_term, " C ");
    Terminal_SendString(p_term, " Threshold: ");
    Terminal_SendNum(p_term, Thermistor_GetFaultThreshold_Adcu(p_thermistor)); Terminal_SendString(p_term, " ADCU, ");
    Terminal_SendNum(p_term, Thermistor_GetFaultThreshold_DegCScalar(p_thermistor, 1U)); Terminal_SendString(p_term, " C\r\n");

    Terminal_SendString(p_term, "Warning: ");
    Terminal_SendNum(p_term, Thermistor_GetWarningTrigger_Adcu(p_thermistor)); Terminal_SendString(p_term, " ADCU, ");
    Terminal_SendNum(p_term, Thermistor_GetWarning_DegCScalar(p_thermistor, 1U)); Terminal_SendString(p_term, " C ");
    Terminal_SendString(p_term, " Threshold: ");
    Terminal_SendNum(p_term, Thermistor_GetWarningThreshold_Adcu(p_thermistor)); Terminal_SendString(p_term, " ADCU, ");
    Terminal_SendNum(p_term, Thermistor_GetWarningThreshold_DegCScalar(p_thermistor, 1U)); Terminal_SendString(p_term, " C\r\n");
}

//todo seperate print functions
static Cmd_Status_T Cmd_heat(MotorController_T * p_mc, int argc, char ** argv)
{
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

    if(argc == 1U)
    {
        Terminal_SendString(p_term, "PCB: ");
        Terminal_SendNum(p_term, MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_PCB)); Terminal_SendString(p_term, " ADCU, ");
        Terminal_SendNum(p_term, MotorController_User_GetHeatPcb_DegC(p_mc, 1U)); Terminal_SendString(p_term, " C\r\n");
    #if        defined(CONFIG_MOTOR_CONTROLLER_HEAT_MOSFETS_TOP_BOT_ENABLE)
        Terminal_SendString(p_term, "MOSFETs Top: ");
        Terminal_SendNum(p_term, MotorController_User_GetHeatMosfetsTop_DegC(p_mc, 1U));     Terminal_SendString(p_term, " C\r\n");
        Terminal_SendString(p_term, "MOSFETs Bot: ");
        Terminal_SendNum(p_term, MotorController_User_GetHeatMosfetsBot_DegC(p_mc, 1U));     Terminal_SendString(p_term, " C\r\n");
    #else
        Terminal_SendString(p_term, "MOSFETs: ");
        Terminal_SendNum(p_term, MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_HEAT_MOSFETS)); Terminal_SendString(p_term, " ADCU, ");
        Terminal_SendNum(p_term, MotorController_User_GetHeatMosfets_DegC(p_mc, 1U)); Terminal_SendString(p_term, " C\r\n");
    #endif
        Terminal_SendString(p_term, "Motor0: ");
        Terminal_SendNum(p_term, Motor_User_GetAdcu(p_motor, MOTOR_ANALOG_CHANNEL_HEAT)); Terminal_SendString(p_term, " ADCU, ");
        Terminal_SendNum(p_term, Motor_User_GetHeat_DegC(p_motor, 1U)); Terminal_SendString(p_term, " C\r\n");
    }
    else if(argc == 2U)
    {
        if(strncmp(argv[1U], "limits", 7U) == 0U)
        {
            Terminal_SendString(p_term, "PCB:\r\n"); PrintThermistorLimit(p_term, &p_mc->ThermistorPcb);

            Terminal_SendString(p_term, "MOSFETs:\r\n"); PrintThermistorLimit(p_term, &p_mc->ThermistorMosfets);

            Terminal_SendString(p_term, "Motor0:\r\n"); PrintThermistorLimit(p_term, &p_motor->Thermistor);
            Terminal_SendString(p_term, "\r\n");
        }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_auser(MotorController_T * p_mc, int argc, char ** argv)
{
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 1U)
    {
        Terminal_SendString(p_term, "Throttle: ");
        Terminal_SendNum(p_term, MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_THROTTLE));     Terminal_SendString(p_term, " Adcu ");
        Terminal_SendNum(p_term, MotAnalogUser_GetThrottle(&p_mc->AnalogUser));                     Terminal_SendString(p_term, " Frac16\r\n");
        Terminal_SendString(p_term, "Brake: ");
        Terminal_SendNum(p_term, MotorController_User_GetAdcu(p_mc, MOT_ANALOG_CHANNEL_BRAKE));     Terminal_SendString(p_term, " Adcu ");
        Terminal_SendNum(p_term, MotAnalogUser_GetBrake(&p_mc->AnalogUser));                         Terminal_SendString(p_term, " Frac16\r\n");
    }

    return CMD_STATUS_SUCCESS;
}

//MotorController_String.c
// static const char * STR_VSOURCE         = "V Supply:\r\n";
// static const char * STR_VSENSE         = "V Sensor:\r\n";
// static const char * STR_VACC         = "V Accessories:\r\n";

// static size_t MotorController_ToString_VMonitorsLimits(MotorController_T * p_mc, char * p_stringBuffer)
// {
//     char * p_stringDest = p_stringBuffer;

//     memcpy(p_stringDest, STR_VSOURCE, strlen(STR_VSOURCE));         p_stringDest += strlen(STR_VSOURCE);
//     p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorSource, p_stringDest, 1000U);
//     memcpy(p_stringDest, "\r\n", 2U);     p_stringDest += 2U;

//     memcpy(p_stringDest, STR_VSENSE, strlen(STR_VSENSE));     p_stringDest += strlen(STR_VSENSE);
//     p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorSense, p_stringDest, 1000U);
//     memcpy(p_stringDest, "\r\n", 2U);     p_stringDest += 2U;

//     memcpy(p_stringDest, STR_VACC, strlen(STR_VACC));         p_stringDest += strlen(STR_VACC);
//     p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorAccs, p_stringDest, 1000U);
//     memcpy(p_stringDest, "\r\n", 2U);     p_stringDest += 2U;

//     return p_stringDest - p_stringBuffer;
// }


#ifdef CONFIG_VMONITOR_STRING_FUNCTIONS_ENABLE
static const char * STR_LIMIT = "Limit: ";
static const char * STR_WARNING = "Warning: ";
static const char * STR_UPPER = "Upper: ";
static const char * STR_LOWER = "Lower: ";

/*
    Limit: Upper: [Num] Lower: [Num] Warning: Upper: [Num] Lower: [Num]
*/
size_t VMonitor_ToString_Verbose(VMonitor_T * p_vMonitor, char * p_stringBuffer, uint16_t unitVScalar)
{
    char * p_stringDest = p_stringBuffer;
    int32_t num;
    char numStr[16U];

    memcpy(p_stringDest, STR_LIMIT, strlen(STR_LIMIT)); p_stringDest += strlen(STR_LIMIT);

    memcpy(p_stringDest, STR_UPPER, strlen(STR_UPPER)); p_stringDest += strlen(STR_UPPER);
    num = Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.FaultUpper_Adcu, unitVScalar);
    snprintf(numStr, 16U, "%d", (int)num);
    memcpy(p_stringDest, numStr, strlen(numStr)); p_stringDest += strlen(numStr);
    *p_stringDest = ' '; p_stringDest++;

    memcpy(p_stringDest, STR_LOWER, strlen(STR_LOWER)); p_stringDest += strlen(STR_LOWER);
    num = Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.FaultLower_Adcu, unitVScalar);
    snprintf(numStr, 16U, "%d", (int)num);
    memcpy(p_stringDest, numStr, strlen(numStr)); p_stringDest += strlen(numStr);
    *p_stringDest = ' '; p_stringDest++;

    memcpy(p_stringDest, STR_WARNING, strlen(STR_WARNING)); p_stringDest += strlen(STR_WARNING);

    memcpy(p_stringDest, STR_UPPER, strlen(STR_UPPER)); p_stringDest += strlen(STR_UPPER);
    num = Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.WarningUpper_Adcu, unitVScalar);
    snprintf(numStr, 16U, "%d", (int)num);
    memcpy(p_stringDest, numStr, strlen(numStr)); p_stringDest += strlen(numStr);
    *p_stringDest = ' '; p_stringDest++;

    memcpy(p_stringDest, STR_LOWER, strlen(STR_LOWER)); p_stringDest += strlen(STR_LOWER);
    num = Linear_Voltage_ScalarV(&p_vMonitor->Units, p_vMonitor->Config.WarningLower_Adcu, unitVScalar);
    snprintf(numStr, 16U, "%d", (int)num);
    memcpy(p_stringDest, numStr, strlen(numStr)); p_stringDest += strlen(numStr);
    *p_stringDest = ' '; p_stringDest++;

    return p_stringDest - p_stringBuffer;
}
#endif

static Cmd_Status_T Cmd_v(MotorController_T * p_mc, int argc, char ** argv)
{
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    // char * p_txString;
    // uint8_t txSize = 0U;
    int32_t vSense, vAcc, vSource, battery;

    if(argc == 1U)
    {
        vSource = MotorController_User_GetVSource_V(p_mc, 1000U);
        vSense = MotorController_User_GetVSense_V(p_mc, 1000U);
        vAcc = MotorController_User_GetVAccs_V(p_mc, 1000U);
        battery = MotorController_User_GetBatteryCharge_Unit1000(p_mc);
        Terminal_SendString(p_term, "VSource: "); Terminal_SendNum(p_term, vSource); Terminal_SendString(p_term, " mV\r\n");
        Terminal_SendString(p_term, "VSense: "); Terminal_SendNum(p_term, vSense); Terminal_SendString(p_term, " mV\r\n");
        Terminal_SendString(p_term, "VAcc: "); Terminal_SendNum(p_term, vAcc); Terminal_SendString(p_term, " mV\r\n");
        Terminal_SendString(p_term, "Battery: "); Terminal_SendNum(p_term, battery); Terminal_SendString(p_term, " 1000th\r\n");
        Terminal_SendString(p_term, "\r\n");
    }
    else if(argc == 2U)
    {
        if(strncmp(argv[1U], "limits", 7U) == 0U)
        {
            //     p_txString = (char *)Terminal_AcquireTxBuffer(p_term);
            //     txSize = MotorController_ToString_VMonitorsLimits(p_mc, p_txString);
            //     Terminal_ReleaseTxBuffer(p_term, txSize);
        }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_fault(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argv;
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 1U)
    {
        MotorController_User_ToggleUserFault(p_mc);

        Terminal_SendString(p_term, "FaultFlags [VSource][VAcc][VSense][Pcb][MosTop][MosBot]: ");
        Terminal_SendNum(p_term, p_mc->FaultFlags.VSourceLimit);
        Terminal_SendNum(p_term, p_mc->FaultFlags.VAccsLimit);
        Terminal_SendNum(p_term, p_mc->FaultFlags.VSenseLimit);
        Terminal_SendNum(p_term, p_mc->FaultFlags.PcbOverheat);
        Terminal_SendNum(p_term, p_mc->FaultFlags.MosfetsOverheat);
        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Fault ADCU:\r\n");
        Terminal_SendString(p_term, "VSource: ");         Terminal_SendNum(p_term, p_mc->FaultAnalogRecord.VSource_Adcu);         Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "VAcc: ");             Terminal_SendNum(p_term, p_mc->FaultAnalogRecord.VAccs_Adcu);             Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "VSense: ");         Terminal_SendNum(p_term, p_mc->FaultAnalogRecord.VSense_Adcu);             Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Pcb: ");             Terminal_SendNum(p_term, p_mc->FaultAnalogRecord.HeatPcb_Adcu);         Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Mosfets: ");         Terminal_SendNum(p_term, p_mc->FaultAnalogRecord.HeatMosfets_Adcu);     Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Fault DegC:\r\n");
        Terminal_SendString(p_term, "Pcb: ");         Terminal_SendNum(p_term, MotorController_User_GetFaultHeatPcb_DegC(p_mc, 1U));     Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "Mosfets: ");     Terminal_SendNum(p_term, MotorController_User_GetFaultHeatMosfets_DegC(p_mc, 1U));     Terminal_SendString(p_term, "\r\n");
        Terminal_SendString(p_term, "\r\n");
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_direction(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

    if(argc == 2U)
    {
        if(strncmp(argv[1U], "fwd", 2U) == 0U) { MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD); }
        else if(strncmp(argv[1U], "rev", 2U) == 0U) { MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE); }
    }
    if(argc == 3U)
    {
        if(strncmp(argv[1U], "calfwd", 4U) == 0U)
        {
            if(strncmp(argv[2U], "ccw", 4U) == 0U) { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW); }
            else if(strncmp(argv[2U], "cw", 3U) == 0U) { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW); }
        }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_set(MotorController_T * p_mc, int argc, char ** argv)
{
    //    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 2U)
    {
        if(strncmp(argv[1U], "default", 8U) == 0U)
        {
            MotorController_User_SetLoadDefault(p_mc, true);
            MotorController_User_SaveBootReg_Blocking(p_mc);
            Terminal_SendString(p_term, "Need to reboot\r\n");
        }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_save(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argc;
    (void)argv;
    MotorController_User_SaveConfig_Blocking(p_mc);
    return CMD_STATUS_SUCCESS;
}

extern void SystemSoftwareReset(void); //todo abstraction layer

static Cmd_Status_T Cmd_reboot(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)p_mc;
    (void)argc;
    (void)argv;
    // System_Reboot();
    // SystemSoftwareReset();
    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_beep(MotorController_T * p_mc, int argc, char ** argv)
{
    uint8_t count;

    if(argc == 1U)
    {
        MotorController_User_BeepN(p_mc, 500U, 500U, 1U);
    }
    else if(argc == 2U)
    {
        if(strncmp(argv[1U], "stop", 5U) == 0U) { MotorController_User_BeepStop(p_mc); }
        else if(strncmp(argv[1U], "start", 6U) == 0U) { MotorController_User_BeepStart(p_mc, 500U, 500U); }
        else
        {
            count = strtoul(argv[1U], 0U, 10);
            MotorController_User_BeepN(p_mc, 500U, 500U, count);
        }
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_kvspeed(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    uint16_t rpm;

    if(argc == 2U)
    {
        rpm = strtoul(argv[1U], 0U, 10);
        Motor_User_SetVSpeedRef_Rpm(p_motor, rpm);
        rpm = Motor_User_GetSpeedVRef_Rpm(p_motor);

        Terminal_SendString(p_term, "\r\nSpeedV_Rpm: "); Terminal_SendNum(p_term, rpm);
        Terminal_SendString(p_term, " PWM: "); Terminal_SendNum(p_term, Motor_User_ConvertToVSpeed(p_motor, rpm));
        Terminal_SendString(p_term, " Frac16\r\n");

        Terminal_SendString(p_term, "RPM: "); Terminal_SendNum(p_term, rpm * 3U / 4U);
        Terminal_SendString(p_term, " PWM: "); Terminal_SendNum(p_term, Motor_User_ConvertToVSpeed(p_motor, rpm * 3U / 4U));
        Terminal_SendString(p_term, " Frac16\r\n");

        Terminal_SendString(p_term, "RPM: "); Terminal_SendNum(p_term, rpm / 2U);
        Terminal_SendString(p_term, " PWM: "); Terminal_SendNum(p_term, Motor_User_ConvertToVSpeed(p_motor, rpm / 2U));
        Terminal_SendString(p_term, " Frac16\r\n");

        Terminal_SendString(p_term, "RPM: "); Terminal_SendNum(p_term, rpm * 1U / 4U);
        Terminal_SendString(p_term, " PWM: "); Terminal_SendNum(p_term, Motor_User_ConvertToVSpeed(p_motor, rpm * 1U / 4U));
        Terminal_SendString(p_term, " Frac16\r\n");
    }

    return CMD_STATUS_SUCCESS;
}

static void PrintIPeak(Terminal_T * p_term, uint16_t min_Adcu, int32_t min_Frac16, uint16_t max_Adcu, int32_t max_Frac16)
{
    Terminal_SendString(p_term, "Min: ");
    Terminal_SendNum(p_term, min_Adcu); Terminal_SendString(p_term, " ADCU ");
    Terminal_SendNum(p_term, min_Frac16); Terminal_SendString(p_term, " Frac16 ");
    Terminal_SendString(p_term, "Max: ");
    Terminal_SendNum(p_term, max_Adcu); Terminal_SendString(p_term, " ADCU ");
    Terminal_SendNum(p_term, max_Frac16); Terminal_SendString(p_term, " Frac16\r\n");
}

#if    defined(CONFIG_MOTOR_CONTROLLER_DEBUG_ENABLE)
static Cmd_Status_T Cmd_ipeak(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    uint16_t zeroToPeak_Adcu;
    uint16_t min_Adcu;
    uint16_t max_Adcu;

    if(argc == 2U)
    {
        zeroToPeak_Adcu = strtoul(argv[1U], 0U, 10);
        Motor_User_SetIPeakRef_Adcu_Debug(p_motor, zeroToPeak_Adcu);

        Terminal_SendString(p_term, "Phase A:\r\n");
        min_Adcu = p_motor->Config.IaZeroRef_Adcu - p_motor->Config.IPeakRef_Adcu;
        max_Adcu = p_motor->Config.IaZeroRef_Adcu + p_motor->Config.IPeakRef_Adcu;
        PrintIPeak(p_term, min_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIa, min_Adcu), max_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIa, max_Adcu));

        Terminal_SendString(p_term, "Phase B:\r\n");
        min_Adcu = p_motor->Config.IbZeroRef_Adcu - p_motor->Config.IPeakRef_Adcu;
        max_Adcu = p_motor->Config.IbZeroRef_Adcu + p_motor->Config.IPeakRef_Adcu;
        PrintIPeak(p_term, min_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIb, min_Adcu), max_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIb, max_Adcu));

        Terminal_SendString(p_term, "Phase C:\r\n");
        min_Adcu = p_motor->Config.IcZeroRef_Adcu - p_motor->Config.IPeakRef_Adcu;
        max_Adcu = p_motor->Config.IcZeroRef_Adcu + p_motor->Config.IPeakRef_Adcu;
        PrintIPeak(p_term, min_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIc, min_Adcu), max_Adcu, Linear_ADC_Frac16(&p_motor->UnitsIc, max_Adcu));

        Terminal_SendString(p_term, "\r\n");
    }

    return CMD_STATUS_SUCCESS;
}
#endif

static Cmd_Status_T Cmd_ilimit(MotorController_T * p_mc, int argc, char ** argv)
{
    Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
    Terminal_T * p_term = &p_mc->Shell.Terminal;
    uint16_t ilimit_Frac16;

    if(argc == 2U)
    {
        ilimit_Frac16 = strtoul(argv[1U], 0U, 10);
        Motor_User_SetILimitParam_Scalar16(p_motor, ilimit_Frac16, ilimit_Frac16);
        Terminal_SendString(p_term, "ILimitMotoringParam: "); Terminal_SendNum(p_term, p_motor->Config.ILimitMotoring_ScalarU16);
        Terminal_SendString(p_term, " ILimitMotoringActive: "); Terminal_SendNum(p_term, p_motor->ILimitMotoring_ScalarU16);
        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "PidSpeed Min, Max: ");
        Terminal_SendNum(p_term, p_motor->PidSpeed.OutputMin);        Terminal_SendString(p_term, " ");
        Terminal_SendNum(p_term, p_motor->PidSpeed.OutputMax);        Terminal_SendString(p_term, "\r\n");

        Terminal_SendString(p_term, "PidIq Min, Max: ");
        Terminal_SendNum(p_term, p_motor->PidIq.OutputMin);        Terminal_SendString(p_term, " ");
        Terminal_SendNum(p_term, p_motor->PidIq.OutputMax);        Terminal_SendString(p_term, "\r\n");
    }

    return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_version(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argc;
    (void)argv;
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    Terminal_SendString(p_term, "Library: ");
    Terminal_SendNum(p_term, MotorController_User_GetLibraryVersionIndex(3U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetLibraryVersionIndex(2U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetLibraryVersionIndex(1U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetLibraryVersionIndex(0U)); Terminal_SendString(p_term, "\r\n");

    Terminal_SendString(p_term, "Firmware: ");
    Terminal_SendNum(p_term, MotorController_User_GetMainVersionIndex(p_mc, 3U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetMainVersionIndex(p_mc, 2U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetMainVersionIndex(p_mc, 1U)); Terminal_SendString(p_term, " ");
    Terminal_SendNum(p_term, MotorController_User_GetMainVersionIndex(p_mc, 0U)); Terminal_SendString(p_term, "\r\n");

    Terminal_SendString(p_term, "Manufacture: ");
    Terminal_SendString(p_term, "\r\n");

    return CMD_STATUS_SUCCESS;
}


static Cmd_Status_T Cmd_servo(MotorController_T * p_mc, int argc, char ** argv)
{
#if    defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE)
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    if(argc == 1U)
    {
        MotorController_User_EnterServoMode(p_mc);
        if(MotorController_User_GetStateId(p_mc) == MCSM_STATE_ID_SERVO) { Terminal_SendString(p_term, "Entered Servo Mode\r\n"); }
        else { Terminal_SendString(p_term, "Failed\r\n"); }
    }
    else if(argc == 2U)
    {
        if(strncmp(argv[1U], "exit", 5U) == 0U)
        {
            MotorController_User_ExitServoMode(p_mc);
            if(MotorController_User_GetStateId(p_mc) != MCSM_STATE_ID_SERVO) { Terminal_SendString(p_term, "Exited Servo Mode\r\n"); }
            else { Terminal_SendString(p_term, "Failed\r\n"); }
        }
    }
#endif
    return CMD_STATUS_SUCCESS;
}


static Cmd_Status_T Cmd_debug(MotorController_T * p_mc, int argc, char ** argv)
{
    (void)argc;
    (void)argv;
    Terminal_T * p_term = &p_mc->Shell.Terminal;

    // const MotorController_Manufacture_T TEST =
    // {
    //     .NAME = {'q', 'b', 'c', 'd'},
    //     .MANUFACTURE_NUMBER_WORD = 123U,
    // };

    // const MotorController_Manufacture_T TEST1 =
    // {
    //     .NAME = {'x', 'y', 'z', 'z'},
    //     .MANUFACTURE_NUMBER_WORD = 9877U,
    // };

    // MotorController_Manufacture_T TestRead = {0};

    // Terminal_SendString(p_term, "Manufacture:\r\n");
    // MotorController_User_GetManufacture(p_mc, &TestRead);
    // Terminal_SendString_Len(p_term, TestRead.NAME, 8U); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, TestRead.MANUFACTURE_NUMBER_WORD); Terminal_SendString(p_term, "\r\n");

    // Terminal_SendString(p_term, "write1:\r\n");
    // MotorController_User_WriteManufacture(p_mc, &TEST);
    // Terminal_SendNum(p_term, p_mc->NvmStatus);Terminal_SendString(p_term, "\r\n");
    // MotorController_ReadOnce_Blocking(p_mc);
    // MotorController_User_GetManufacture(p_mc, &TestRead);
    // Terminal_SendString_Len(p_term, TestRead.NAME, 8U); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, TestRead.MANUFACTURE_NUMBER_WORD); Terminal_SendString(p_term, "\r\n");

    // Terminal_SendString(p_term, "write2:\r\n");
    // MotorController_User_WriteManufacture(p_mc, &TEST1);
    // Terminal_SendNum(p_term, p_mc->NvmStatus);Terminal_SendString(p_term, "\r\n");
    // MotorController_ReadOnce_Blocking(p_mc);
    // MotorController_User_GetManufacture(p_mc, &TestRead);
    // Terminal_SendString_Len(p_term, TestRead.NAME, 8U); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, TestRead.MANUFACTURE_NUMBER_WORD); Terminal_SendString(p_term, "\r\n");

    // qfrac16_t atan2_0 = qfrac16_atan2(0, 32767);
    // qfrac16_t atan2_90 = qfrac16_atan2(32767, 0);
    // qfrac16_t atan2_180 = qfrac16_atan2(0, -32767);
    // qfrac16_t atan2_270 = qfrac16_atan2(-32767, 0);
    // qfrac16_t atan2_err = qfrac16_atan2(32767, 32767);

    // Terminal_SendString(p_term, "Atan2: "); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, atan2_0); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, atan2_90); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, atan2_180); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, atan2_270); Terminal_SendString(p_term, "\r\n");
    // Terminal_SendNum(p_term, atan2_err); Terminal_SendString(p_term, "\r\n");

        // Terminal_SendString(p_term, "PwmOn "); Terminal_SendNum(p_term, sampleCount); Terminal_SendString(p_term, " :\r\n");
        // for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
        // {
        //     Terminal_SendNum(p_term, p_samples[iSample]); Terminal_SendString(p_term, ", ");
        // }
        // Terminal_SendString(p_term, "\r\n");

        // Terminal_SendString(p_term, "PwmOff "); Terminal_SendNum(p_term, sampleCount); Terminal_SendString(p_term, " :\r\n");
        // for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
        // {
        //     Terminal_SendNum(p_term, p_samples[iSample]); Terminal_SendString(p_term, ", ");
        // }
        // Terminal_SendString(p_term, "\r\n");

    Terminal_SendString(p_term, "\r\n");
    return CMD_STATUS_SUCCESS;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT] =
{
    {"monitor",     "Display motor info",                (Cmd_Function_T)Cmd_monitor,     {.FUNCTION = (Cmd_ProcessFunction_T)Cmd_monitor_Proc,         .PERIOD = 2000U } },
    {"heat",        "Display temperatures",                (Cmd_Function_T)Cmd_heat,         {0U}    },
    {"v",           "Display voltages",                 (Cmd_Function_T)Cmd_v,             {0U}    },
    {"auser",       "Display analog user",                 (Cmd_Function_T)Cmd_auser,         {0U}    },
    {"hall",         "Read hall",                         (Cmd_Function_T)Cmd_hall,         {0U}    },
    {"fault",         "Display fault info",                 (Cmd_Function_T)Cmd_fault,         {0U}    },
    {"mode",         "Sets mode using options",            (Cmd_Function_T)Cmd_mode,         {0U}    },

    {"direction",     "Sets direction",                    (Cmd_Function_T)Cmd_direction,    {0U}    },
    {"calibrate",     "calibrate",                        (Cmd_Function_T)Cmd_calibrate,     {.FUNCTION = (Cmd_ProcessFunction_T)Cmd_calibrate_Proc,     .PERIOD = 500U } },

    {"set",         "Sets motor parameters",             (Cmd_Function_T)Cmd_set,         {0U}    },
    {"save",         "Save parameters to nv memory",     (Cmd_Function_T)Cmd_save,         {0U}    },
    {"reboot",         "reboot",                             (Cmd_Function_T)Cmd_reboot,     {0U}    },

    {"run",         "Set motor to run mode",             (Cmd_Function_T)Cmd_run,         {0U}    },
    {"stop",         "Set motor to freewheel mode",         (Cmd_Function_T)Cmd_stop,         {0U}    },
    {"phase",         "Sets motor phase",                 (Cmd_Function_T)Cmd_phase,         {0U}    },
    {"jog",         "Jog motor",                         (Cmd_Function_T)Cmd_jog,         {0U}    },
    {"rev",         "Rev motor",                         (Cmd_Function_T)Cmd_rev,         {.FUNCTION = (Cmd_ProcessFunction_T)Cmd_rev_Proc,             .PERIOD = 250U } },
    {"servo",         "servo mode",                         (Cmd_Function_T)Cmd_servo,         {0U}    },
    {"beep",         "beep",                                (Cmd_Function_T)Cmd_beep,         {0U}    },
    {"speedv",         "speedv",                            (Cmd_Function_T)Cmd_kvspeed,     {0U}    },

    {"ipeak",         "ipeak",                            (Cmd_Function_T)Cmd_ipeak,         {0U}    },
    {"ilimit",         "ilimit",                            (Cmd_Function_T)Cmd_ilimit,     {0U}    },

    {"version",     "version",                            (Cmd_Function_T)Cmd_version,     {0U}    },
    {"debug",         "print debug info",                    (Cmd_Function_T)Cmd_debug,         {0U}    },

};

#endif

//static Cmd_Status_T Cmd_configfile(int argc, char **argv)
//{
//
//    return MC_SHELL_CMD_RETURN_CODE_SUCCESS;
//}
