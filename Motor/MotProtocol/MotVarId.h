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
    @file   MotVarId.h
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#ifndef MOT_VAR_ID_H
#define MOT_VAR_ID_H

typedef enum MotVarId_Tag
{
    /*
        Partition0 - 0-255 0X0-0xFF
        Read Only
    */
    MOT_VAR_NULL = 0U,
    MOT_VAR_SPEED = 1U,             /* FracU16, where  */
    MOT_VAR_I_PHASE = 2U,           /* FracS16 may over saturate */
    MOT_VAR_V_PHASE = 3U,           /* FracS16 may over saturate */
    MOT_VAR_POWER = 4U,             /* FracS16 may over saturate */
    MOT_VAR_MOTOR_STATE = 5U,       /* Value enum:  */
    MOT_VAR_RAMP_CMD = 6U,

    MOT_VAR_FOC_IQ = 16U,
    MOT_VAR_FOC_ID = 17U,
    MOT_VAR_FOC_Q_REQ = 18U,
    MOT_VAR_FOC_D_REQ = 19U,
    MOT_VAR_FOC_IA = 20U,
    MOT_VAR_FOC_IB = 21U,
    MOT_VAR_FOC_IC = 22U,
    MOT_VAR_FOC_VQ = 23U,
    MOT_VAR_FOC_VD = 24U,

    MOT_VAR_MC_STATE            = 32U,  /* Value enum: 0:INIT, 1:STOP, 2:RUN, 3:FAULT */
    MOT_VAR_MC_STATUS_FLAGS     = 33U,
    // MOT_VAR_ERROR_FLAGS = 34U,      /* Value Bitfield 16-bit */
    MOT_VAR_V_SOURCE            = 40U,  /* Base as ADCU */
    MOT_VAR_V_SENSOR            = 41U,
    MOT_VAR_V_ACC               = 42U,
    MOT_VAR_HEAT_PCB            = 43U,
    MOT_VAR_HEAT_MOSFETS        = 44U,
    MOT_VAR_HEAT_PCB_DEG_C      = 45U,
    MOT_VAR_HEAT_MOSFETS_DEG_C  = 46U,


    MOT_VAR_DEBUG = 128U,
    MOT_VAR_MILLIS = 129U,

    // MOT_VAR_MC_MOTOR0_STATE = 203U,      /*   */
    MOT_VAR_ANALOG_THROTTLE = 130U,         /* Value U16 */
    MOT_VAR_ANALOG_BRAKE = 131U,            /* Value U16 */

    MOT_VAR_TX_PACKET_COUNT = 254U,
    MOT_VAR_RX_PACKET_COUNT = 255U,

    /*
        Partition1 256-511 0x100-0x1FF
        Read Write
    */
    MOT_VAR_USER_CMD = 256U,
    MOT_VAR_THROTTLE = 257U,          /* Value U16 */
    MOT_VAR_BRAKE = 258U,             /* Value U16 */
    MOT_VAR_DIRECTION = 259U,         /* Value enum: 0:Neutral, 1:Reverse, 2:Forward */
    // MOT_VAR_DIRECTION_FORWARD = 0x01U,        /* Main direction */
    // MOT_VAR_DIRECTION_REVERSE = 0x02U,        /* Main direction */
    // MOT_VAR_DIRECTION_NEUTRAL = 0x03U,        /* Main direction */
    MOT_VAR_ACTIVE_FEEDBACK_MODE = 260U,

    /*
        Partition2 256-512 0x200-0x2FF
        Write Only
    */
    MOT_VAR_BEEP = 512U,                /* Beep */
    MOT_VAR_CALIBRATE_SENSOR = 513U,    /* Value enum: 0:Adc, 1:Hall, 2:Encoder */

    // Set Cmd and Feedback Mode
    // MOT_VAR_CMD_VOLTAGE = 0x13U,         /* Value [-32768:32767] */
    // MOT_VAR_CMD_CURRENT = 0x14U,         /* Value [-32768:32767] */
    // MOT_VAR_CMD_SPEED = 0x15U,           /* Value [-32768:32767] */
    // MOT_VAR_CMD_ANGLE = 0x16U,           /*   */


    /*
        Partition4 1024  0x400-0x4FF
        Parameter Vars Read Write
    */
    /*
        Motor[0] by default
    */
    MOT_VAR_DEFAULT_FEEDBACK_MODE = 1024U,
    MOT_VAR_HALL_SENSOR_TABLE_1 = 1025U,
    MOT_VAR_HALL_SENSOR_TABLE_2 = 1026U,
    MOT_VAR_HALL_SENSOR_TABLE_3 = 1027U,
    MOT_VAR_HALL_SENSOR_TABLE_4 = 1028U,
    MOT_VAR_HALL_SENSOR_TABLE_5 = 1029U,
    MOT_VAR_HALL_SENSOR_TABLE_6 = 1030U,
    // MOT_VAR_USER_INPUT_MODE =  U,     /* Value enum: 0:Analog, 1:Protocol */
    // MOT_VAR_POLE_PAIRS,
    // MOT_VAR_SPEED_FEEDBACK_REF_RPM,
    // MOT_VAR_SPEED_VMATCH_REF_RPM,
    // MOT_VAR_I_REF_PEAK_ADCU,
    // MOT_VAR_IA_REF_ZERO_ADCU,
    // MOT_VAR_IB_REF_ZERO_ADCU,
    // MOT_VAR_IC_REF_ZERO_ADCU,

    // MOT_VAR_SPEED_LIMIT_CCW_FRAC16,
    // MOT_VAR_SPEED_LIMIT_CW_FRAC16,
    // MOT_VAR_I_LIMIT_MOTORING_FRAC16,
    // MOT_VAR_I_LIMIT_GENERATING_FRAC16,
    // MOT_VAR_I_LIMIT_SCALAR_HEAT_FRAC16,
    // MOT_VAR_DIRECTION_CALIBRATION,
    // MOT_VAR_COMMUTATION_MODE,
    // MOT_VAR_SENSOR_MODE,
    // MOT_VAR_FEEDBACK_MODE,
    // MOT_VAR_ALIGN_MODE,
    // MOT_VAR_ALIGN_VOLTAGE_FRAC16,
    // MOT_VAR_ALIGN_TIME_CONTROLCYCLES,
    // MOT_VAR_PHASE_PWM_MODE,

    // MOT_VAR_OPENLOOPVPWMMIN,
    // MOT_VAR_OPENLOOPVPWMMAX,
    // MOT_VAR_OPENLOOPSPEEDSTART,
    // MOT_VAR_OPENLOOPSPEEDFINAL,
    // MOT_VAR_OPENLOOPACCEL ,


    // MOT_VAR_COUNTS_PER_REVOLUTION         ,
    // MOT_VAR_IS_QUADRATURE_CAPTURE_ENABLED ,
    // MOT_VAR_IS_A_LEAD_B_POSITIVE             ,
    // MOT_VAR_EXTENDED_TIMER_DELTA_T_STOP     ,
    // MOT_VAR_HALL[0U],

    // MOT_VAR_SENSORSTABLE,
    // {
    //     [0U], 0U,
    //     [1U], HALL_ANGLE_90_150,
    //     [2U], HALL_ANGLE_330_30,
    //     [3U], HALL_ANGLE_30_90,
    //     [4U], HALL_ANGLE_210_270,
    //     [5U], HALL_ANGLE_150_210,
    //     [6U], HALL_ANGLE_270_330,
    //     [7U], 0U,
    // },
    // MOT_VAR_SIN_COS[0U],

    // // MAX,> 4MOT_VAR_2V [3440 ADCU], MIN, 1MOT_VAR_9V [1556 ADCU],  ZERO, 2498 ADCU
    // MOT_VAR_ZERO_ADCU     , 2360U, //2360
    // MOT_VAR_MAX_ADCU     , 3120U, //3120
    // MOT_VAR_MAX_MILLIV , 4200U,
    // MOT_VAR_ANGLE_OFFET , 0,
    // MOT_VAR_IS_B_POSITIVE , TRUE,
    // MOT_VAR_ELECTRICAL_ROTATIONS_PER_CYCLE, 4U, //, POLEPAIRS/CYCLESPERROTATION
    // ,
    // *
    // * INPUT SPEED Q0MOT_VAR_16
    // * OUTPUT SPEEDCONTROL,> VPWM, VQ, IQ,
    // */
    // MOT_VAR_PID_SPEED[0U],

    // MOT_VAR_CALCFREQ, 1000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 1,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 2,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    // *
    // * INPUT  IQ
    // * OUTPUT VQ, SIGN INDICATES DIRECTION
    // */
    // MOT_VAR_PID_FOC_IQ[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 1,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 1,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    //MOT_VAR_PID_FOC_ID[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 1,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 2,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    //MOT_VAR_PID_SIX_STEP_IBUS[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 2,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 2,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    //MOT_VAR_THERMISTOR_MOTORS[0U],

    // MOT_VAR_RNOMINAL, 100000U,
    // MOT_VAR_TNOMINAL, 298U,
    // MOT_VAR_BCONSTANT, 3950U,
    // MOT_VAR_SHUTDOWN_ADCU, 0,
    // MOT_VAR_THRESHOLD_ADCU, 0,
    // MOT_VAR_CAPTURESCALAR, 1U,
    // MOT_VAR_ISENABLEONINIT, FALSE,
    // ,
    //MOT_VAR_THERMISTOR_PCB,

    // MOT_VAR_RNOMINAL, 100000U,
    // MOT_VAR_TNOMINAL, 298U,
    // MOT_VAR_BCONSTANT, 4250U,
    // MOT_VAR_SHUTDOWN_ADCU, 219U,     //APPROX 100C
    // MOT_VAR_THRESHOLD_ADCU, 294U, //APPROX 90C
    // MOT_VAR_CAPTURESCALAR, 1U,
    // MOT_VAR_ISENABLEONINIT, TRUE,

    //MOT_VAR_THERMISTOR_MOSFETS_TOP,

    // MOT_VAR_RNOMINAL,
    // MOT_VAR_TNOMINAL,
    // MOT_VAR_BCONSTANT,
    // MOT_VAR_SHUTDOWN_ADCU,
    // MOT_VAR_THRESHOLD_ADCU,
    // MOT_VAR_CAPTURESCALAR,
    // MOT_VAR_ISENABLEONINIT,


    // MOT_VAR_RNOMINAL,
    // MOT_VAR_TNOMINAL,
    // MOT_VAR_BCONSTANT,
    // MOT_VAR_SHUTDOWN_ADCU,
    // MOT_VAR_THRESHOLD_ADCU,
    // MOT_VAR_CAPTURESCALAR,
    // MOT_VAR_ISENABLEONINIT,


    // MOT_VAR_LIMITUPPER_ADCU,
    // MOT_VAR_LIMITLOWER_ADCU,
    // MOT_VAR_ISENABLEONINIT,

    // MOT_VAR_LIMITUPPER_ADCU,
    // MOT_VAR_LIMITLOWER_ADCU,
    // MOT_VAR_ISENABLEONINIT,


    // MOT_VAR_LIMITUPPER_ADCU,
    // MOT_VAR_LIMITLOWER_ADCU,
    // MOT_VAR_ISENABLEONINIT,


    // MOT_VAR_ADCVREF_MILLIV,
    // MOT_VAR_VSUPPLY,
    // MOT_VAR_INPUTMODE                 ,
    // MOT_VAR_COASTMODE                 ,
    // MOT_VAR_BATTERYZERO_ADCU,
    // MOT_VAR_BATTERYFULL_ADCU,
    // MOT_VAR_CANSERVICESID,
    // MOT_VAR_ISCANENABLE,
    //MOT_VAR_ANALOG_USER,

    // MOT_VAR_THROTTLEZERO_ADCU ,
    // MOT_VAR_THROTTLEMAX_ADCU ,
    // MOT_VAR_BRAKEZERO_ADCU    ,
    // MOT_VAR_BRAKEMAX_ADCU    ,
    // MOT_VAR_USETHROTTLEEDGEPIN ,
    // MOT_VAR_USEBRAKEEDGEPIN    ,
    // MOT_VAR_USENEUTRALPIN     ,
    // MOT_VAR_BISTATEBRAKEVALUE_FRAC16,
    // MOT_VAR_USEBISTATEBRAKE,

    // MOT_VAR_XCVRID    ,
    // MOT_VAR_SPECSID,
    // MOT_VAR_ISENABLEONINIT ,
    //         MOT_VAR_XCVRID    ,
    //         MOT_VAR_SPECSID,
    //         MOT_VAR_ISENABLEONINIT ,
    //MOT_VAR_SHELL,

    // MOT_VAR_XCVRID    ,
    // MOT_VAR_BAUDRATE ,
    // MOT_VAR_ISENABLEONINIT ,
    /*
        Partition5 2048  0x500-0x5FF
        Parameter Vars Read Only
    */
    MOT_VAR_VERSION = 2048U,
    MOT_VAR_I_MAX_REF_AMP,

    MOT_VAR_RESERVED_65535 = 0xFFFFU,
}
MotVarId_T;

// typedef enum MotVarSize_Tag
// {
//     MOT_VAR_SIZE_16 = 0U,
//     MOT_VAR_SIZE_32 = 1U,
// }
// MotVarSize_T;

// uint8_t MotVarId_SizeOf(MotVarId_T varId)
// {

// }

#endif