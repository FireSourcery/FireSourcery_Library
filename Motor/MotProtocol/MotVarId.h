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
    @file     MotProtocol.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_VAR_ID_H
#define MOT_VAR_ID_H

typedef enum MotVarId_Tag
{
    /*
        Partition0 - 0-255 0xFF
    */
    MOT_VAR_NULL = 0U,
    MOT_VAR_SPEED = 1U,           /* FracS16 may over saturate */
    MOT_VAR_CURRENT = 2U,         /* FracS16 may over saturate */
    MOT_VAR_VOLTAGE = 3U,         /* FracS16 may over saturate */
    MOT_VAR_POWER = 4U,           /* FracS16 may over saturate */
    MOT_VAR_FOC_IQ = 17U,
    MOT_VAR_FOC_ID = 18U,
    MOT_VAR_DEBUG = 128U,
    MOT_VAR_MILLIS = 129U,

    MOT_VAR_TX_PACKET_COUNT = 130U,
    MOT_VAR_RX_PACKET_COUNT = 131U,

    /*
        Demo Group 0-255
    */
    MOT_VAR_THROTTLE = 101U,          /* Value 16-bit */
    MOT_VAR_BRAKE = 102U,             /* Value 16-bit */
    MOT_VAR_DIRECTION = 103U,         /* Value 0: Neutral, 1: Reverse, 2: Forward */
    MOT_VAR_BEEP = 104U,              /* Beep */
    MOT_VAR_USER_CMD = 105U,

    MOT_VAR_SPEED_RPM = 200U,       /* Value 16-bit */
    MOT_VAR_ERROR_CODE = 201U,      /* Value Bitfield 16-bit */
    MOT_VAR_MC_STATE = 202U,        /* Value enum: 0:INIT, 1:STOP ,2:RUN 3:FAULT*/


    // MOT_VAR_HEAT_PCB_DEG_C = 133U,            /* Value 16-bit */
    // MOT_VAR_FOC_IQ = 134U,                    /* Value 16-bit */
    // MOT_VAR_I_PEAK_AMP = 130U,                 /* Value 16-bit */
    // MOT_VAR_SPEED_GROUND_KMH = 131U,        /* Value 16-bit */

    MOT_VAR_PARAM_TEST_BEGIN = 256U,    /*  */
    MOT_VAR_USER_INPUT_MODE = 257U,     /* Value enum: 0:Analog, 1:Protocol */
    MOT_VAR_PARAM_TEST_1 = 258U,        /* Value 16-bit */
    MOT_VAR_PARAM_TEST_2 = 259U,        /* Value 32-bit */
    MOT_VAR_PARAM_TEST_3 = 260U,        /* Value 0, 1 */
    MOT_VAR_PARAM_TEST_4 = 261U,        /* Value 16-bit Read-Only */

    /*
        Partition2 0x100:0x1FF
    */


    /*
        Real Time Monitor Vars
    */


    /*
        Parameter Vars
    */

    /*
        Motor[0] by default
    */
    MOT_VAR_I_MAX_REF_AMP,

    MOT_VAR_POLE_PAIRS,
    MOT_VAR_SPEED_FEEDBACK_REF_RPM,
    MOT_VAR_SPEED_VMATCH_REF_RPM,
    MOT_VAR_I_REF_PEAK_ADCU,
    MOT_VAR_IA_REF_ZERO_ADCU,
    MOT_VAR_IB_REF_ZERO_ADCU,
    MOT_VAR_IC_REF_ZERO_ADCU,

    MOT_VAR_SPEED_LIMIT_CCW_FRAC16,
    MOT_VAR_SPEED_LIMIT_CCW_RPM,

    MOT_VAR_SPEED_LIMIT_CW_FRAC16,
    MOT_VAR_I_LIMIT_MOTORING_FRAC16,
    MOT_VAR_I_LIMIT_GENERATING_FRAC16,
    MOT_VAR_I_LIMIT_SCALAR_HEAT_FRAC16,
    MOT_VAR_DIRECTION_CALIBRATION,
    MOT_VAR_COMMUTATION_MODE,
    MOT_VAR_SENSOR_MODE,
    MOT_VAR_FEEDBACK_MODE,
    MOT_VAR_ALIGN_MODE,
    MOT_VAR_ALIGN_VOLTAGE_FRAC16,
    MOT_VAR_ALIGN_TIME_CONTROLCYCLES,
    MOT_VAR_PHASE_PWM_MODE,

    // MOT_VAR_OPENLOOPVPWMMIN,
    // MOT_VAR_OPENLOOPVPWMMAX,
    // MOT_VAR_OPENLOOPSPEEDSTART,
    // MOT_VAR_OPENLOOPSPEEDFINAL,
    // MOT_VAR_OPENLOOPACCEL ,


    // MOT_VAR_COUNTSPERREVOLUTION         ,
    // MOT_VAR_DISTANCEPERCOUNT             ,
    // MOT_VAR_ISQUADRATURECAPTUREENABLED ,
    // MOT_VAR_ISALEADBPOSITIVE             ,
    // MOT_VAR_EXTENDEDTIMERDELTATSTOP     ,
    // OT_VAR_HALL[0U],

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
    // ,
    // OT_VAR_SIN_COS[0U],

    // // MAX,> 4MOT_VAR_2V [3440 ADCU], MIN, 1MOT_VAR_9V [1556 ADCU],  ZERO, 2498 ADCU
    // MOT_VAR_ZERO_ADCU     , 2360U, //2360
    // MOT_VAR_MAX_ADCU     , 3120U, //3120
    // MOT_VAR_MAX_MILLIV , 4200U,
    // MOT_VAR_ANGLEOFFET , 0,
    // MOT_VAR_ISBPOSITIVE , TRUE,
    // MOT_VAR_ELECTRICALROTATIONSPERCYCLE, 4U, //, POLEPAIRS/CYCLESPERROTATION
    // ,
    // *
    // * INPUT SPEED Q0MOT_VAR_16
    // * OUTPUT SPEEDCONTROL,> VPWM, VQ, IQ,
    // */
    // OT_VAR_PID_SPEED[0U],

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
    // OT_VAR_PID_FOC_IQ[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 1,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 1,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    // OT_VAR_PID_FOC_ID[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 1,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 2,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    // OT_VAR_PID_SIX_STEP_IBUS[0U],

    // MOT_VAR_CALCFREQ, 20000U,
    // MOT_VAR_MODE, PID_MODE_PI,
    // MOT_VAR_KPFACTOR, 1,
    // MOT_VAR_KPDIVISOR, 2,
    // MOT_VAR_KIFACTOR, 1,
    // MOT_VAR_KIDIVISOR, 2,
    // MOT_VAR_KDFACTOR, 0,
    // MOT_VAR_KDDIVISOR, 0,
    // ,
    // OT_VAR_THERMISTOR_MOTORS[0U],

    // MOT_VAR_RNOMINAL, 100000U,
    // MOT_VAR_TNOMINAL, 298U,
    // MOT_VAR_BCONSTANT, 3950U,
    // MOT_VAR_SHUTDOWN_ADCU, 0,
    // MOT_VAR_THRESHOLD_ADCU, 0,
    // MOT_VAR_CAPTURESCALAR, 1U,
    // MOT_VAR_ISENABLEONINIT, FALSE,
    // ,
    // OT_VAR_THERMISTOR_PCB,

    // MOT_VAR_RNOMINAL, 100000U,
    // MOT_VAR_TNOMINAL, 298U,
    // MOT_VAR_BCONSTANT, 4250U,
    // MOT_VAR_SHUTDOWN_ADCU, 219U,     //APPROX 100C
    // MOT_VAR_THRESHOLD_ADCU, 294U, //APPROX 90C
    // MOT_VAR_CAPTURESCALAR, 1U,
    // MOT_VAR_ISENABLEONINIT, TRUE,

    // OT_VAR_THERMISTOR_MOSFETS_TOP,

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
    // OT_VAR_ANALOG_USER,

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
    // OT_VAR_SHELL,

    // MOT_VAR_XCVRID    ,
    // MOT_VAR_BAUDRATE ,
    // MOT_VAR_ISENABLEONINIT ,
    // ,

    MOT_VAR_RESERVED_65535 = 0xFFFFU,
}
MotVarId_T;

typedef enum MotVarSize_Tag
{
    MOT_VAR_SIZE_16 = 0U,
    MOT_VAR_SIZE_32 = 1U,
}
MotVarSize_T;

#endif