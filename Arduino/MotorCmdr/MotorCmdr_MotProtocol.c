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
    @file       MotPacket.c
    @author     FireSourcery
    @version     V0
    @brief    App Protocol Packet Interface. Parse Packet to App structure
*/
/******************************************************************************/
#include "MotorCmdr.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*!    Ping */
/******************************************************************************/
static void Ping_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_PingResp_T * p_rxPacket, size_t rxSize)
{
    (void)p_txPacket; (void)p_txSize; (void)rxSize;
    // p_app->RespStatus =
    MotPacket_PingResp_Parse(p_rxPacket, &p_app->Version[0U]);
}

/******************************************************************************/
/*!    Stop */
/******************************************************************************/
static void StopAll_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_StopResp_T * p_rxPacket, size_t rxSize)
{
    (void)p_txPacket; (void)p_txSize; (void)rxSize;
    // p_app->RespStatus = MotPacket_StopResp_Parse(p_rxPacket);
}

/******************************************************************************/
/*!    Write Var */
/******************************************************************************/
static void WriteVar_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_WriteVarResp_T * p_rxPacket, size_t rxSize)
{
    (void)p_txPacket; (void)p_txSize; (void)rxSize;
    p_app->RespStatus = p_rxPacket->Header.Status;
}

/******************************************************************************/
/*!    Read Var */
/******************************************************************************/
static void ReadVar_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_ReadVarResp_T * p_rxPacket, size_t rxSize)
{
    (void)p_txPacket; (void)p_txSize; (void)rxSize;

    p_app->RespStatus = MotPacket_ReadVarResp_Parse(p_rxPacket, &p_app->MotorReadVarValue);

    // switch(p_app->MotorReadVarId)
    // {
    //     case MOT_DRIVE_VAR_THROTTLE:        p_app->Throttle break;                    /* Value 16-bit */
    //     case MOT_DRIVE_VAR_BRAKE:              break;                    /* Value 16-bit */
    //     case MOT_DRIVE_VAR_DIRECTION:          break;                /* Value 0: Neutral, 1: Reverse, 2: Forward */

    //     case MOT_VAR_SPEED_RPM:             break;
    //     case MOT_VAR_ERROR_CODE:         break;
    //     case MOT_VAR_MC_STATE:              break;
    //     // case MOT_VAR_I_PEAK_AMP:        Motor_User_GetIPhase_Amps(MotorController_User_GetPtrMotor(p_mc, 0U));     break;
    //     // case MOT_VAR_SPEED_GROUND_KMH:    Motor_User_GetSpeed_Rpm(MotorController_User_GetPtrMotor(p_mc, 0U));     break;
    //     // case MOT_VAR_HEAT_PCB_DEG_C:    MotorController_User_GetHeatPcb_DegC(p_mc, 1U);             break;
    //     // case MOTOR_VAR_FOC_IQ:            MotorController_User_GetPtrMotor(p_mc, 0U)->Foc.Iq;         break;

    //     case MOT_VAR_PARAM_TEST_BEGIN:    break;                    /*  */
    //     case MOT_VAR_PARAM_TEST_1:        break;                        /* Value 16-bit */
    //     case MOT_VAR_PARAM_TEST_2:        break;                        /* Value 32-bit */
    //     case MOT_VAR_PARAM_TEST_3:        break;                        /* Value 0, 1 */
    //     case MOT_VAR_PARAM_TEST_4:        break;                        /* Value enum: 0:White, 1:Black, 2:Red, */
    //     // case MOT_VAR_PARAM_TEST_5:    value = p_mc->Config.Test[4U];    break;                        /*   */

    //     case MOTOR_VAR_POLE_PAIRS:                        break;
    //     case MOTOR_VAR_SPEED_V_REF_RPM:    break;
    //     case MOT_VAR_I_MAX_REF_AMP:           break;
    //     default: break;
    // }
}

/******************************************************************************/
/*!    Save NvMemory */
/******************************************************************************/
static void SaveNvm_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_SaveNvmResp_T * p_rxPacket, size_t rxSize)
{
    (void)p_txPacket; (void)p_txSize; (void)rxSize;
    p_app->RespStatus = p_rxPacket->Header.Status;
}

/******************************************************************************/
/*!    Control Monitor */
/******************************************************************************/
// static void Control_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_ControlResp_T * p_rxPacket, size_t rxSize)
// {
//     (void)p_txPacket; (void)p_txSize; (void)rxSize;
//     //may need to check correctness comparing ControlIdActive, control resp does not contain control id

//     // p_app->RespStatus = p_rxPacket->Header.Status;
// }

// static void Monitor_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_MonitorResp_T * p_rxPacket, size_t rxSize)
// {
//     (void)p_txPacket; (void)p_txSize; (void)rxSize;
//     // p_app->RespStatus =
//     switch(p_app->MonitorIdActive)
//     {
//         case MOT_PACKET_MONITOR_SPEED: MotPacket_MonitorResp_Speed_Parse(&p_app->Speed, p_rxPacket);         break;
//         case MOT_PACKET_MONITOR_I_FOC:
//             MotPacket_MonitorResp_IFoc_Parse(&p_app->Ia, &p_app->Ib, &p_app->Ic, &p_app->Ialpha, &p_app->Ibeta, &p_app->Id, &p_app->Iq, p_rxPacket);
//             break;
//         default: break;
//     }
// }

/******************************************************************************/
/*!    Init Units */
/******************************************************************************/
// static void InitUnits_ParseResp(MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const MotPacket_InitUnitsResp_T * p_rxPacket, size_t rxSize)
// {
//     (void)p_txPacket; (void)p_txSize; (void)rxSize;
//     // p_app->RespStatus =
//     MotPacket_InitUnitsResp_Parse
//     (
//         &p_app->Units.SpeedFeedbackRef_Rpm, &p_app->Units.IMaxRef_Amp, &p_app->Units.VSupplyRef_Volts,
//         &p_app->Units.VSupply_R1, &p_app->Units.VSupply_R2,
//         &p_app->Units.VSense_R1, &p_app->Units.VSense_R2,
//         &p_app->Units.VAcc_R1, &p_app->Units.VAcc_R2,
//         p_rxPacket
//     );
// }

//todo stateful req
// static void InitUnits_ProcReqRespExt(MotProtocol_DataModeState_T * p_substate, MotorCmdr_T * p_app, uint8_t * p_txPacket, size_t * p_txSize, const uint8_t * p_rxPacket, size_t rxSize)
// {
//      (void)rxSize;
//     switch(p_substate->StateId)
//     {
//         case 0U:
//             MotPacket_InitUnitsResp_Parse
//             (
//                 &p_app->Units.SpeedFeedbackRef_Rpm, &p_app->Units.IMaxRef_Amp, &p_app->Units.VSupplyRef_Volts,
//                 &p_app->Units.VSupply_R1, &p_app->Units.VSupply_R2,
//                 &p_app->Units.VSense_R1, &p_app->Units.VSense_R2,
//                 &p_app->Units.VAcc_R1, &p_app->Units.VAcc_R2,
//                 (const MotPacket_InitUnitsResp_T *)p_rxPacket
//             );
//             break;
//         default:
//             break;
//     }
//     // p_app->RespStatus =
// }

/*

*/
static const Protocol_Req_T REQ_TABLE[] =
{
    PROTOCOL_REQ_DEFINE(MOT_PACKET_PING,                 Ping_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),
    PROTOCOL_REQ_DEFINE(MOT_PACKET_STOP_ALL,             StopAll_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),  //todo sync set wait for ack, retx if resp is nack
    PROTOCOL_REQ_DEFINE(MOT_PACKET_READ_VAR,             ReadVar_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),
    PROTOCOL_REQ_DEFINE(MOT_PACKET_WRITE_VAR,             WriteVar_ParseResp,     0U,     PROTOCOL_SYNC_ID_DISABLE),
    PROTOCOL_REQ_DEFINE(MOT_PACKET_SAVE_CONFIG,             SaveNvm_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_INIT_UNITS,             InitUnits_ParseResp,     0U,     PROTOCOL_SYNC_ID_DISABLE),
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_INIT_UNITS,             0U,     InitUnits_ProcReqRespExt,     PROTOCOL_SYNC_ID_DISABLE), /todo statful
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_CONTROL_TYPE,         Control_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_MONITOR_TYPE,         Monitor_ParseResp,         0U,     PROTOCOL_SYNC_ID_DISABLE),
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_DATA_MODE_WRITE,         0U,             WriteData,     PROTOCOL_SYNC_ID ),
    // PROTOCOL_REQ_DEFINE(MOT_PACKET_DATA_MODE_READ,         0U,             ReadData,     PROTOCOL_SYNC_ID ),
};

/******************************************************************************/
/*! BuildTxReq */
/******************************************************************************/
void Cmdr_BuildTxReq(uint8_t * p_txPacket, size_t * p_txLength, const MotorCmdr_T * p_app, packet_id_t reqId)
{
    switch(reqId)
    {
        case MOT_PACKET_PING:                 *p_txLength = MotPacket_PingReq_Build((MotPacket_PingReq_T *)p_txPacket);             break;
        case MOT_PACKET_STOP_ALL:             *p_txLength = MotPacket_StopReq_Build((MotPacket_StopReq_T *)p_txPacket);             break;
        case MOT_PACKET_READ_VAR:              *p_txLength = MotPacket_ReadVarReq_Build((MotPacket_ReadVarReq_T *)p_txPacket, p_app->MotorReadVarId); break;
        case MOT_PACKET_WRITE_VAR:             *p_txLength = MotPacket_WriteVarReq_Build((MotPacket_WriteVarReq_T *)p_txPacket, p_app->MotorWriteVarId, p_app->MotorWriteVarValue); break;
        case MOT_PACKET_SAVE_CONFIG:             *p_txLength = MotPacket_SaveNvmReq_Build((MotPacket_SaveNvmReq_T *)p_txPacket);        break;

        // case MOT_PACKET_INIT_UNITS:         *p_txLength = MotPacket_InitUnitsReq_Build((MotPacket_InitUnitsReq_T *)p_txPacket);        break;
        case MOT_PACKET_REBOOT:                         break;
        case MOT_PACKET_CALL:                             break;
        // case MOT_PACKET_READ_VAR16:                     break;
        // case MOT_PACKET_WRITE_VAR16:                     break;
        // case MOT_PACKET_READ_MEMORY:                     break;
        // case MOT_PACKET_WRITE_MEMORY:                     break;
        case MOT_PACKET_DATA_MODE_READ:                 break;
        case MOT_PACKET_DATA_MODE_WRITE:                 break;
        case MOT_PACKET_DATA_MODE_DATA:                 break;

        // case MOT_PACKET_MONITOR_TYPE:             *p_txLength = MotPacket_MonitorReq_Build((MotPacket_MonitorReq_T *)p_txPacket, p_app->MonitorIdActive);        break;
        // case MOT_PACKET_CONTROL_TYPE:
        //     switch(p_app->ControlIdActive)
        //     {
        //         case MOT_PACKET_CONTROL_THROTTLE:            *p_txLength = MotPacket_ControlReq_Throttle_Build((MotPacket_ControlReq_T *)p_txPacket, p_app->MotorCmdValue);         break;
        //         case MOT_PACKET_CONTROL_BRAKE:                 *p_txLength = MotPacket_ControlReq_Brake_Build((MotPacket_ControlReq_T *)p_txPacket, p_app->MotorCmdValue);         break;
        //         case MOT_PACKET_CONTROL_RELEASE:             *p_txLength = MotPacket_ControlReq_Release_Build((MotPacket_ControlReq_T *)p_txPacket);                             break;
        //         case MOT_PACKET_CONTROL_DIRECTION_FORWARD:     *p_txLength = MotPacket_ControlReq_DirectionForward_Build((MotPacket_ControlReq_T *)p_txPacket);                     break;
        //         case MOT_PACKET_CONTROL_DIRECTION_REVERSE:     *p_txLength = MotPacket_ControlReq_DirectionReverse_Build((MotPacket_ControlReq_T *)p_txPacket);                     break;
        //         case MOT_PACKET_CONTROL_DIRECTION_NEUTRAL:     *p_txLength = MotPacket_ControlReq_DirectionNeutral_Build((MotPacket_ControlReq_T *)p_txPacket);                     break;
        //         default: break;
        //     }
        //     break;
        case MOT_PACKET_EXT_CMD:                             break;
        // case MOT_PACKET_EXT_RSVR2:                             break;
        case MOT_PACKET_ID_RESERVED_255:                     break;
        default: break;
    }
}

const Packet_Class_T MOTOR_CMDR_MOT_PROTOCOL_SPECS =
{
    .RX_LENGTH_MIN         = MOT_PACKET_LENGTH_MIN,
    .RX_LENGTH_MAX         = MOT_PACKET_LENGTH_MAX,
    .PARSE_RX_META         = (Packet_ParseRxMeta_T)MotProtocol_ParseRxMeta,
    .BUILD_TX_SYNC         = (Packet_BuildTxSync_T)MotProtocol_BuildTxSync,

    .P_REQ_TABLE         = &REQ_TABLE[0U],
    .REQ_TABLE_LENGTH     = sizeof(REQ_TABLE)/sizeof(Protocol_Req_T),
    .REQ_EXT_RESET         = (Protocol_ResetReqState_T)MotProtocol_ResetSubState,
    .CMDR_BUILD_TX_REQ     = (Protocol_Cmdr_BuildTxReq_T)Cmdr_BuildTxReq,

    .RX_START_ID         = MOT_PACKET_START_BYTE,
    .RX_END_ID             = 0x00U,

    .BAUD_RATE_DEFAULT     = MOT_PROTOCOL_BAUD_RATE_DEFAULT,
    .RX_TIMEOUT         = MOT_PROTOCOL_TIMEOUT_MS,
    //.RX_TIMEOUT_BYTE =  ,
    .REQ_TIMEOUT        = MOT_PROTOCOL_TIMEOUT_MS,
    .ENCODED             = false,
};


