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
    @file   MotorController_MotProtocol.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_Var.h"
#include "Motor/MotorController/MotorController_User.h"


/******************************************************************************/
/*!
    Notes
    Directly use MotorController_T as Protocol interface (P_APP_INTERFACE)
    avoids double buffering
*/
/******************************************************************************/

/******************************************************************************/
/*!
    Request Response
*/
/******************************************************************************/
/******************************************************************************/
/*! Ping */
/******************************************************************************/
static protocol_txsize_t Ping(MotorControllerPtr_T p_mc, MotPacket_PingResp_T * p_txPacket, const MotPacket_PingReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_User_BeepN(p_mc, 500U, 500U, 1U);
    return MotPacket_PingResp_Build(p_txPacket);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
static protocol_txsize_t Version(MotorControllerPtr_T p_mc, MotPacket_VersionResp_T * p_txPacket, const MotPacket_PingReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    return MotPacket_VersionResp_Build(p_txPacket, MotorController_User_GetLibraryVersion(), MotorController_User_GetMainVersion(p_mc), MotorController_User_GetBoardVersion());
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static protocol_txsize_t StopAll(MotorControllerPtr_T p_mc, MotPacket_StopResp_T * p_txPacket, const MotPacket_StopReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_User_DisableControl(p_mc);
    return MotPacket_StopResp_Build(p_txPacket, 0U);
}

/******************************************************************************/
/*! Call - Blocking  */
/******************************************************************************/
typedef enum MotCallId
{
    MOT_CALL_ENTER_BLOCKING,
    MOT_CALL_EXIT_BLOCKING,
    MOT_CALL_CALIBRATE_SENSOR,
    MOT_CALL_SAVE_PARAMS,
    MOT_CALL_READ_MANUFACTURE,
    MOT_CALL_WRITE_MANUFACTURE,
}
MotCallId_T;

static protocol_txsize_t Call_Blocking(MotorControllerPtr_T p_mc, MotPacket_CallResp_T * p_txPacket, const MotPacket_CallReq_T * p_rxPacket)
{
    MotorPtr_T p_motor = MotorController_GetPtrMotor(p_mc, 0U);
    uint16_t status = 0U;

    switch((MotCallId_T)p_rxPacket->CallReq.Id)
    {
        case MOT_CALL_ENTER_BLOCKING:       MotorController_User_ProcBlocking_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_ENTER);              status = 0;                 break;
        case MOT_CALL_EXIT_BLOCKING:        MotorController_User_ProcBlocking_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_EXIT);               status = 0;                 break;
        case MOT_CALL_CALIBRATE_SENSOR:     MotorController_User_ProcBlocking_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_CALIBRATE_SENSOR);   status = 0U;                break;
        case MOT_CALL_SAVE_PARAMS:          MotorController_User_ProcBlocking_Blocking(p_mc, MOTOR_CONTROLLER_BLOCKING_NVM_SAVE_PARAMS);    status = p_mc->NvmStatus;   break;
        // case MOT_CALL_WRITE_MANUFACTURE: MotorController_User_ProcBlocking_Blocking(p_mc, MOT_CALL_WRITE_MANUFACTURE);       status = 0U;                    break;
        // case MOT_CALL_READ_MANUFACTURE:  MotorController_User_ProcBlocking_Blocking(p_mc, MOT_CALL_READ_MANUFACTURE);        status = 0U;                    break;
        default: break;
    }

    return MotPacket_CallResp_Build(p_txPacket, p_rxPacket->CallReq.Id, status);
}

/******************************************************************************/
/*! Read Vars */
/******************************************************************************/
/* Resp Truncates 32-Bit Vars */
static protocol_txsize_t VarRead(MotorControllerPtr_T p_mc, MotPacket_VarReadResp_T * p_txPacket, const MotPacket_VarReadReq_T * p_rxPacket)
{
    volatile uint8_t varsCount = MotPacket_VarReadReq_ParseVarIdCount(p_rxPacket);
    uint16_t idCheckSum = p_rxPacket->VarReadReq.IdChecksum;
    for(uint8_t index = 0U; index < varsCount; index++)
    {
        MotPacket_VarReadResp_BuildVarValue(p_txPacket, index, (uint16_t)MotorController_Var_Get(p_mc, (MotVarId_T)MotPacket_VarReadReq_ParseVarId(p_rxPacket, index)));
    }
    MotPacket_VarReadResp_BuildInnerHeader(p_txPacket, idCheckSum, MOT_VAR_STATUS_OK);
    return MotPacket_VarReadResp_BuildHeader(p_txPacket, varsCount);

}

/******************************************************************************/
/*! Write Vars */
/******************************************************************************/
static protocol_txsize_t VarWrite(MotorControllerPtr_T p_mc, MotPacket_VarWriteResp_T * p_txPacket, const MotPacket_VarWriteReq_T * p_rxPacket)
{
    uint8_t varsCount = MotPacket_VarWriteReq_ParseVarCount(p_rxPacket);
    uint16_t idCheckSum = p_rxPacket->VarWriteReq.IdChecksum;
    MotVarId_Status_T headerStatus = MOT_VAR_STATUS_OK;
    MotVarId_Status_T varStatus;
    for(uint8_t index = 0U; index < varsCount; index++)
    {
        varStatus = MotorController_Var_Set(p_mc, (MotVarId_T)MotPacket_VarWriteReq_ParseVarId(p_rxPacket, index), MotPacket_VarWriteReq_ParseVarValue(p_rxPacket, index));
        MotPacket_VarWriteResp_BuildVarStatus(p_txPacket, index, varStatus);
        if(varStatus != MOT_VAR_STATUS_OK) { headerStatus = MOT_VAR_STATUS_ERROR; }
    }
    MotPacket_VarWriteResp_BuildInnerHeader(p_txPacket, idCheckSum, headerStatus);
    return MotPacket_VarWriteResp_BuildHeader(p_txPacket, varsCount);
}

    /*
        Once Manufacturer
    */
    // MOT_VAR_NAME_0,
    // MOT_VAR_NAME_1,
    // MOT_VAR_NAME_2,
    // MOT_VAR_NAME_3,
    // MOT_VAR_NAME_4,
    // MOT_VAR_NAME_5,
    // MOT_VAR_NAME_6,
    // MOT_VAR_NAME_7,
    // MOT_VAR_NAME_REG32_0,
    // MOT_VAR_NAME_REG32_1,
    // MOT_VAR_SERIAL_NUMBER_0,
    // MOT_VAR_SERIAL_NUMBER_1,
    // MOT_VAR_SERIAL_NUMBER_2,
    // MOT_VAR_SERIAL_NUMBER_3,
    // MOT_VAR_SERIAL_NUMBER_REG32,
    // MOT_VAR_MANUFACTURE_NUMBER_0, // Day
    // MOT_VAR_MANUFACTURE_NUMBER_1, // Month
    // MOT_VAR_MANUFACTURE_NUMBER_2, // Year
    // MOT_VAR_MANUFACTURE_NUMBER_3, // Resv
    // MOT_VAR_MANUFACTURE_NUMBER_REG32,
    // MOT_VAR_HARDWARE_VERSION_0,
    // MOT_VAR_HARDWARE_VERSION_1,
    // MOT_VAR_HARDWARE_VERSION_2,
    // MOT_VAR_HARDWARE_VERSION_3,
    // MOT_VAR_HARDWARE_VERSION_REG32,
    // MOT_VAR_ID_EXT_0,
    // MOT_VAR_ID_EXT_1,
    // MOT_VAR_ID_EXT_2,
    // MOT_VAR_ID_EXT_3,
    // MOT_VAR_ID_EXT_REG32,
    // char NAME[8U];
    // union { uint8_t SERIAL_NUMBER[4U]; uint32_t SERIAL_NUMBER_REG; };
    // union
    // {
    //     uint8_t MANUFACTURE_NUMBER[4U];
    //     uint32_t MANUFACTURE_NUMBER_REG;
    //     struct { uint8_t MANUFACTURE_DAY; uint8_t MANUFACTURE_MONTH; uint8_t MANUFACTURE_YEAR; uint8_t MANUFACTURE_RESV; };
    // };
    // union { uint8_t HARDWARE_VERSION[4U]; uint32_t HARDWARE_VERSION_REG; };
    // uint8_t ID_EXT[4U];
    // uint8_t RESERVED[8U];

// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
// static Protocol_ReqCode_T ReadData(MotorControllerPtr_T p_appInterface, Protocol_ReqContext_T * args)
// {
//     MotProtocol_SubState_T * p_subState = args->p_SubState;
//     Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_AWAIT_PROCESS;
//     uint16_t readSize;
//     switch(p_subState->StateId)
//     {
//         case 0U: /* Tx Ack handled by common Sync */
            // p_subState->DataModeAddress = MotPacket_ReadDataReq_ParseAddress((MotPacket_DataModeReq_T *)args->p_RxPacket);
//             p_subState->DataModeSize = MotPacket_ReadDataReq_ParseSize((MotPacket_DataModeReq_T *)args->p_RxPacket);
//             p_subState->StateId = 1U;
//             args->TxSize = MotPacket_DataModeReadResp_Build((MotPacket_DataModeResp_T *)p_txPacket, MOT_VAR_STATUS_OK);
//             reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
//             break;
//         case 1U: /* Tx Data */
//             if(p_subState->DataModeSize > 0U)
//             {
//                 //sequenceid*32 == data
//                 readSize = (p_subState->DataModeSize > 32U) ? 32U : p_subState->DataModeSize;
//                 args->TxSize = MotPacket_DataRead_BuildStatus((MotPacket_DataMode_T *)args->p_TxPacket, 0, 0);
//                 args->TxSize = MotPacket_DataRead_BuildData((MotPacket_DataMode_T *)args->p_TxPacket, (uint8_t *)p_subState->DataModeAddress, readSize);
//                 p_subState->DataModeSize -= readSize;
//                 p_subState->DataModeAddress += readSize;
//                 reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
//             }
//             else /* (p_subState->DataModeSize == 0U) */
//             {
//                 args->TxSize = MotPacket_DataModeReadResp_Build((MotPacket_DataMode_T *)args->p_TxPacket, MOT_VAR_STATUS_OK);
//                 reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
//             }
//         default:
//             break;
//     }

//     return reqCode;
// }

// /******************************************************************************/
// /*! Stateful Write Data */
// /******************************************************************************/
// static Protocol_ReqCode_T WriteData_Blocking(MotorControllerPtr_T p_appInterface, Protocol_ReqContext_T * args)
// {
//     MotProtocol_SubState_T * const p_subState = args->p_SubState;
//     Flash_T * const p_flash = p_appInterface->CONFIG.P_FLASH;
//     Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_AWAIT_PROCESS;
//     Flash_Status_T flashStatus;
//     const uint8_t * p_writeData; /* DataPacket Payload */
//     uint8_t writeSize; /* DataPacket Size */

//     switch(p_subState->StateId)
//     {
//         case 0U: /* Tx Ack handled by Common Req Sync */
//             p_subState->DataModeAddress = MotPacket_WriteDataReq_ParseAddress((MotPacket_DataModeReq_T *)args->p_RxPacket);
//             p_subState->DataModeSize = MotPacket_WriteDataReq_ParseSize((MotPacket_DataModeReq_T *)args->p_RxPacket);
//             /* use full set to check boundaries. bytecount will be overwritten */
//             flashStatus = Flash_SetWrite(p_flash, (uint8_t *)p_subState->DataModeAddress, 0U, p_subState->DataModeSize);
//             p_subState->StateId = 1U;
//             args->TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, flashStatus);
//             reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE;
//             break;

//         case 1U: /* Write Data - rxPacket is DataPacket */
//             if(MotPacket_DataWrite_ParseChecksum((const MotPacket_DataMode_T *)args->p_RxPacket) == true)
//             {
//                 writeSize = MotPacket_DataWrite_ParseDataSize((const MotPacket_DataMode_T *)args->p_RxPacket);
//                 p_writeData = MotPacket_DataWrite_ParsePtrData((const MotPacket_DataMode_T *)args->p_RxPacket);
//             }
//             if(p_subState->DataModeSize >= writeSize)
//             {
//                 flashStatus = Flash_ContinueWrite_Blocking(p_flash, p_writeData, writeSize);
//                 if(flashStatus == FLASH_STATUS_SUCCESS)
//                 {
//                     p_subState->DataModeSize -= writeSize;
//                     args->TxSize = 0U;
//                     reqCode = PROTOCOL_REQ_CODE_PROCESS_CONTINUE; /* need separate state for tx response after tx ack */
//                 }
//                 else
//                 {
//                     args->TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, flashStatus);
//                     reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
//                 }
//             }
//             else
//             {
//                 args->TxSize = MotPacket_DataModeWriteResp_Build((MotPacket_DataModeResp_T *)p_txPacket, MOT_VAR_STATUS_OK);
//                 reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
//             }
//             break;

//         default:
//             break;
//     }

//     return reqCode;
// }
// #endif

/******************************************************************************/
/*! Req Table */
/******************************************************************************/
#define REQ_SYNC_DATA_MODE PROTOCOL_SYNC_EXT(1U, 1U, 1U, 1U, 3U)

static const Protocol_Req_T REQ_TABLE[] =
{
    PROTOCOL_REQ(MOT_PACKET_PING,           Ping,               0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_STOP_ALL,       StopAll,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VERSION,        Version,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_CALL,           Call_Blocking,      0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_WRITE,      VarWrite,           0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_VAR_READ,       VarRead,            0U,     PROTOCOL_SYNC_DISABLE),
#if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
    // PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
    // PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE,     0U,     WriteData_Blocking,     REQ_SYNC_DATA_MODE),
#endif
    // PROTOCOL_REQ(MOT_PACKET_WRITE_VAR,       WriteVar,           0U,     PROTOCOL_SYNC_DISABLE),
    // PROTOCOL_REQ(MOT_PACKET_READ_VAR,        ReadVar,            0U,     PROTOCOL_SYNC_DISABLE),
};

const Protocol_Specs_T MOTOR_CONTROLLER_MOT_PROTOCOL_SPECS =
{
    .RX_LENGTH_MIN = MOT_PACKET_LENGTH_MIN,
    .RX_LENGTH_MAX = MOT_PACKET_LENGTH_MAX,
    .PARSE_RX_META = (Protocol_ParseRxMeta_T)MotProtocol_ParseRxMeta,
    .BUILD_TX_SYNC = (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,
    .P_REQ_TABLE = &REQ_TABLE[0U],
    .REQ_TABLE_LENGTH = sizeof(REQ_TABLE) / sizeof(Protocol_Req_T),
    .REQ_EXT_RESET = 0U,
    .RX_START_ID = MOT_PACKET_START_BYTE,
    .RX_END_ID = 0x00U,
    .RX_TIMEOUT = MOT_PROTOCOL_TIMEOUT_RX,
    .REQ_TIMEOUT = MOT_PROTOCOL_TIMEOUT_REQ,
    .BAUD_RATE_DEFAULT = MOT_PROTOCOL_BAUD_RATE_DEFAULT,
};


// /******************************************************************************/
// /*! Read Single Var */
// /******************************************************************************/
// static uint8_t ReadVar(MotorControllerPtr_T p_mc, MotPacket_ReadVarResp_T * p_txPacket, const MotPacket_ReadVarReq_T * p_rxPacket)
// {
//
//     return MotPacket_ReadVarResp_Build(p_txPacket, MotorController_Var_Get(p_mc, (MotVarId_T)MotPacket_ReadVarReq_ParseVarId(p_rxPacket)));
// }

// /******************************************************************************/
// /*! Write Single Var */
// /******************************************************************************/
// static uint8_t WriteVar(MotorControllerPtr_T p_mc, MotPacket_WriteVarResp_T * p_txPacket, const MotPacket_WriteVarReq_T * p_rxPacket)
// {
//
//     MotPacket_Status_T status = MOT_VAR_STATUS_OK;
//     if(MotorController_Var_Set(p_mc, (MotVarId_T)MotPacket_WriteVarReq_ParseVarId(p_rxPacket), MotPacket_WriteVarReq_ParseVarValue(p_rxPacket)) == 0U)
//         { status = MOT_VAR_STATUS_ERROR; }
//     return MotPacket_WriteVarResp_Build(p_txPacket, status);
// }

