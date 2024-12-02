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
    MotProtocol implementation using MotorController_T
    Directly use MotorController_T as Protocol interface (P_APP_INTERFACE) avoids double buffering
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
static protocol_size_t Ping(MotorController_T * p_mc, MotPacket_PingResp_T * p_txPacket, const MotPacket_PingReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_User_BeepN(p_mc, 500U, 500U, 1U);
    return MotPacket_PingResp_Build(p_txPacket, MOT_PACKET_SYNC_ACK);
}

/******************************************************************************/
/*! Version */
/******************************************************************************/
static protocol_size_t Version(MotorController_T * p_mc, MotPacket_VersionResp_T * p_txPacket, const MotPacket_VersionReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    return MotPacket_VersionResp_Build(p_txPacket, MotorController_User_GetLibraryVersion(), MotorController_User_GetMainVersion(p_mc), 0);
    // todo with variable
    // return MotPacket_VersionFlexResp_Build(p_txPacket, MotorController_User_GetLibraryVersion(), MotorController_User_GetMainVersion(p_mc), 0);
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static protocol_size_t StopAll(MotorController_T * p_mc, MotPacket_StopResp_T * p_txPacket, const MotPacket_StopReq_T * p_rxPacket)
{
    (void)p_rxPacket;
    MotorController_User_DisableControl(p_mc);
    return MotPacket_StopResp_Build(p_txPacket, MOT_STATUS_OK);
}

/******************************************************************************/
/*! Call - Blocking  */
// functions of the same cast type
/******************************************************************************/
typedef enum MotProtocol_CallId
{
    MOT_CALL_LOCKED_STATE,
    // MOT_CALL_REBOOT,
}
MotProtocol_CallId_T;

static protocol_size_t Call_Blocking(MotorController_T * p_mc, MotPacket_CallResp_T * p_txPacket, const MotPacket_CallReq_T * p_rxPacket)
{
    Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
    uint16_t status = 0U; /* Overloaded status */

    switch((MotProtocol_CallId_T)p_rxPacket->CallReq.Id)
    {
        case MOT_CALL_LOCKED_STATE:
            // status = MotorController_User_InputLock(p_mc, (MotorController_LockId_T)p_rxPacket->CallReq.Arg); //todo
            switch((MotorController_LockId_T)p_rxPacket->CallReq.Arg) /* StateMachine will check for invalid BlockingId */
            {
                case MOTOR_CONTROLLER_LOCK_PARK:      status = MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_PARK) ? MOT_STATUS_OK : MOT_STATUS_ERROR;  break;
                case MOTOR_CONTROLLER_LOCK_ENTER:     status = MotorController_User_EnterLockState(p_mc) ? MOT_STATUS_OK : MOT_STATUS_ERROR;  break;
                case MOTOR_CONTROLLER_LOCK_EXIT:      status = MotorController_User_ExitLockState(p_mc) ? MOT_STATUS_OK : MOT_STATUS_ERROR;   break;
                /* Non Blocking function, host/caller poll status after. */ // calibration status todo
                case MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR:    MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_CALIBRATE_SENSOR);    status = MOT_STATUS_OK; break;
                case MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC:       MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC);       status = MOT_STATUS_OK; break;
                /* Blocking functions can directly return status. */
                case MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG:     status = MotorController_User_SaveConfig_Blocking(p_mc);    break;
                case MOTOR_CONTROLLER_LOCK_REBOOT:       MotorController_User_InputLock(p_mc, MOTOR_CONTROLLER_LOCK_REBOOT);  status = MOT_STATUS_OK; break;
                default: break;
            }
            break;
        default: break;
    }

    return MotPacket_CallResp_Build(p_txPacket, p_rxPacket->CallReq.Id, status);
}

/******************************************************************************/
/*! Read Vars */
/******************************************************************************/
/* Resp Truncates 32-Bit Vars */
static protocol_size_t VarRead(MotorController_T * p_mc, MotPacket_VarReadResp_T * p_txPacket, const MotPacket_VarReadReq_T * p_rxPacket)
{
    uint8_t varCount = MotPacket_VarReadReq_ParseVarIdCount(p_rxPacket);

    for(uint8_t index = 0U; index < varCount; index++)
    {
        MotPacket_VarReadResp_BuildVarValue(p_txPacket, index, (uint16_t)MotorController_Var_Get(p_mc, (MotVarId_T)MotPacket_VarReadReq_ParseVarId(p_rxPacket, index)));
    }
    // MotPacket_VarReadResp_BuildMeta(p_txPacket, MOT_VAR_STATUS_OK);
    return MotPacket_VarReadResp_BuildHeader(p_txPacket, varCount);

}

/******************************************************************************/
/*! Write Vars */
/******************************************************************************/
static protocol_size_t VarWrite(MotorController_T * p_mc, MotPacket_VarWriteResp_T * p_txPacket, const MotPacket_VarWriteReq_T * p_rxPacket)
{
    uint8_t varCount = MotPacket_VarWriteReq_ParseVarCount(p_rxPacket);

    MotVarId_Status_T headerStatus = MOT_VAR_STATUS_OK;
    MotVarId_Status_T varStatus;
    for(uint8_t index = 0U; index < varCount; index++)
    {
        varStatus = MotorController_Var_Set(p_mc, (MotVarId_T)MotPacket_VarWriteReq_ParseVarId(p_rxPacket, index), MotPacket_VarWriteReq_ParseVarValue(p_rxPacket, index));
        MotPacket_VarWriteResp_BuildVarStatus(p_txPacket, index, varStatus);
        if(varStatus != MOT_VAR_STATUS_OK) { headerStatus = MOT_VAR_STATUS_ERROR; }
    }
    // MotPacket_VarWriteResp_BuildMeta(p_txPacket, headerStatus);
    return MotPacket_VarWriteResp_BuildHeader(p_txPacket, varCount);
}

/******************************************************************************/
/*! Mem */
/* with StateMachine check and address virtualization */
/******************************************************************************/
static protocol_size_t ReadMem_Blocking(MotorController_T * p_mc, MotPacket_MemReadResp_T * p_txPacket, const MotPacket_MemReadReq_T * p_rxPacket)
{
    uint32_t address = p_rxPacket->MemReadReq.Address;
    uint8_t size = p_rxPacket->MemReadReq.Size;
    uint16_t config = p_rxPacket->MemReadReq.Config;

    uint8_t * p_buffer = &(p_txPacket->MemReadResp.ByteData[0U]);
    uint8_t * p_data;
    NvMemory_Status_T status;

    memset(p_buffer, 0U, size);

    switch(config)
    {
        case MOT_MEM_CONFIG_RAM: memcpy(p_buffer, (void *)address, size);  status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_ONCE: status = MotorController_User_ReadManufacture_Blocking(p_mc, p_buffer, address, size); break;
        // case MOT_MEM_CONFIG_FLASH: memcpy(p_buffer, (void *)address, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return MotPacket_MemReadResp_BuildHeader(p_txPacket, size, status);
}

static protocol_size_t WriteMem_Blocking(MotorController_T * p_mc, MotPacket_MemWriteResp_T * p_txPacket, const MotPacket_MemWriteReq_T * p_rxPacket)
{
    uint32_t address = p_rxPacket->MemWriteReq.Address;
    const uint8_t * p_data = &(p_rxPacket->MemWriteReq.ByteData[0U]);
    uint8_t size = p_rxPacket->MemWriteReq.Size;
    uint16_t config = p_rxPacket->MemWriteReq.Config;
    NvMemory_Status_T status;

    switch(config)
    {
        case MOT_MEM_CONFIG_RAM: memcpy((void *)address, p_data, size); status = NV_MEMORY_STATUS_SUCCESS; break;
        case MOT_MEM_CONFIG_ONCE: status = MotorController_User_WriteManufacture_Blocking(p_mc, address, p_data, size); break;
        // case MOT_MEM_CONFIG_FLASH: status = Flash_Write_Blocking(p_flash, address, p_data, size); break;
        default: status = NV_MEMORY_STATUS_ERROR_NOT_IMPLEMENTED; break;
    }

    return MotPacket_MemWriteResp_Build(p_txPacket, status);
}

/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
static Protocol_ReqCode_T ReadData(MotorController_T * p_mc, Protocol_ReqContext_T * p_reqContext)
{
    // if(MotorController_User_IsLockState(p_mc) == true)
    return MotProtocol_ReadData(NULL, p_reqContext);
}

/******************************************************************************/
/*! Stateful Write Data */
/******************************************************************************/
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
static Protocol_ReqCode_T WriteData_Blocking(MotorController_T * p_mc, Protocol_ReqContext_T * p_reqContext)
{
    return MotProtocol_Flash_WriteData_Blocking(p_mc->CONST.P_FLASH, p_reqContext);
}
// #endif

// /******************************************************************************/
// /*! Read Single Var */
// /******************************************************************************/
// static uint8_t ReadVar(MotorController_T * p_mc, MotPacket_ReadVarResp_T * p_txPacket, const MotPacket_ReadVarReq_T * p_rxPacket)
// {
//     return MotPacket_ReadVarResp_Build(p_txPacket, MotorController_Var_Get(p_mc, (MotVarId_T)MotPacket_ReadVarReq_ParseVarId(p_rxPacket)));
// }
// /******************************************************************************/
// /*! Write Single Var */
// /******************************************************************************/
// static uint8_t WriteVar(MotorController_T * p_mc, MotPacket_WriteVarResp_T * p_txPacket, const MotPacket_WriteVarReq_T * p_rxPacket)
// {
//     MotPacket_Status_T status = MOT_VAR_STATUS_OK;
//     if(MotorController_Var_Set(p_mc, (MotVarId_T)MotPacket_WriteVarReq_ParseVarId(p_rxPacket), MotPacket_WriteVarReq_ParseVarValue(p_rxPacket)) == 0U)
//         { status = MOT_VAR_STATUS_ERROR; }
//     return MotPacket_WriteVarResp_Build(p_txPacket, status);
// }

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
    PROTOCOL_REQ(MOT_PACKET_MEM_READ,       ReadMem_Blocking,   0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_MEM_WRITE,      WriteMem_Blocking,  0U,     PROTOCOL_SYNC_DISABLE),
    // PROTOCOL_REQ(MOT_PACKET_WRITE_VAR,       WriteVar,           0U,     PROTOCOL_SYNC_DISABLE),
    // PROTOCOL_REQ(MOT_PACKET_READ_VAR,        ReadVar,            0U,     PROTOCOL_SYNC_DISABLE),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
    PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE,     0U,     WriteData_Blocking,     REQ_SYNC_DATA_MODE),
    // PROTOCOL_REQ(MOT_PACKET_DATA_MODE_DATA,      0U,     ReadData,               REQ_SYNC_DATA_MODE),
// #if defined(CONFIG_MOTOR_CONTROLLER_FLASH_LOADER_ENABLE)
// #endif
};

const Protocol_Specs_T MOTOR_CONTROLLER_MOT_PROTOCOL_SPECS =
{
    .RX_LENGTH_MIN = MOT_PACKET_LENGTH_MIN,
    .RX_LENGTH_MAX = MOT_PACKET_LENGTH_MAX,
    .PARSE_RX_META = (Protocol_ParseRxMeta_T)MotProtocol_ParseRxMeta,
    .BUILD_TX_SYNC = (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,
    .P_REQ_TABLE = &REQ_TABLE[0U],
    .REQ_TABLE_LENGTH = sizeof(REQ_TABLE) / sizeof(Protocol_Req_T),
    .RX_START_ID = MOT_PACKET_START_BYTE,
    .RX_TIMEOUT = MOT_PROTOCOL_TIMEOUT_RX,
    .REQ_TIMEOUT = MOT_PROTOCOL_TIMEOUT_REQ,
    // .BAUD_RATE_DEFAULT = MOT_PROTOCOL_BAUD_RATE_DEFAULT,
};

