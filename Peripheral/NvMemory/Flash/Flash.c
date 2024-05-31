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
    @brief
    @version V0
*/
/******************************************************************************/
#include "Flash.h"
#include "System/Critical/Critical.h"
#include <string.h>
#include <assert.h>
// #include <stdalign.h>

/******************************************************************************/
/*!
    HAL Mediator Interface
*/
/******************************************************************************/
/******************************************************************************/
/*!
*/
/******************************************************************************/
static void StartCmdWritePage           (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseSector         (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyWriteUnit     (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyEraseUnits    (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdWriteOnce           (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdReadOnce            (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseAll            (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;

static void StartCmdWritePage           (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdWritePage(p_hal, destAddress, p_cmdData);}
static void StartCmdEraseSector         (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units;   HAL_Flash_StartCmdEraseSector(p_hal, destAddress);}
static void StartCmdVerifyWriteUnit     (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdVerifyWriteUnit(p_hal, destAddress, p_cmdData);}
static void StartCmdVerifyEraseUnits    (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData;                HAL_Flash_StartCmdVerifyEraseUnits(p_hal, destAddress, units);}
static void StartCmdWriteOnce           (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdWriteOnce(p_hal, destAddress, p_cmdData);}
static void StartCmdReadOnce            (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units;   HAL_Flash_StartCmdReadOnce(p_hal, destAddress);}
static void StartCmdEraseAll            (void * p_hal, uintptr_t destAddress, const uint8_t * p_cmdData, size_t units) { (void)destAddress; (void)p_cmdData; (void)units;  HAL_Flash_StartCmdEraseAll(p_hal);}

/******************************************************************************/
/*!
*/
/******************************************************************************/
static void FinalizeCmdReadOnce(void * p_hal, uintptr_t destAddress, uint8_t * p_data, size_t units) { HAL_Flash_ReadOnceData(p_hal, p_data); }

/******************************************************************************/
/*!
    HAL Parse Error
    Individual HAL interface for parsing error registers
        as register meaning might differ depending on Cmd
*/
/******************************************************************************/
static Flash_Status_T ParseCmdErrorWrite(void * p_hal)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorProtectionFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_PROTECTION; }
    else { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorErase(void * p_hal)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorProtectionFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_PROTECTION; }
    else { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorVerify(void * p_hal)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorVerifyFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_VERIFY; }
    else { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorWriteOnce(void * p_hal)
{
    (void)p_hal;
    return NV_MEMORY_STATUS_ERROR_CMD;
}

static Flash_Status_T ParseCmdErrorReadOnce(void * p_hal)
{
    (void)p_hal;
    return NV_MEMORY_STATUS_ERROR_CMD;
}


/******************************************************************************/
/*!
    Set Op - Common Blocking Non Blocking
*/
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_WRITE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdWritePage,
    .FINALIZE_CMD        = 0U,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorWrite,
    .UNIT_SIZE           = FLASH_UNIT_WRITE_SIZE,
    .FORCE_ALIGN         = NvMemory_AlignDown,
};

static Flash_Status_T SetWrite(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_WRITE, destAddress, p_data, size);
}


/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_ERASE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdEraseSector,
    .FINALIZE_CMD        = 0U,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorErase,
    .UNIT_SIZE           = FLASH_UNIT_ERASE_SIZE,
    .FORCE_ALIGN         = NvMemory_AlignUp,
};

static Flash_Status_T SetErase(Flash_T * p_flash, uintptr_t destAddress, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_ERASE, destAddress, NULL, size);
}

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_VERIFY_WRITE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdVerifyWriteUnit,
    .FINALIZE_CMD        = 0U,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorVerify,
    .UNIT_SIZE           = FLASH_UNIT_VERIFY_WRITE_SIZE,
    .FORCE_ALIGN          = NvMemory_AlignDown,
};

/* will repeat copy buffer for buffered verify after write case */
static Flash_Status_T SetVerifyWrite(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_VERIFY_WRITE, destAddress, p_data, size);
}

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_VERIFY_ERASE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdVerifyEraseUnits,
    .FINALIZE_CMD        = 0U,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorVerify,
    .UNIT_SIZE           = FLASH_UNIT_VERIFY_ERASE_SIZE,
    .FORCE_ALIGN         = NvMemory_AlignUp,

// #ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
//     return 0U; // overwrite with totalBytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
// #elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
//     return FLASH_UNIT_VERIFY_ERASE_SIZE;
// #endif
};

//todo units
// static inline size_t VerifyEraseUnitsPerCmd(size_t totalBytes)
// {
// #ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
//     return totalBytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
// #elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
//     return 1U;
// #endif
// }

static Flash_Status_T SetVerifyErase(Flash_T * p_flash, uintptr_t destAddress, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_VERIFY_ERASE, destAddress, NULL, size);
}

/******************************************************************************/
/*! */
/* Aligned source only */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_WRITE_ONCE =
{
    .START_CMD          = (HAL_NvMemory_StartCmd_T)StartCmdWriteOnce,
    .FINALIZE_CMD       = NULL,
    .PARSE_CMD_ERROR    = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorWriteOnce,
    .UNIT_SIZE          = FLASH_UNIT_WRITE_ONCE_SIZE,
    .FORCE_ALIGN        = NULL,
};

static Flash_Status_T SetWriteOnce(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_WRITE_ONCE, destAddress, p_data, size);
}

/******************************************************************************/
/*! */
/* Aligned source only */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_READ_ONCE =
{
    .START_CMD          = (HAL_NvMemory_StartCmd_T)StartCmdReadOnce,
    .FINALIZE_CMD       = FinalizeCmdReadOnce,
    .PARSE_CMD_ERROR    = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorReadOnce,
    .UNIT_SIZE          = FLASH_UNIT_READ_ONCE_SIZE,
    .FORCE_ALIGN        = NULL,
};

/* Sets p_OpData to result buffer */
static Flash_Status_T SetReadOnce(Flash_T * p_flash, uint8_t * p_resultBuffer, uintptr_t destAddress, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_READ_ONCE, destAddress, p_resultBuffer, size);
}

// void Flash_GetReadOnceResults(const Flash_T * p_flash, uint8_t * p_result)
// {
//     memcpy(p_result, &p_flash->CONFIG.P_BUFFER[0U], p_flash->OpSizeAligned);
// }

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_ERASE_ALL =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdEraseAll,
    .FINALIZE_CMD        = 0U,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_CmdStatus_T)ParseCmdErrorErase,
    .UNIT_SIZE           = 0U,
};

static Flash_Status_T SetEraseAll(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, &FLASH_OP_ERASE_ALL, 0U, NULL, 0u);
    return NV_MEMORY_STATUS_SUCCESS;
}

/******************************************************************************/
/*!
    Public Functions
*/
/******************************************************************************/
void Flash_Init(Flash_T * p_flash)
{
    HAL_Flash_Init(p_flash->CONFIG.P_HAL);
    NvMemory_Init(p_flash);
    p_flash->IsForceAlignEnable = true;
}


/******************************************************************************/
/*!
    Blocking Implementations
    Proc Include Set and Activate
*/
/******************************************************************************/
Flash_Status_T Flash_SetOp(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size, Flash_Operation_T opId)
{
    Flash_Status_T status;

    switch(opId)
    {
        case FLASH_OPERATION_WRITE:             status = SetWrite(p_flash, destAddress, p_data, size);                break;
        case FLASH_OPERATION_ERASE:             status = SetErase(p_flash, destAddress, size);                        break;
        case FLASH_OPERATION_VERIFY_WRITE:      status = SetVerifyWrite(p_flash, destAddress, p_data, size);          break;
        case FLASH_OPERATION_VERIFY_ERASE:      status = SetVerifyErase(p_flash, destAddress, size);                  break;
        case FLASH_OPERATION_WRITE_ONCE:        status = SetWriteOnce(p_flash, destAddress, p_data, size);            break;
        case FLASH_OPERATION_READ_ONCE:         status = SetReadOnce(p_flash, (uint8_t *)p_data, destAddress, size);  break;
        default:                                status = NV_MEMORY_STATUS_ERROR_INVALID_OP;                           break;
    }

    return status;
}

Flash_Status_T Flash_ProcThisOp_Blocking(Flash_T * p_flash)
{
    volatile Flash_Status_T status;
    Critical_Enter(); /* Flash Op must not invoke isr table stored in flash */
    status = NvMemory_ProcOp_Blocking(p_flash);
    Critical_Exit();
    return status;
}

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MAX_WRITE_SIZE MAX(FLASH_UNIT_WRITE_SIZE, FLASH_UNIT_WRITE_ONCE_SIZE)

// alt move to NvMemory, with flexible buffer
static Flash_Status_T WriteRemainder(Flash_T * p_flash, uint8_t unitSize)
{
    Flash_Status_T status;
    size_t remainder = p_flash->OpSize - p_flash->OpSizeAligned;
    uint8_t alignedData[MAX_WRITE_SIZE]; //= { [0U ... (MAX_WRITE_SIZE - 1U)] = FLASH_UNIT_ERASE_PATTERN };
    memset(&alignedData[0U], FLASH_UNIT_ERASE_PATTERN, MAX_WRITE_SIZE);
    memcpy(&alignedData[0U], &p_flash->p_OpData[p_flash->OpSizeAligned], remainder); /* start from remaining data, OpSizeAlignedDown */
    p_flash->p_OpData = &alignedData[0U];
    p_flash->OpDestAddress = p_flash->OpDestAddress + p_flash->OpSizeAligned; /* end of the aligned down  */
    p_flash->OpSizeAligned = unitSize;
    status = Flash_ProcThisOp_Blocking(p_flash);
    return status;
}

/* Shortcut check status proc */
static inline Flash_Status_T ProcAfterSet(Flash_T * p_flash, Flash_Status_T status)
{
    return (status == NV_MEMORY_STATUS_SUCCESS) ? Flash_ProcThisOp_Blocking(p_flash) : status;
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
    Flash_Status_T status = ProcAfterSet(p_flash, SetWrite(p_flash, destAddress, p_data, size));
    if(status == NV_MEMORY_STATUS_SUCCESS) { if(p_flash->OpSize != p_flash->OpSizeAligned) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_SIZE); } }
    return status;
}

/*
    Validates write boundaries.
    Total size is not retained on following writes.
*/
Flash_Status_T Flash_SetContinueWrite(Flash_T * p_flash, uintptr_t destAddress, size_t size)
{
    return SetWrite(p_flash, destAddress, NULL, size);
}

/*
    Append Write Operation, from prev p_dest, incrementing OpDestAddress
    Previous cmd must end on aligned boundary
*/
Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_data, size_t size)
{
    assert(p_flash->OpSize == p_flash->OpSizeAligned); /* Previous Write must have ended on an aligned boundary */
    // NV_MEMORY_STATUS_ERROR_ALIGNMENT
    return Flash_Write_Blocking(p_flash, p_flash->OpDestAddress + p_flash->OpSize, p_data, size);
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, uintptr_t destAddress, size_t size)
{
    return ProcAfterSet(p_flash, SetErase(p_flash, destAddress, size));
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
#if defined(FLASH_UNIT_VERIFY_WRITE_SIZE) && (FLASH_UNIT_VERIFY_WRITE_SIZE != 0U)
    return ProcAfterSet(p_flash, SetVerifyWrite(p_flash, destAddress, p_data, size));
#else  /* Manual compare if no hw verify implemented */
    return (memcmp((const uint8_t *)destAddress, p_data, size) == 0U) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_VERIFY;
#endif
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, uintptr_t destAddress, size_t size)
{
    return ProcAfterSet(p_flash, SetVerifyErase(p_flash, destAddress, size));
}

/* Caller ensure align. Alternatively write remainder filling with erase pattern which can be overwritten */
Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
{
    Flash_Status_T status = ProcAfterSet(p_flash, SetWriteOnce(p_flash, destAddress, p_data, size));
    // if(status == NV_MEMORY_STATUS_SUCCESS) { if(p_flash->OpSizeRemainder != 0U) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE); } }
    return status;
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, uint8_t * p_resultBuffer, uintptr_t destAddress, size_t size)
{
    return ProcAfterSet(p_flash, SetReadOnce(p_flash, p_resultBuffer, destAddress, size));
}

Flash_Status_T Flash_EraseAll_Blocking(Flash_T *p_flash)
{
    return ProcAfterSet(p_flash, SetEraseAll(p_flash));
}

Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size, Flash_Operation_T opId)
{
    return ProcAfterSet(p_flash, Flash_SetOp(p_flash, destAddress, p_data, size, opId));
}


/******************************************************************************/
/*!
    Non Blocking - UNTESTED
*/
/******************************************************************************/
/*
    returns true when complete
*/
// bool Flash_ProcOp(Flash_T * p_flash) { return NvMemory_ProcOp(p_flash); }

// size_t Flash_GetOpBytesRemaining(Flash_T * p_flash) { return NvMemory_GetOpBytesRemaining(p_flash); }

// // bool Flash_ReadIsOpComplete(Flash_T * p_flash) { return NvMemory_ReadIsOpComplete(p_flash); }

// Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (SetWrite(p_flash, destAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, uintptr_t destAddress, size_t size)
// {
//     p_flash->Status = (SetErase(p_flash, destAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyWrite(p_flash, destAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, uintptr_t destAddress, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyErase(p_flash, destAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, uintptr_t destAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (Flash_SetWriteOnce(p_flash, destAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, uint8_t * p_resultBuffer, const uint8_t * p_once, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, p_resultBuffer, p_once, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }
// void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
// {
//     memcpy(p_result, &p_flash->CONFIG.P_BUFFER[0U], p_flash->OpSizeAligned);
// }
// Flash_Status_T Flash_StartReadOnce_Direct_NonBlocking(Flash_T * p_flash, uint8_t * p_destResult, uintptr_t destAddress, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, destAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }
/******************************************************************************/
/*!
    Virtual
*/
/******************************************************************************/
/*
    creates copy of flash in buffer
*/
 //void Flash_OpenVirtual(Flash_T * p_flash, const uint8_t * p_physical, size_t size)
 //{

 //
 //}
 //
 ///*
 // * dest of physical flash location
 // */
 //void Flash_WriteVirtual(Flash_T * p_flash, const uint8_t * p_physical, const uint8_t * p_src, size_t size)
 //{

 //}
 //
 //void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t size)
 //{

 //}
 //
 //void Flash_CloseVirtual(Flash_T * p_flash)
 //{
 //    p_flash->State = FLASH_STATE_WRITE;
 //}
 //
 //bool Flash_CloseVirtual_Blocking(Flash_T * p_flash)
 //{

 //}

