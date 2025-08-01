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
static void StartCmdWritePage           (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseSector         (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyWriteUnit     (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyEraseUnits    (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdWriteOnce           (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdReadOnce            (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseAll            (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;

static void StartCmdWritePage           (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)units;                  HAL_Flash_StartCmdWritePage(p_hal, address, p_data); }
static void StartCmdEraseSector         (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)p_data; (void)units;    HAL_Flash_StartCmdEraseSector(p_hal, address); }
static void StartCmdVerifyWriteUnit     (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)units;                  HAL_Flash_StartCmdVerifyWriteUnit(p_hal, address, p_data); }
static void StartCmdVerifyEraseUnits    (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)p_data;                 HAL_Flash_StartCmdVerifyEraseUnits(p_hal, address, units); }
static void StartCmdWriteOnce           (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)units;                  HAL_Flash_StartCmdWriteOnce(p_hal, address, p_data); }
static void StartCmdReadOnce            (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)p_data; (void)units;    HAL_Flash_StartCmdReadOnce(p_hal, address); }
static void StartCmdEraseAll            (void * p_hal, uintptr_t address, const uint8_t * p_data, size_t units) { (void)address; (void)p_data; (void)units;  HAL_Flash_StartCmdEraseAll(p_hal); }

/******************************************************************************/
/*!
*/
/******************************************************************************/
static void FinalizeCmdReadOnce(void * p_hal, uintptr_t address, size_t units, uint8_t * p_data) { HAL_Flash_ReadOnceData(p_hal, p_data); }

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
    if (HAL_Flash_ReadErrorProtectionFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_PROTECTION; }
    else { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorErase(void * p_hal)
{
    Flash_Status_T status;
    if (HAL_Flash_ReadErrorProtectionFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_PROTECTION; }
    else { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorVerify(void * p_hal)
{
    Flash_Status_T status;
    if (HAL_Flash_ReadErrorVerifyFlag(p_hal) == true) { status = NV_MEMORY_STATUS_ERROR_VERIFY; }
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
    .FINALIZE_CMD        = NULL,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_MapStatus_T)ParseCmdErrorWrite,
    .UNIT_SIZE           = FLASH_UNIT_WRITE_SIZE,
    .FORCE_ALIGN         = nvmemory_align_down,
};

static Flash_Status_T SetWrite(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
    return NvMemory_SetOpControl_Write(p_flash, &FLASH_OP_WRITE, flashAddress, p_data, size);
}

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_ERASE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdEraseSector,
    .FINALIZE_CMD        = NULL,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_MapStatus_T)ParseCmdErrorErase,
    .UNIT_SIZE           = FLASH_UNIT_ERASE_SIZE,
    .FORCE_ALIGN         = nvmemory_align_up,
};

static Flash_Status_T SetErase(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_ERASE, flashAddress, size);
}

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_VERIFY_WRITE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdVerifyWriteUnit,
    .FINALIZE_CMD        = NULL,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_MapStatus_T)ParseCmdErrorVerify,
    .UNIT_SIZE           = FLASH_UNIT_VERIFY_WRITE_SIZE,
    .FORCE_ALIGN         = nvmemory_align_down,
};


static Flash_Status_T SetVerifyWrite(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
    return NvMemory_SetOpControl_Write(p_flash, &FLASH_OP_VERIFY_WRITE, flashAddress, p_data, size); /* will repeat copy buffer for buffered verify after write case */
}

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_VERIFY_ERASE =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdVerifyEraseUnits,
    .FINALIZE_CMD        = NULL,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_MapStatus_T)ParseCmdErrorVerify,
    .UNIT_SIZE           = FLASH_UNIT_VERIFY_ERASE_SIZE,
    .FORCE_ALIGN         = nvmemory_align_up,
};

//todo units
// #ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
//     return 0U; // overwrite with totalBytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
// #elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
//     return FLASH_UNIT_VERIFY_ERASE_SIZE;
// #endif
// static inline size_t VerifyEraseUnitsPerCmd(size_t totalBytes)
// {
// #ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
//     return totalBytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
// #elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
//     return 1U;
// #endif
// }

static Flash_Status_T SetVerifyErase(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
{
    return NvMemory_SetOpControl(p_flash, &FLASH_OP_VERIFY_ERASE, flashAddress, size);
}

/******************************************************************************/
/*! */
/* Aligned source only */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_WRITE_ONCE =
{
    .START_CMD          = (HAL_NvMemory_StartCmd_T)StartCmdWriteOnce,
    .FINALIZE_CMD       = NULL,
    .PARSE_CMD_ERROR    = (HAL_NvMemory_MapStatus_T)ParseCmdErrorWriteOnce,
    .UNIT_SIZE          = FLASH_UNIT_WRITE_ONCE_SIZE,
    .FORCE_ALIGN        = NULL,
};

static Flash_Status_T SetWriteOnce(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
    return NvMemory_SetOpControl_Write(p_flash, &FLASH_OP_WRITE_ONCE, flashAddress, p_data, size);
}

/******************************************************************************/
/*! */
/* Aligned source only */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_READ_ONCE =
{
    .START_CMD          = (HAL_NvMemory_StartCmd_T)StartCmdReadOnce,
    .FINALIZE_CMD       = (HAL_NvMemory_FinalizeCmd_T)FinalizeCmdReadOnce,
    .PARSE_CMD_ERROR    = (HAL_NvMemory_MapStatus_T)ParseCmdErrorReadOnce,
    .UNIT_SIZE          = FLASH_UNIT_READ_ONCE_SIZE,
    .FORCE_ALIGN        = NULL,
};

/* Sets p_OpData to result buffer */
static Flash_Status_T SetReadOnce(Flash_T * p_flash, uintptr_t flashAddress, size_t size, void * p_result)
{
    return NvMemory_SetOpControl_Read(p_flash, &FLASH_OP_READ_ONCE, flashAddress, size, p_result);
}

// void Flash_GetReadOnceResults(const Flash_T * p_flash, uint8_t * p_result)
// {
//     memcpy(p_result, &p_flash->CONST.P_BUFFER[0U], p_flash->OpSizeAligned);
// }

/******************************************************************************/
/*! */
/******************************************************************************/
static const NvMemory_OpControl_T FLASH_OP_ERASE_ALL =
{
    .START_CMD           = (HAL_NvMemory_StartCmd_T)StartCmdEraseAll,
    .FINALIZE_CMD        = NULL,
    .PARSE_CMD_ERROR     = (HAL_NvMemory_MapStatus_T)ParseCmdErrorErase,
    .UNIT_SIZE           = 0U,
};

static Flash_Status_T SetEraseAll(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, &FLASH_OP_ERASE_ALL, 0U, 0U);
    return NV_MEMORY_STATUS_SUCCESS;
}

/******************************************************************************/
/*!
    Public Functions
*/
/******************************************************************************/
void Flash_Init(Flash_T * p_flash)
{
    HAL_Flash_Init(p_flash-> P_HAL);
    NvMemory_Init(p_flash);
    p_flash->P_STATE->IsForceAlignEnable = true;
}


/******************************************************************************/
/*!
    Blocking Implementations
    Proc Include Set and Activate
*/
/******************************************************************************/
Flash_Status_T Flash_ProcThisOp_Blocking(Flash_T * p_flash)
{
    Flash_Status_T status;
    _Critical_DisableIrq(); /* Flash Op must not invoke isr table stored in flash */
    status = NvMemory_ProcOp_Blocking(p_flash);
    _Critical_EnableIrq();
    return status;
}

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MAX_WRITE_SIZE MAX(FLASH_UNIT_WRITE_SIZE, FLASH_UNIT_WRITE_ONCE_SIZE)

// alt move to NvMemory, with flexible buffer
static Flash_Status_T WriteRemainder(Flash_T * p_flash, uint8_t unitSize)
{
    NvMemory_State_T * p_state = p_flash->P_STATE;

    size_t remainder = p_state->OpSize - p_state->OpSizeAligned; /* unaligned size */
    // size_t remainder = NvMemory_GetOpSizeRemainder(p_state);
    uint8_t alignedData[MAX_WRITE_SIZE]; //= { [0U ... (MAX_WRITE_SIZE - 1U)] = FLASH_UNIT_ERASE_PATTERN };
    Flash_Status_T status;

    if (remainder > MAX_WRITE_SIZE) { status = NV_MEMORY_STATUS_ERROR_ALIGNMENT; }
    else
    {
        memset(&alignedData[0U], FLASH_UNIT_ERASE_PATTERN, MAX_WRITE_SIZE);
        memcpy(&alignedData[0U], &((const uint8_t *)p_state->p_OpData)[p_state->OpSizeAligned], remainder); /* Start from remaining data, OpSize AlignDown */

        p_state->p_OpData = &alignedData[0U];
        p_state->OpAddress = p_state->OpAddress + p_state->OpSizeAligned; /* Update previous aligned down address  */
        p_state->OpSizeAligned = unitSize;
        status = Flash_ProcThisOp_Blocking(p_flash);
    }

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
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
    Flash_Status_T status = ProcAfterSet(p_flash, SetWrite(p_flash, flashAddress, p_data, size));
    if (status == NV_MEMORY_STATUS_SUCCESS) { if (p_flash->P_STATE->OpSize > p_flash->P_STATE->OpSizeAligned) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_SIZE); } }
    return status;
}

/*
    Validates write boundaries.
    Total size is not retained on following writes.
*/
Flash_Status_T Flash_SetContinueWrite(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
{
    return SetWrite(p_flash, flashAddress, NULL, size);
}

/*
    Append Write Operation, from prev p_dest, incrementing OpAddress
    Previous cmd must end on aligned boundary
*/
Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const void * p_data, size_t size)
{
    assert(p_flash->P_STATE->OpSize == p_flash->P_STATE->OpSizeAligned); /* Previous Write must have ended on an aligned boundary */

    Flash_Status_T status = Flash_Write_Blocking(p_flash, p_flash->P_STATE->OpAddress, p_data, size);
    p_flash->P_STATE->OpAddress += size;
    return status;
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
{
    return ProcAfterSet(p_flash, SetErase(p_flash, flashAddress, size));
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
#if defined(FLASH_UNIT_VERIFY_WRITE_SIZE) && (FLASH_UNIT_VERIFY_WRITE_SIZE != 0U)
    return ProcAfterSet(p_flash, SetVerifyWrite(p_flash, flashAddress, p_data, size));
#else  /* Manual compare if no hw verify implemented */
    return (memcmp((const void *)flashAddress, (const void *)p_data, size) == 0U) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_VERIFY;
    // return NvMemory_MemCompare(flashAddress, p_data, size);
#endif
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
{
    return ProcAfterSet(p_flash, SetVerifyErase(p_flash, flashAddress, size));
}

/* Caller ensure align. Alternatively write remainder filling with erase pattern which can be overwritten */
// if(status == NV_MEMORY_STATUS_SUCCESS) { if(p_flash->OpSizeRemainder != 0U) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE); } }
Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, uintptr_t flashAddress, const void * p_data, size_t size)
{
    return ProcAfterSet(p_flash, SetWriteOnce(p_flash, flashAddress, p_data, size));
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size, void * p_result)
{
    return ProcAfterSet(p_flash, SetReadOnce(p_flash, flashAddress, size, p_result));
}

Flash_Status_T Flash_EraseAll_Blocking(Flash_T * p_flash)
{
    return ProcAfterSet(p_flash, SetEraseAll(p_flash));
}

/* memcpy with partitions bounds check */
// Flash_Status_T Flash_Read_Blocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size, uint8_t * p_result)
// {
//     return ProcAfterSet(p_flash, SetReadOnce(p_flash, flashAddress, size, p_result));
// }




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

// Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (SetWrite(p_flash, flashAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
// {
//     p_flash->Status = (SetErase(p_flash, flashAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyWrite(p_flash, flashAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyErase(p_flash, flashAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, uintptr_t flashAddress, const uint8_t * p_data, size_t size)
// {
//     p_flash->Status = (Flash_SetWriteOnce(p_flash, flashAddress, p_data, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, uint8_t * p_result, const uint8_t * p_once, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, p_result, p_once, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }
// void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
// {
//     memcpy(p_result, &p_flash->CONST.P_BUFFER[0U], p_flash->OpSizeAligned);
// }
// Flash_Status_T Flash_StartReadOnce_Direct_NonBlocking(Flash_T * p_flash, uint8_t * p_destResult, uintptr_t flashAddress, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, flashAddress, size) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : NV_MEMORY_STATUS_ERROR_INPUT);
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

