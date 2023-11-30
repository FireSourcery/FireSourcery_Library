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
    HAL Launch Cmd
*/
/******************************************************************************/
//todo use struct, regularize hal or flash
static void StartCmdWritePage           (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseSector         (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyWriteUnit     (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdVerifyEraseUnits    (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdWriteOnce           (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdReadOnce            (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void StartCmdEraseAll            (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;

static void StartCmdWritePage           (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdWritePage(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdEraseSector         (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units;   HAL_Flash_StartCmdEraseSector(p_hal, p_cmdDest);}
static void StartCmdVerifyWriteUnit     (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdVerifyWriteUnit(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdVerifyEraseUnits    (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData;                HAL_Flash_StartCmdVerifyEraseUnits(p_hal, p_cmdDest, units);}
static void StartCmdWriteOnce           (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)units;                    HAL_Flash_StartCmdWriteOnce(p_hal, p_cmdDest, p_cmdData);}
static void StartCmdReadOnce            (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdData; (void)units;   HAL_Flash_StartCmdReadOnce(p_hal, p_cmdDest);}
static void StartCmdEraseAll            (HAL_Flash_T * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units) { (void)p_cmdDest; (void)p_cmdData; (void)units;  HAL_Flash_StartCmdEraseAll(p_hal);}

/******************************************************************************/
/*!
    Wrap HAL Finalize Cmd
*/
/******************************************************************************/
// static Flash_Status_T FinalizeCmdNull(Flash_T * p_flash) { (void)p_flash; }
static Flash_Status_T FinalizeCmdReadOnce(Flash_T * p_flash, size_t opIndex)
{
    HAL_Flash_ReadOnceData(p_flash->CONFIG.P_HAL, &p_flash->CONFIG.P_BUFFER[opIndex]);
    return FLASH_STATUS_SUCCESS;
}

/******************************************************************************/
/*!
    Wrap HAL Parse Error
    Individual HAL interface for parsing error registers
        as register meaning might differ depending on Cmd
*/
/******************************************************************************/
static Flash_Status_T ParseCmdErrorWrite(Flash_T * p_flash)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorProtectionFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_PROTECTION; }
    else { status = FLASH_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorErase(Flash_T * p_flash)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorProtectionFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_PROTECTION; }
    else { status = FLASH_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorVerify(Flash_T * p_flash)
{
    Flash_Status_T status;
    if(HAL_Flash_ReadErrorVerifyFlag(p_flash->CONFIG.P_HAL) == true) { status = FLASH_STATUS_ERROR_VERIFY; }
    else { status = FLASH_STATUS_ERROR_CMD; }
    return status;
}

static Flash_Status_T ParseCmdErrorWriteOnce(Flash_T * p_flash)
{
    (void)p_flash;
    return FLASH_STATUS_ERROR_CMD;
}

static Flash_Status_T ParseCmdErrorReadOnce(Flash_T * p_flash)
{
    (void)p_flash;
    return FLASH_STATUS_ERROR_CMD;
}



/******************************************************************************/
/*!
    Private Set - Referenced by Finalize and Set todo struct
    todo replace with set image precompile fixed
*/
/******************************************************************************/
static inline size_t VerifyEraseUnitsPerCmd(size_t totalBytes)
{
#ifdef CONFIG_FLASH_HW_VERIFY_ERASE_N_UNITS
    return totalBytes / FLASH_UNIT_VERIFY_ERASE_SIZE;
#elif defined(CONFIG_FLASH_HW_VERIFY_ERASE_1_UNIT)
    return 1U;
#endif
}

// static const NvMemory_OpControl_T FLASH_OP_WRITE =
// {
//     .START_CMD           = StartCmdWritePage,
//     .FINALIZE_CMD        = 0U,
//     .PARSE_CMD_ERROR     = ParseCmdErrorWrite,
//     .FINALIZE_OP         = FinalizeWrite,
//     .UNIT_SIZE           = FLASH_UNIT_WRITE_SIZE,
// };

static inline void SetWriteControl(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdWritePage, (NvMemory_Process_T)ParseCmdErrorWrite);
    NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_SIZE, 1U);
}

static inline void SetEraseControl(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdEraseSector, (NvMemory_Process_T)ParseCmdErrorErase);
    NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_ERASE_SIZE, 1U);
}

static inline void SetVerifyWriteControl(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdVerifyWriteUnit, (NvMemory_Process_T)ParseCmdErrorVerify);
    NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_WRITE_SIZE, 1U);
}

static inline void SetVerifyEraseControl(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdVerifyEraseUnits, (NvMemory_Process_T)ParseCmdErrorVerify);
    NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_VERIFY_ERASE_SIZE, VerifyEraseUnitsPerCmd(p_flash->OpSize));
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
    p_flash->IsFillAlignEnable = true;
}

/* Yield must point to RAM address for Flash case  */
void Flash_SetYield(Flash_T * p_flash, void (*yield)(void *), void * p_callbackData) { NvMemory_SetYield(p_flash, yield, p_callbackData); }

bool Flash_ReadSecurityFlag(Flash_T * p_flash) { return HAL_Flash_ReadSecurityFlag(p_flash->CONFIG.P_HAL); }

void Flash_EnableFillAlign(Flash_T * p_flash) { NvMemory_EnableFillAlign(p_flash); }
void Flash_DisableFillAlign(Flash_T * p_flash) { NvMemory_DisableFillAlign(p_flash); }

/******************************************************************************/
/*!
    Set Op - Common Blocking Non Blocking
    change to private?
    template function with struct?
*/
/******************************************************************************/
Flash_Status_T Flash_SetWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSizeAlignDown(p_flash, size, FLASH_UNIT_WRITE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpDataSource(p_flash, p_source, size); }
    if(status == FLASH_STATUS_SUCCESS) { SetWriteControl(p_flash); /* p_flash->p_OpControl = &FLASH_OP_WRITE; */ }
    return status;
}

Flash_Status_T Flash_SetErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_ERASE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSizeAlignUp(p_flash, size, FLASH_UNIT_ERASE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS) { SetEraseControl(p_flash); }
    return status;
}

Flash_Status_T Flash_SetVerifyWrite(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_WRITE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSizeAlignDown(p_flash, size, FLASH_UNIT_VERIFY_WRITE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpDataSource(p_flash, p_source, size); } /* will repeat copy for verify after write case */
    if(status == FLASH_STATUS_SUCCESS) { SetVerifyWriteControl(p_flash); }
    return status;
}

Flash_Status_T Flash_SetVerifyErase(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_VERIFY_ERASE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSizeAlignUp(p_flash, size, FLASH_UNIT_VERIFY_ERASE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS) { SetVerifyEraseControl(p_flash); }
    return status;
}

/* Aligned source only */
Flash_Status_T Flash_SetWriteOnce(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destFlash, size, FLASH_UNIT_WRITE_ONCE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSize(p_flash, size, FLASH_UNIT_WRITE_ONCE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpDataSource(p_flash, p_source, size); }
    if(status == FLASH_STATUS_SUCCESS)
    {
        NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdWriteOnce, (NvMemory_Process_T)ParseCmdErrorWriteOnce);
        NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE, 1U);
    }
    return status;
}

/* Aligned source only */
Flash_Status_T Flash_SetReadOnce(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_destOnce, size_t size)
{
    Flash_Status_T status = NvMemory_SetOpDest(p_flash, p_destOnce, size, FLASH_UNIT_READ_ONCE_SIZE);
    if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSize(p_flash, size, FLASH_UNIT_READ_ONCE_SIZE); }
    if(status == FLASH_STATUS_SUCCESS)
    {
        NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdReadOnce, (NvMemory_Process_T)ParseCmdErrorReadOnce);
        NvMemory_SetOpCmdSize(p_flash, FLASH_UNIT_READ_ONCE_SIZE, 1U);
        p_flash->p_OpData = p_dataResult; /* Sets p_OpData to result buffer */
        p_flash->FinalizeCmd = (NvMemory_FinalizeCmd_T)FinalizeCmdReadOnce;
    }

    return status;
}

Flash_Status_T Flash_SetEraseAll(Flash_T * p_flash)
{
    NvMemory_SetOpControl(p_flash, (HAL_NvMemory_StartCmd_T)StartCmdEraseAll, (NvMemory_Process_T)0U);
    return FLASH_STATUS_SUCCESS;
}

Flash_Status_T Flash_SetOp(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_data, size_t size, Flash_Operation_T opId)
{
    Flash_Status_T status;

    switch(opId)
    {
        case FLASH_OPERATION_WRITE:             status = Flash_SetWrite(p_flash, p_destFlash, p_data, size);                break;
        case FLASH_OPERATION_ERASE:             status = Flash_SetErase(p_flash, p_destFlash, size);                        break;
        case FLASH_OPERATION_VERIFY_WRITE:      status = Flash_SetVerifyWrite(p_flash, p_destFlash, p_data, size);          break;
        case FLASH_OPERATION_VERIFY_ERASE:      status = Flash_SetVerifyErase(p_flash, p_destFlash, size);                  break;
        case FLASH_OPERATION_WRITE_ONCE:        status = Flash_SetWriteOnce(p_flash, p_destFlash, p_data, size);            break;
        case FLASH_OPERATION_READ_ONCE:         status = Flash_SetReadOnce(p_flash, (uint8_t *)p_data, p_destFlash, size);  break;
        default:                                status = FLASH_STATUS_ERROR_INVALID_OP;                                     break;
    }

    return status;
}

/******************************************************************************/
/*!
    Blocking Implementations
    Proc Include Set and Activate
*/
/******************************************************************************/
Flash_Status_T Flash_ProcThisOp_Blocking(Flash_T * p_flash)
{
   volatile Flash_Status_T status;
    Critical_Enter(); /* Flash Op must not invoke isr table stored in flash */
    status = NvMemory_ProcOp_Blocking(p_flash);
    Critical_Exit();
    return status;
}

// uint8_t alignedData[FLASH_UNIT_WRITE_SIZE] = { [0U ... (FLASH_UNIT_WRITE_SIZE - 1U)] = FLASH_UNIT_ERASE_PATTERN };
// todo move to NvMemory, with flexible buffer
static Flash_Status_T WriteRemainder(Flash_T * p_flash, uint8_t unitSize)
{
    volatile Flash_Status_T status;
    uint8_t alignedData[FLASH_UNIT_WRITE_SIZE]; // assume FLASH_UNIT_WRITE_SIZE >= FLASH_UNIT_WRITE_ONCE_SIZE
    memcpy(&alignedData[0U], &p_flash->p_OpData[p_flash->OpSize], p_flash->OpSizeRemainder); /* start from remaining data */
    memset(&alignedData[p_flash->OpSizeRemainder], FLASH_UNIT_ERASE_PATTERN, unitSize - p_flash->OpSizeRemainder);
    p_flash->p_OpData = &alignedData[0U];
    p_flash->p_OpDest = p_flash->p_OpDest + p_flash->OpSize;
    p_flash->OpSize = unitSize;

    status = Flash_ProcThisOp_Blocking(p_flash);
    return status;
}

/* Shortcut check status proc */
static inline Flash_Status_T ProcAfterSet(Flash_T * p_flash, Flash_Status_T status)
{
    return (status == FLASH_STATUS_SUCCESS) ? Flash_ProcThisOp_Blocking(p_flash) : status;
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
Flash_Status_T Flash_Write_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
    volatile Flash_Status_T status = ProcAfterSet(p_flash, Flash_SetWrite(p_flash, p_destFlash, p_source, size));
    if(status == FLASH_STATUS_SUCCESS) { if(p_flash->OpSizeRemainder != 0U) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_SIZE); } }
    // if(p_flash->IsVerifyEnable == true) { status = ProcAfterSet(p_flash, Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size)); }
    return status;
}

/*
    Continue write starting from last p_dest
    Append Write Operation, incrementing p_OpDest
    Previous cmd must end on aligned boundry
    previsou verfy cmd maisints OpSize if also set to align down. todo
*/
Flash_Status_T Flash_ContinueWrite_Blocking(Flash_T * p_flash, const uint8_t * p_source, size_t size)
{
    assert(p_flash->OpSizeRemainder == 0);
    return Flash_Write_Blocking(p_flash, p_flash->p_OpDest + p_flash->OpSize, p_source, size);

    // Flash_Status_T status = FLASH_STATUS_SUCCESS;
    // if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpSizeAlignDown(p_flash, size, FLASH_UNIT_WRITE_SIZE); }
    // if(status == FLASH_STATUS_SUCCESS) { status = NvMemory_SetOpDataSource(p_flash, p_source, size); }
    // if(status == FLASH_STATUS_SUCCESS) { status = Flash_ProcThisOp_Blocking(p_flash); }
    // if(status == FLASH_STATUS_SUCCESS) /* If verify returns NV_MEMORY_STATUS_START_VERIFY */
    // {
    //     p_flash->p_OpDest += p_flash->OpSize;
    //     if(p_flash->IsVerifyEnable == true) { SetWriteControl(p_flash); } /* Restore write if verify */
    //     if(p_flash->OpSizeRemainder != 0U) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_SIZE); }
    // }
    // return status;
}

Flash_Status_T Flash_Erase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
    return ProcAfterSet(p_flash, Flash_SetErase(p_flash, p_destFlash, size));
}

Flash_Status_T Flash_VerifyWrite_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
#if defined(FLASH_UNIT_VERIFY_WRITE_SIZE) && (FLASH_UNIT_VERIFY_WRITE_SIZE != 0U)
    return ProcAfterSet(p_flash, Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size));
    // if(status == FLASH_STATUS_SUCCESS) { if(p_flash->OpSizeRemainder != 0U) { status = VerifyWriteRemainder(p_flash, FLASH_UNIT_WRITE_SIZE); } } //todo verify mains original size in case of continue write
#else  /* Manual compare if no hw verify implemented */
    return NvMemory_VerifyWrite(p_flash, p_destFlash, p_source, size) ? FLASH_STATUS_SUCCESS : FLASH_STATUS_ERROR_VERIFY;
#endif
}

Flash_Status_T Flash_VerifyErase_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
{
    return ProcAfterSet(p_flash, Flash_SetVerifyErase(p_flash, p_destFlash, size));
}

/* Caller ensure align. Alternatively write remainder filling with erase pattern which can be overwritten */
Flash_Status_T Flash_WriteOnce_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
{
    Flash_Status_T status = ProcAfterSet(p_flash, Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size));
    // if(status == FLASH_STATUS_SUCCESS) { if(p_flash->OpSizeRemainder != 0U) { status = WriteRemainder(p_flash, FLASH_UNIT_WRITE_ONCE_SIZE); } }
    return status;
}

Flash_Status_T Flash_ReadOnce_Blocking(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_destOnce, size_t size)
{
    return ProcAfterSet(p_flash, Flash_SetReadOnce(p_flash, p_dataResult, p_destOnce, size));
}

Flash_Status_T Flash_EraseAll_Blocking(Flash_T *p_flash)
{
    return ProcAfterSet(p_flash, Flash_SetEraseAll(p_flash));
}

Flash_Status_T Flash_ProcOp_Blocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size, Flash_Operation_T opId)
{
    return ProcAfterSet(p_flash, Flash_SetOp(p_flash, p_destFlash, p_source, size, opId));
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

// Flash_Status_T Flash_StartWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
// {
//     p_flash->Status = (Flash_SetWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
// {
//     p_flash->Status = (Flash_SetErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyWrite_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyWrite(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartVerifyErase_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, size_t size)
// {
//     p_flash->Status = (Flash_SetVerifyErase(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartWriteOnce_NonBlocking(Flash_T * p_flash, const uint8_t * p_destFlash, const uint8_t * p_source, size_t size)
// {
//     p_flash->Status = (Flash_SetWriteOnce(p_flash, p_destFlash, p_source, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }

// Flash_Status_T Flash_StartReadOnce_NonBlocking(Flash_T * p_flash, uint8_t * p_dataResult, const uint8_t * p_once, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, p_dataResult, p_once, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
//     return p_flash->Status;
// }
// void Flash_GetReadOnce(const Flash_T * p_flash, uint8_t * p_result)
// {
//     memcpy(p_result, &p_flash->CONFIG.P_BUFFER[0U], p_flash->OpSize);
// }
// Flash_Status_T Flash_StartReadOnce_Direct_NonBlocking(Flash_T * p_flash, uint8_t * p_destResult, const uint8_t * p_destFlash, size_t size)
// {
//     p_flash->Status = (Flash_SetReadOnce(p_flash, p_destFlash, size) == FLASH_STATUS_SUCCESS ? NvMemory_StartOp(p_flash) : FLASH_STATUS_ERROR_INPUT);
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

