/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
#include "EEPROM.h"

/*
    HAL_NvMemory_StartCmd_T
*/
static void StartCmdWrite(void * p_hal, const void * p_cmdDest, const void * p_cmdData, size_t units)
{
    (void)units;
#if (EEPROM_UNIT_WRITE_SIZE == 4U)
    if(*(uint32_t *)p_cmdDest != (*(uint32_t *)p_cmdData))
#elif (EEPROM_UNIT_WRITE_SIZE == 2U)
    if(*(uint16_t *)p_cmdDest != (*(uint16_t *)p_cmdData))
#elif (EEPROM_UNIT_WRITE_SIZE == 1U)
    if(*(uint8_t *)p_cmdDest != (*(uint8_t *)p_cmdData))
#endif
    {
        HAL_EEPROM_StartCmdWriteUnit(p_hal, p_cmdDest, p_cmdData);
    }
    /* else finalizeOp will return no error and continue to next  */
}

static inline NvMemory_Status_T ParseCmdErrorWrite(EEPROM_T * p_eeprom)
{
    NvMemory_Status_T status;
    if(HAL_EEPROM_ReadErrorProtectionFlag(p_eeprom->CONFIG.P_HAL) == true)     { status = NV_MEMORY_STATUS_ERROR_PROTECTION; }
    else                                                                     { status = NV_MEMORY_STATUS_ERROR_CMD; }
    return status;
}

static inline NvMemory_Status_T FinalizeWrite(EEPROM_T * p_eeprom)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    /* Checksum Verify only */
    if(p_eeprom->IsVerifyEnable == true)
    {
        if(NvMemory_CheckOpChecksum(p_eeprom) == false) { status = NV_MEMORY_STATUS_ERROR_CHECKSUM; }
    }

    return status;
}

/*
    Blocking
*/
void EEPROM_Init_Blocking(EEPROM_T * p_eeprom)
{
    //todo isfirsttime, use startcmd template
    // if(HAL_EEPROM_ReadIsFirstTime(p_eeprom->CONFIG.P_HAL))    { EEPROM_ProgramPartition_Blocking(p_eeprom); }

    // NvMemory_Status_T status = EEPROM_SetInit(p_eeprom);
    // if(status == NV_MEMORY_STATUS_SUCCESS)
    // {
    //     status = NvMemory_ProcOp_Blocking(p_eeprom);

    HAL_EEPROM_Init_Blocking(p_eeprom->CONFIG.P_HAL);
    NvMemory_Init(p_eeprom);
    p_eeprom->IsVerifyEnable = false;
    p_eeprom->IsOpBuffered = false;
    p_eeprom->IsForceAlignEnable = false;
    while(HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL) == false);
}

// void EEPROM_ReadIsFirstTime(EEPROM_T * p_eeprom) { HAL_EEPROM_ReadIsFirstTime(p_eeprom->CONFIG.P_HAL); }

NvMemory_Status_T EEPROM_SetWrite(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes)
{
    NvMemory_Status_T status = NvMemory_SetOpDest(p_eeprom, p_dest, sizeBytes, EEPROM_UNIT_WRITE_SIZE);
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpSize(p_eeprom, sizeBytes, EEPROM_UNIT_WRITE_SIZE); }
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpDataWrite(p_eeprom, p_source, sizeBytes); }
    if(status == NV_MEMORY_STATUS_SUCCESS)
    {
        NvMemory_SetOpFunctions(p_eeprom, (HAL_NvMemory_StartCmd_T)StartCmdWrite, (NvMemory_Process_T)FinalizeWrite, (NvMemory_Process_T)ParseCmdErrorWrite);
        NvMemory_SetOpCmdSize(p_eeprom, EEPROM_UNIT_WRITE_SIZE, 1U);
    }
    return status;
}

NvMemory_Status_T EEPROM_Write_Blocking(EEPROM_T * p_eeprom, const void * p_dest, const void * p_source, size_t sizeBytes)
{
    NvMemory_Status_T status = EEPROM_SetWrite(p_eeprom, p_dest, p_source, sizeBytes);
    return (status == NV_MEMORY_STATUS_SUCCESS) ? NvMemory_ProcOp_Blocking(p_eeprom) : status;
}

/*
    Non Blocking - UNTESTED
*/
// void EEPROM_Init_NonBlocking(EEPROM_T * p_eeprom) { HAL_EEPROM_Init_NonBlocking(p_eeprom->CONFIG.P_HAL); }
// bool EEPROM_ProcOp_NonBlocking(EEPROM_T * p_flash) { return NvMemory_ProcOp(p_flash); }
// bool EEPROM_ReadIsOpComplete(EEPROM_T * p_eeprom) { return HAL_EEPROM_ReadCompleteFlag(p_eeprom->CONFIG.P_HAL); }
// NvMemory_Status_T EEPROM_StartWrite_NonBlocking(EEPROM_T * p_eeprom, void * p_dest, const void * p_source, size_t sizeBytes)
// {
//     return (p_eeprom->Status = (EEPROM_SetWrite(p_eeprom, p_dest, p_source, sizeBytes) == NV_MEMORY_STATUS_SUCCESS ? NvMemory_StartOp(p_eeprom) : NV_MEMORY_STATUS_ERROR_INPUT));
// }



//static void StartCmdProgramPartition(void * p_hal, const void * p_cmdDest, const void * p_cmdData, size_t units)
//{
//    (void)units;
//    (void)p_cmdDest;
//    (void)p_cmdData;
//    HAL_EEPROM_ProgramPartition(p_hal);
//}
