/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/*******************************************************************************/
/******************************************************************************/
/*!
    Abstraction details
    Shared completion and error status -> simultaneous operations not supported.
*/
/******************************************************************************/
#include "NvMemory.h"
#include <string.h>

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline uintptr_t OpCmdAddress(const NvMemory_T * p_this, uintptr_t memAddress) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION; /* incase inline disabled by optimization level */
static inline uintptr_t OpCmdAddress(const NvMemory_T * p_this, uintptr_t memAddress)
{
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE
    return (memAddress + p_this->p_OpPartition->OP_ADDRESS_OFFSET);
#elif defined(CONFIG_NV_MEMORY_HW_OP_ADDRESS_ABSOLUTE)
    (void)p_this; return memAddress;
#endif
}

static inline bool StartOpCmd(const NvMemory_T * p_this, size_t opIndex) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline bool StartOpCmd(const NvMemory_T * p_this, size_t opIndex)
{
    /* Start command validated by caller. No null pointer check */
    p_this->p_OpControl->START_CMD(p_this->CONST.P_HAL, OpCmdAddress(p_this, p_this->OpAddress + opIndex), &p_this->p_OpData[opIndex], 1U);
    return (p_this->CONST.READ_ERROR_FLAGS(p_this->CONST.P_HAL) == false);
}

/******************************************************************************/
/*!
    Protected - Call from child-class only
    Set - Common Blocking Non Blocking
*/
/******************************************************************************/
/******************************************************************************/
/*! Extensible Functions */
/******************************************************************************/
void NvMemory_Init(NvMemory_T * p_this)
{
    p_this->Status = NV_MEMORY_STATUS_SUCCESS;
    p_this->State = NV_MEMORY_STATE_IDLE;
    p_this->OpAddress = 0U;
    p_this->OpIndex = 0U;
    p_this->OpSizeAligned = 0U;
    p_this->IsVerifyEnable = true;
    p_this->IsOpBuffered = (p_this->CONST.BUFFER_SIZE > 0U);
    p_this->IsForceAlignEnable = false;
}

/******************************************************************************/
/*!   */
/******************************************************************************/
/* When hw verify is not implemented */
NvMemory_Status_T NvMemory_MemCompare(const uint8_t * p_dest, const uint8_t * p_data, size_t size)
{
    return (memcmp(p_dest, p_data, size) == 0U) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_VERIFY;
}

static uint32_t Checksum(const uint8_t * p_data, size_t size)
{
    uint32_t sum = 0U;
    for(size_t iByte = 0U; iByte < size; iByte++) { sum += p_data[iByte]; }
    return sum;
}

/*
    Non buffered must maintain source data pointed by p_OpData
*/
bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this, const uint8_t * p_data, size_t size)
{
    return (Checksum((const uint8_t *)p_this->OpAddress, size) == Checksum(p_data, size));
}

/******************************************************************************/
/*!
    Conditional Sets
    Alternatively save errors to flag reg
*/
/******************************************************************************/
static inline bool CheckAddressBoundary(uintptr_t targetStart, size_t targetSize, uintptr_t boundaryStart, size_t boundarySize)
{
    return ((targetStart >= boundaryStart) && ((targetStart + targetSize) <= (boundaryStart + boundarySize)));
}

static inline bool CheckPartitionBoundary(const NvMemory_Partition_T * p_partition, uintptr_t memAddress, size_t size)
{
    return CheckAddressBoundary(memAddress, size, p_partition->ADDRESS, p_partition->SIZE);
}

static inline NvMemory_Partition_T * SearchPartitionTable(const NvMemory_Partition_T * p_partitionTable, uint8_t partitionCount, uintptr_t memAddress, size_t size)
{
    NvMemory_Partition_T * p_partition = 0U;

    for(uint8_t iPartition = 0U; iPartition < partitionCount; iPartition++)
    {
        if(CheckPartitionBoundary(&p_partitionTable[iPartition], memAddress, size) == true) { p_partition = &p_partitionTable[iPartition]; }
    }

    return p_partition;
}

/* Check all partitions for op fit */
static inline bool ValidateOpPartition(NvMemory_T * p_this, uintptr_t memAddress, size_t size)
{
    p_this->p_OpPartition = SearchPartitionTable(p_this->CONST.P_PARTITIONS, p_this->CONST.PARTITION_COUNT, memAddress, size);
    return (p_this->p_OpPartition != 0U);
}

// static size_t NvMemory_GetOpSizeAligned(NvMemory_T * p_this)     { p_this->p_OpControl->FORCE_ALIGN(p_this->OpSize, p_this->p_OpControl->UNIT_SIZE); }
// static size_t NvMemory_GetOpSizeRemainder(NvMemory_T * p_this)   { OpSize - nvmemory_align_down(OpSize) }

/*
    Checks Boundary and Dest Align
    p_this->p_OpControl must be set
*/
NvMemory_Status_T NvMemory_SetOpAddress(NvMemory_T * p_this, uintptr_t memAddress, size_t opSize)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    p_this->OpAddress = memAddress;
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = (ValidateOpPartition(p_this, memAddress, opSize) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_BOUNDARY; }
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = (nvmemory_is_aligned(memAddress, p_this->p_OpControl->UNIT_SIZE) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT; }
    return status;
}

NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_this, size_t opSize)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    p_this->OpSize = opSize;
    if((p_this->IsForceAlignEnable == true) && (p_this->p_OpControl->FORCE_ALIGN != NULL))
    {
        p_this->OpSizeAligned = p_this->p_OpControl->FORCE_ALIGN(opSize, p_this->p_OpControl->UNIT_SIZE); // alternatively use getter
    }
    else
    {
        status = (nvmemory_is_aligned(opSize, p_this->p_OpControl->UNIT_SIZE) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT;
        p_this->OpSizeAligned = p_this->OpSize;  /* Not in effect when status returns error */
    }
    return status;
}

/*
    For Write, or buffered Read
    Accepts p_data == NULL as NV_MEMORY_STATUS_SUCCESS
*/
NvMemory_Status_T NvMemory_SetOpData(NvMemory_T * p_this, const uint8_t * p_data, size_t opSize)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if((p_data != NULL) && (p_this->IsOpBuffered == true))
    {
        status = (opSize <= p_this->CONST.BUFFER_SIZE) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_BUFFER;
        if(status == NV_MEMORY_STATUS_SUCCESS)
        {
            memcpy(p_this->CONST.P_BUFFER, p_data, opSize);
            p_this->p_OpData = p_this->CONST.P_BUFFER;
        }
    }
    else
    {
        p_this->p_OpData = p_data;
    }

    return status;
}

NvMemory_Status_T NvMemory_SetOpControl(NvMemory_T * p_this, const NvMemory_OpControl_T * p_opControl, uintptr_t memAddress, const uint8_t * p_data, size_t size)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    p_this->p_OpControl = p_opControl;
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpAddress(p_this, memAddress, size); }
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpSize(p_this, size); }
    if(status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpData(p_this, p_data, size); }
    return status;
}

// NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd)
// NvMemory_Status_T NvMemory_SetOpSizeUnitsPerCmd(NvMemory_T * p_this, size_t opSize)
// {
//     //overwrite bytepercmd
// }


/******************************************************************************/
/*!
    Blocking Implementations
*/
/******************************************************************************/
/*
    Store in RAM for case of Flash.
*/
static void ProcCmd_Blocking(NvMemory_T * p_this, size_t opIndex) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static void ProcCmd_Blocking(NvMemory_T * p_this, size_t opIndex)
{
    if(StartOpCmd(p_this, opIndex) == true)
    {
        while(p_this->CONST.READ_COMPLETE_FLAG(p_this->CONST.P_HAL) == false)
        {
            if(p_this->CONST.READ_ERROR_FLAGS(p_this->CONST.P_HAL) == true) { break; }
            if(p_this->Yield != NULL) { p_this->Yield(p_this->p_CallbackContext); }
        }
    }
}

NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION; // potentially remove from RAM
NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if(p_this->CONST.READ_COMPLETE_FLAG(p_this->CONST.P_HAL) == true)
    {
        p_this->CONST.CLEAR_ERROR_FLAGS(p_this->CONST.P_HAL);
        for(size_t opIndex = 0U; opIndex < p_this->OpSizeAligned; opIndex += p_this->p_OpControl->UNIT_SIZE)
        {
            ProcCmd_Blocking(p_this, opIndex);
            if(p_this->CONST.READ_ERROR_FLAGS(p_this->CONST.P_HAL) == true)
            {
                status = (p_this->p_OpControl->PARSE_CMD_ERROR != NULL) ? p_this->p_OpControl->PARSE_CMD_ERROR(p_this) : NV_MEMORY_STATUS_ERROR_CMD;
                break;
            }
            if(p_this->p_OpControl->FINALIZE_CMD != NULL)
            {
                p_this->p_OpControl->FINALIZE_CMD(p_this->CONST.P_HAL, OpCmdAddress(p_this, p_this->OpAddress + opIndex), 1U, (uint8_t *)&p_this->p_OpData[opIndex]);
            }
        }
    }
    else
    {
        status = NV_MEMORY_STATUS_ERROR_BUSY;
    }

    return status;
}



/******************************************************************************/
/*!
    Non Blocking - UNTESTED
*/
/******************************************************************************/
// else  /* if (p_this->CONST.READ_ERROR_FLAGS(p_this->CONST.P_HAL) == false) */
// {
//     status = p_this->FinalizeOp(p_this);

//     if(status == NV_MEMORY_STATUS_START_VERIFY)
//     {
//         /* FinalizeOp must set Verify Cmd */
//         status = NvMemory_ProcOp_Blocking(p_this);
//     }
// }

// size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_this) { return p_this->OpSizeAligned - p_this->OpIndex; }

// bool NvMemory_ReadIsOpComplete(NvMemory_T * p_this)
// {
//     return (p_this->State == NV_MEMORY_STATE_IDLE) && (p_this->CONST.READ_COMPLETE_FLAG(p_this->CONST.P_HAL) == true);
// }

// /*
//     returns true when complete
// */
// bool NvMemory_ProcOp(NvMemory_T * p_this)
// {
//     bool isComplete = false;

//     switch(p_this->State)
//     {
//         case NV_MEMORY_STATE_IDLE:
//             break;

//         case NV_MEMORY_STATE_ACTIVE:
//             //multithread use mutex
//             if(p_this->CONST.READ_ERROR_FLAGS(p_this->CONST.P_HAL) == false)
//             {
//                 if(p_this->CONST.READ_COMPLETE_FLAG(p_this->CONST.P_HAL) == true)
//                 {
//                     //check first time
// //                    if (p_this->OpIndex > 0U)
// //                    {
//     //                    if (p_this->OpType == NV_MEMORY_OPERATION_READ_ONCE)
//     //                    {
//     //                        HAL_NvMemory_ReadOnceData(p_this->CONST.P_HAL, &p_this->Buffer[p_this->OpIndex]);
//     //                    }
// //                    }

//                     if(p_this->OpIndex < p_this->OpSizeAligned)
//                     {
//                         if(StartOpCmd(p_this, p_this->OpIndex) == true)
//                         {
//                             p_this->OpIndex += p_this->BytesPerCmd;
//                         }
//                     }
//                     else  //all pages complete
//                     {
//                         isComplete = true;
//                     }
//                 }
//             }
//             else //error occurred during command operation
//             {
//                 isComplete = true;
//             }

//             if(isComplete == true)
//             {
//                 p_this->Status = p_this->FinalizeOp(p_this);

//                 if(p_this->Status == NV_MEMORY_STATUS_START_VERIFY)
//                 {
//                     p_this->Status = NV_MEMORY_STATUS_PROCESSING;
//                     p_this->OpIndex = 0U;
//                     isComplete = false;
//                 }
//                 else
//                 {
//                     p_this->State = NV_MEMORY_STATE_IDLE;
//                     isComplete = true;
//                 }
//             }
//             break;

//         default:
//             break;
//     }

//     return isComplete;
// }

// NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_this)
// {
//     if(NvMemory_ReadIsOpComplete(p_this) == true)
//     {
//         p_this->CONST.CLEAR_ERROR_FLAGS(p_this->CONST.P_HAL);
//         p_this->State = NV_MEMORY_STATE_ACTIVE;
//         p_this->Status = NV_MEMORY_STATUS_PROCESSING;
//         p_this->OpIndex = 0U;
//     }
//     else
//     {
//         p_this->Status = NV_MEMORY_STATUS_ERROR_BUSY;
//     }

//     return p_this->Status;
// }


/******************************************************************************/
/*!
    Virtual - creates copy of flash in buffer
*/
/******************************************************************************/
 //void Flash_OpenVirtual(Flash_T * p_flash, const uint8_t * p_physical, size_t size)
 //{
 //    memcpy(&p_flash->Buffer[0U], p_physical, size);
 //
 //    p_flash->OpAddress = OpCmdAddress(p_flash, p_physical);
 //    p_flash->OpIndex = 0U;
 //    p_flash->OpSizeAligned = size;
 //
 //}
 //
 ///*
 // * dest of physical flash location
 // */
 //void Flash_WriteVirtual(Flash_T * p_flash, const uint8_t * p_physical, const uint8_t * p_src, size_t size)
 //{
 //    uint32_t offset;
 //
 //    if (p_physical >= p_flash->OpAddress && p_physical + size < p_flash->OpAddress + p_flash->OpSizeAligned)
 //    {
 //        offset = p_physical - p_flash->OpAddress;
 //        memcpy(&p_flash->Buffer[offset], p_src, size);
 //    }
 //}
 //
 //void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * memAddress, const uint8_t * p_physical, size_t size)
 //{
 //    uint32_t offset;
 //
 //    if (p_physical >= p_flash->OpAddress && p_physical <= p_flash->OpAddress + p_flash->OpSizeAligned)
 //    {
 //        offset = p_physical - p_flash->OpAddress;
 //        memcpy(memAddress, &p_flash->Buffer[offset], size);
 //    }
 //}
 //
 //void Flash_CloseVirtual(Flash_T * p_flash)
 //{
 //    p_flash->State = FLASH_STATE_WRITE;
 //}
 //
 //bool Flash_CloseVirtual_Blocking(Flash_T * p_flash)
 //{
 //    bool isSuccess = false;
 //
 //    if (Flash_Erase_Blocking(p_flash, p_flash->OpAddress, p_flash->OpSizeAligned) == true)
 //    {
 //        if (Flash_Write_Blocking(p_flash, p_flash->OpAddress, &p_flash->Buffer[0U], p_flash->OpSizeAligned) == true)
 //        {
 //            if (ChecksumOp(p_flash) == true)
 //            {
 //                if (p_flash->IsVerifyEnable == true)
 //                {
 //                    if (Flash_VerifyWrite_Blocking(p_flash, p_flash->OpAddress, &p_flash->Buffer[0U], p_flash->OpSizeAligned) == true)
 //                    {
 //                        isSuccess = true;
 //                    }
 //                }
 //                else
 //                {
 //                    isSuccess = true;
 //                }
 //            }
 //        }
 //    }
 //
 //    return isSuccess;
 //}


