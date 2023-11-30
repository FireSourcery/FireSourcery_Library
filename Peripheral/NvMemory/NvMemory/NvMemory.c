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

/*
    define control struct,
    use setcmd
*/

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline const uint8_t * OpCmdAddress(const NvMemory_T * p_this, const uint8_t * p_dest) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION; /* incase inline disabled by optimization level */
static inline const uint8_t * OpCmdAddress(const NvMemory_T * p_this, const uint8_t * p_dest)
{
#ifdef CONFIG_NV_MEMORY_HW_OP_ADDRESS_RELATIVE
    return (uint8_t *)((uint32_t)p_dest + p_this->p_OpPartition->OP_ADDRESS_OFFSET);
#elif defined(CONFIG_NV_MEMORY_HW_OP_ADDRESS_ABSOLUTE)
    (void)p_this; return p_dest;
#endif
}

static inline bool StartOpCmd(const NvMemory_T * p_this, size_t opIndex) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline bool StartOpCmd(const NvMemory_T * p_this, size_t opIndex)
{
    /* Start command validated by caller. No null pointer check (p_this->StartCmd != 0U) */
    p_this->StartCmd(p_this->CONFIG.P_HAL, OpCmdAddress(p_this, &p_this->p_OpDest[opIndex]), &p_this->p_OpData[opIndex], p_this->UnitsPerCmd);

    return (p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false);
}

/*!
    @param[in] num - address or size
    @param[in] align - unit msut be power of 2
*/
static inline uint32_t AlignDown(size_t num, size_t align) { return ((num) & -(align)); }
static inline uint32_t AlignUp(size_t num, size_t align) { return (-(-(num) & -(align))); }
static inline bool IsAligned(size_t num, size_t align) { return ((num & (align - 1U)) == 0UL); }

/******************************************************************************/
/*!
    Protected - Call from child-class only
        _NvMemory notation is omitted
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
    p_this->p_OpDest = 0U;
    p_this->OpIndex = 0U;
    p_this->OpSize = 0U;
    p_this->IsVerifyEnable = true;
    p_this->IsOpBuffered = (p_this->CONFIG.BUFFER_SIZE > 0U);
    p_this->IsFillAlignEnable = false;
}

bool NvMemory_IsAligned(size_t num, size_t align) { return IsAligned(num, align); }
bool NvMemory_IsPtrAligned(const void * ptr, size_t align) { return IsAligned((uintptr_t)ptr, align); }

void NvMemory_EnableFillAlign(NvMemory_T * p_this) { p_this->IsFillAlignEnable = true; }
void NvMemory_DisableFillAlign(NvMemory_T * p_this) { p_this->IsFillAlignEnable = false; }

void NvMemory_SetYield(NvMemory_T * p_this, NvMemory_Callback_T yield, void * p_callbackData)
{
    p_this->Yield = yield;
    p_this->p_CallbackContext = p_callbackData;
}

/******************************************************************************/
/*! Unchecked helper functions */
/******************************************************************************/
// typedef const struct NvMemory_OpControl
// {
//     HAL_NvMemory_StartCmd_T START_CMD;
//     NvMemory_FinalizeCmd_T FINALIZE_CMD;
//     NvMemory_Process_T PARSE_CMD_ERROR;
//     size_t UNIT_SIZE;
//     size_t CMD_UNITS;
//     size_t CMD_BYTES;
// }
// NvMemory_OpControl_T;

void NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd)
{
    p_this->UnitsPerCmd = unitsPerCmd; /* Most cases unit is 1 */
    p_this->BytesPerCmd = unitsPerCmd * unitSize;
}

void NvMemory_SetOpControl(NvMemory_T * p_this, HAL_NvMemory_StartCmd_T startCmd, NvMemory_Process_T parseCmdError)
{
    p_this->StartCmd = startCmd;
    p_this->ParseCmdError = parseCmdError;
    p_this->FinalizeCmd = 0U; /* Temp. only readonce use for now. */
}


/* When hw verify is not implemented */
bool NvMemory_VerifyWrite(const NvMemory_T * p_this, const uint8_t * p_dest, const uint8_t * p_source, size_t size)
{
    bool status = true;
    //todo +align
    for(size_t iByte = 0U; iByte < size; iByte++) { if(p_dest[iByte] != p_source[iByte]) { status = false; break; } }
    return status;
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
bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this, const uint8_t * p_source, size_t size)
{
    return (Checksum(p_this->p_OpDest, size) == Checksum(p_source, size));
}

/******************************************************************************/
/*!
    Conditional Sets
    Alternatively save errors to flag reg
*/
/******************************************************************************/
static inline bool CheckAddressBoundary(uint32_t targetStart, size_t targetSize, uint32_t boundaryStart, size_t boundarySize)
{
    return ((targetStart >= boundaryStart) && ((targetStart + targetSize) <= (boundaryStart + boundarySize)));
}

static inline bool CheckPartitionBoundary(const NvMemory_Partition_T * p_partition, const uint8_t * p_dest, size_t size)
{
    return CheckAddressBoundary((uint32_t)p_dest, size, (uint32_t)p_partition->P_START, p_partition->SIZE);
}

static inline NvMemory_Partition_T * SearchPartitionTable(const NvMemory_Partition_T * p_partitionTable, uint8_t partitionCount, const uint8_t * p_dest, size_t size)
{
    NvMemory_Partition_T * p_partition = 0U;

    for(uint8_t iPartition = 0U; iPartition < partitionCount; iPartition++)
    {
        if(CheckPartitionBoundary(&p_partitionTable[iPartition], p_dest, size) == true) { p_partition = &p_partitionTable[iPartition]; }
    }

    return p_partition;
}

/* Check all partitions for op fit */
static inline bool ValidateOpPartition(NvMemory_T * p_this, const uint8_t * p_dest, size_t size)
{
    p_this->p_OpPartition = SearchPartitionTable(p_this->CONFIG.P_PARTITIONS, p_this->CONFIG.PARTITION_COUNT, p_dest, size);
    return (p_this->p_OpPartition != 0U);
}

/* Todo template set with struct, containing unit size */
/*
    Checks Boundary and Dest Align
*/
NvMemory_Status_T NvMemory_SetOpDest(NvMemory_T * p_this, const uint8_t * p_dest, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status;

    if(ValidateOpPartition(p_this, p_dest, opSize) == true)
    {
        if(IsAligned((uintptr_t)p_dest, unitSize) == true)
        {
            p_this->p_OpDest = p_dest;
            status = NV_MEMORY_STATUS_SUCCESS;
        }
        else
        {
            status = NV_MEMORY_STATUS_ERROR_ALIGNMENT;
        }
    }
    else
    {
        status = NV_MEMORY_STATUS_ERROR_BOUNDARY;
    }

    return status;
}

/*
    Set Size
*/
/* Rejects unaligned */
NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_this, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status = (IsAligned(opSize, unitSize) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT;
    if(status == NV_MEMORY_STATUS_SUCCESS) { p_this->OpSize = opSize; }
    return status;
}

/* Align Down, for write. Remainder handled in calling function */
NvMemory_Status_T NvMemory_SetOpSizeAlignDown(NvMemory_T * p_this, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status;

    if(p_this->IsFillAlignEnable == true)
    {
        p_this->OpSize = AlignDown(opSize, unitSize);
        p_this->OpSizeRemainder = opSize - p_this->OpSize; // or matain opsize total
        status = NV_MEMORY_STATUS_SUCCESS;
    }
    else
    {
        status = NvMemory_SetOpSize(p_this, opSize, unitSize);
    }

    return status;
}

/* Align Up, for erase */
NvMemory_Status_T NvMemory_SetOpSizeAlignUp(NvMemory_T * p_this, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status;

    if(p_this->IsFillAlignEnable == true)
    {
        p_this->OpSize = AlignUp(opSize, unitSize);
        status = NV_MEMORY_STATUS_SUCCESS;
    }
    else
    {
        status = NvMemory_SetOpSize(p_this, opSize, unitSize);
    }

    return status;
}


static NvMemory_Status_T SetOpDataBuffer(NvMemory_T * p_this, const uint8_t * p_source, size_t size)
{
    NvMemory_Status_T status = (size <= p_this->CONFIG.BUFFER_SIZE) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_BUFFER;
    if(status == NV_MEMORY_STATUS_SUCCESS)
    {
        memcpy(p_this->CONFIG.P_BUFFER, p_source, size);
        p_this->p_OpData = p_this->CONFIG.P_BUFFER;
    }
    return status;
}

/*
    For Write
    Accepts p_source == 0 as NV_MEMORY_STATUS_SUCCESS
        use to indicate preliminary set, for dest boundary checking
*/
NvMemory_Status_T NvMemory_SetOpDataSource(NvMemory_T * p_this, const uint8_t * p_source, size_t size)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if(p_source != 0U)
    {
        if(p_this->IsOpBuffered == true) { status = SetOpDataBuffer(p_this, p_source, size); }
        else { p_this->p_OpData = p_source; }
    }

    return status;
}


/******************************************************************************/
/*!
    Blocking Implementations
*/
/******************************************************************************/
/*
    Store in RAM for case of Flash. SetCmd need not reside in RAM.
        However HAL may not support grainularity for SetCmd and StartCmd e.g.:
            p_this->SetCmd(p_this->CONFIG.P_HAL, OpCmdAddress(p_this, &p_this->p_OpDest[opIndex]), &p_this->p_OpData[opIndex], p_this->UnitsPerCmd)
            p_this->CONFIG.START_CMD(p_this->CONFIG.P_HAL)
    Implement as StartCmd
*/
static void ProcCmd_Blocking(NvMemory_T * p_this, size_t opIndex) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static void ProcCmd_Blocking(NvMemory_T * p_this, size_t opIndex)
{
    if(StartOpCmd(p_this, opIndex) == true)
    {
        while(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == false)
        {
            if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == true) { break; }
            if(p_this->Yield != 0U) { p_this->Yield(p_this->p_CallbackContext); }
        }
    }
}

NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true)
    {
        p_this->CONFIG.CLEAR_ERROR_FLAGS(p_this->CONFIG.P_HAL);

        //alternatively calc aligned opsize here. use control acess unit size
        for(size_t opIndex = 0U; opIndex < p_this->OpSize; opIndex += p_this->BytesPerCmd)
        {
            ProcCmd_Blocking(p_this, opIndex);
            // status = (p_this->ParseCmdStatus != 0U) ? p_this->ParseCmdStatus(p_this) : NV_MEMORY_STATUS_SUCCESS;
            if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == true)
            {
                status = (p_this->ParseCmdError != 0U) ? p_this->ParseCmdError(p_this) : NV_MEMORY_STATUS_ERROR_CMD;
                break;
            }
            if(p_this->FinalizeCmd != 0U) { status = p_this->FinalizeCmd(p_this, opIndex); }
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
// else  /* if (p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false) */
// {
//     status = p_this->FinalizeOp(p_this);

//     if(status == NV_MEMORY_STATUS_START_VERIFY)
//     {
//         /* FinalizeOp must set Verify Cmd */
//         status = NvMemory_ProcOp_Blocking(p_this);
//     }
// }

// size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_this) { return p_this->OpSize - p_this->OpIndex; }

// bool NvMemory_ReadIsOpComplete(NvMemory_T * p_this)
// {
//     return (p_this->State == NV_MEMORY_STATE_IDLE) && (p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true);
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
//             if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false)
//             {
//                 if(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true)
//                 {
//                     //check first time
// //                    if (p_this->OpIndex > 0U)
// //                    {
//     //                    if (p_this->OpType == NV_MEMORY_OPERATION_READ_ONCE)
//     //                    {
//     //                        HAL_NvMemory_ReadOnceData(p_this->CONFIG.P_HAL, &p_this->Buffer[p_this->OpIndex]);
//     //                    }
// //                    }

//                     if(p_this->OpIndex < p_this->OpSize)
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
//         p_this->CONFIG.CLEAR_ERROR_FLAGS(p_this->CONFIG.P_HAL);
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
 //    p_flash->p_OpDest = OpCmdAddress(p_flash, p_physical);
 //    p_flash->OpIndex = 0U;
 //    p_flash->OpSize = size;
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
 //    if (p_physical >= p_flash->p_OpDest && p_physical + size < p_flash->p_OpDest + p_flash->OpSize)
 //    {
 //        offset = p_physical - p_flash->p_OpDest;
 //        memcpy(&p_flash->Buffer[offset], p_src, size);
 //    }
 //}
 //
 //void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * p_dest, const uint8_t * p_physical, size_t size)
 //{
 //    uint32_t offset;
 //
 //    if (p_physical >= p_flash->p_OpDest && p_physical <= p_flash->p_OpDest + p_flash->OpSize)
 //    {
 //        offset = p_physical - p_flash->p_OpDest;
 //        memcpy(p_dest, &p_flash->Buffer[offset], size);
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
 //    if (Flash_Erase_Blocking(p_flash, p_flash->p_OpDest, p_flash->OpSize) == true)
 //    {
 //        if (Flash_Write_Blocking(p_flash, p_flash->p_OpDest, &p_flash->Buffer[0U], p_flash->OpSize) == true)
 //        {
 //            if (ChecksumOp(p_flash) == true)
 //            {
 //                if (p_flash->IsVerifyEnable == true)
 //                {
 //                    if (Flash_VerifyWrite_Blocking(p_flash, p_flash->p_OpDest, &p_flash->Buffer[0U], p_flash->OpSize) == true)
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


