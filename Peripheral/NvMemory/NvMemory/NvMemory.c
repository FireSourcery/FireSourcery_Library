/*******************************************************************************/
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
static inline const uint8_t * CalcOpCmdAddress(const NvMemory_T * p_this, const uint8_t * p_dest) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline const uint8_t * CalcOpCmdAddress(const NvMemory_T * p_this, const uint8_t * p_dest)
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
    // const uint8_t * p_cmdDest = CalcOpCmdAddress(p_this, &p_this->p_OpDest[opIndex]);
    // const uint8_t * p_cmdData = &p_this->p_OpData[opIndex]; /* Unused by non write type */
    // size_t units = p_this->UnitsPerCmd;

    /* Start command is known set. no null pointer check (p_this->StartCmd != 0U)*/
    // p_this->StartCmd(p_this->CONFIG.P_HAL, p_cmdDest, p_cmdData, units);
    /* typedef void (*HAL_NvMemory_StartCmd_T)(void * p_hal, const uint8_t * p_cmdDest, const uint8_t * p_cmdData, size_t units); */
    p_this->StartCmd(p_this->CONFIG.P_HAL, CalcOpCmdAddress(p_this, &p_this->p_OpDest[opIndex]), &p_this->p_OpData[opIndex], p_this->UnitsPerCmd);

    /* SetCmd need not reside in RAM  */
    // p_this->SetCmd(p_this->CONFIG.P_HAL, CalcOpCmdAddress(p_this, &p_this->p_OpDest[opIndex]), &p_this->p_OpData[opIndex], p_this->UnitsPerCmd);
    // p_this->CONFIG.START_CMD();

    return (p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false);
}


/*!
    @param[in] num - address or size
    @param[in] align - unit always power of 2
*/
static inline uint32_t CalcAlignDown(uint32_t num, uint32_t align) { return ((num) & -(align)); }
static inline uint32_t CalcAlignUp(uint32_t num, uint32_t align) { return (-(-(num) & -(align))); }
static inline bool CheckAligned(uint32_t num, uint32_t align) { return ((num & (align - 1U)) == 0UL); }

/******************************************************************************/
/*!
    Protected - Call from child-class only
        _NvMemory notation is omitted
    Set - Common Blocking Non Blocking
*/
/******************************************************************************/
/******************************************************************************/
/*!    Extensible Functions */
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
    p_this->IsForceAlignEnable = false;
}

void NvMemory_SetYield(NvMemory_T * p_this, void (*yield)(void *), void * p_callbackData)
{
    p_this->Yield = yield;
    p_this->p_CallbackData = p_callbackData;
}

void NvMemory_EnableForceAlign(NvMemory_T * p_this) { p_this->IsForceAlignEnable = true; }
void NvMemory_DisableForceAlign(NvMemory_T * p_this) { p_this->IsForceAlignEnable = false; p_this->ForceAlignBytes = 0U; }

/******************************************************************************/
/*!    Unchecked helper functions */
/******************************************************************************/
void NvMemory_SetOpCmdSize(NvMemory_T * p_this, size_t unitSize, uint8_t unitsPerCmd)
{
    p_this->UnitsPerCmd = unitsPerCmd;
    p_this->BytesPerCmd = unitsPerCmd * unitSize;
}

void NvMemory_SetOpFunctions(NvMemory_T * p_this, HAL_NvMemory_StartCmd_T startCmd, NvMemory_Process_T finalizeOp, NvMemory_Process_T parseCmdError)
{
    p_this->StartCmd = startCmd;
    p_this->FinalizeOp = finalizeOp;
    p_this->ParseCmdError = parseCmdError;
    p_this->FinalizeCmd = 0U; /* Temp. only readonce use for now. */
}

static inline uint32_t CalcChecksum(const uint8_t * p_data, size_t size)
{
    uint32_t sum = 0U;
    for(size_t iByte = 0U; iByte < size; iByte++) { sum += p_data[iByte]; }
    return sum;
}

/*
    return true if correct
    Non buffered must maintain source data pointed by p_OpData
*/
bool NvMemory_CheckOpChecksum(const NvMemory_T * p_this)
{
    return (CalcChecksum(&p_this->p_OpDest[0U], p_this->OpSize) == CalcChecksum(p_this->p_OpData, p_this->OpSize));
}

/******************************************************************************/
/*!
    Conditional Sets
    Alternatively save errors to flag reg
*/
/******************************************************************************/
// static inline bool CheckOpAligned(const uint8_t * p_dest, size_t size, size_t align)
// {
//     return (CheckAligned((uint32_t)p_dest, align) && CheckAligned(size, align));
// }

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
static inline bool CheckOpBoundary(NvMemory_T * p_this, const uint8_t * p_dest, size_t size)
{
    p_this->p_OpPartition = SearchPartitionTable(p_this->CONFIG.P_PARTITIONS, p_this->CONFIG.PARTITION_COUNT, p_dest, size);
    return (p_this->p_OpPartition != 0U);
}

/* Todo template set with struct */
/*
    Checks Boundary and Dest Align
*/
NvMemory_Status_T NvMemory_SetOpDest(NvMemory_T * p_this, const uint8_t * p_dest, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status;

    if(CheckOpBoundary(p_this, p_dest, opSize) == true)
    {
        if(CheckAligned((uint32_t)p_dest, unitSize) == true)
        {
            p_this->p_OpDest = p_dest;
            status = NV_MEMORY_STATUS_SUCCESS;
        }
        else
        {
            status = NV_MEMORY_STATUS_ERROR_ALIGNMENT;
        }

        // if(p_this->IsForceAlignEnable == true) /* Not need check align, always set ForceAlignBytes as indicator this way */
        // {
        //     p_this->p_OpDest = p_dest; /* temp cannot force align over boundary this way, todo  */
        //     // p_this->OpSize = CalcAlignDown(opSize, unitSize); /* Align down so we may uniquely process last cmd after loop */
        //     p_this->OpSize = CalcAlignDown(opSize, unitSize);
        //     p_this->ForceAlignBytes = opSize - CalcAlignDown(opSize, p_this->OpSize);
        //     status = NV_MEMORY_STATUS_SUCCESS;
        // }
        // else if(CheckOpAligned(p_dest, opSize, unitSize) == true) //CheckAligned((uint32_t)p_dest, align) && CheckAligned(size, align)
        // {
        //     p_this->p_OpDest = p_dest;
        //     p_this->OpSize = opSize;
        //     status = NV_MEMORY_STATUS_SUCCESS;
        // }
        // else
        // {
        //     status = NV_MEMORY_STATUS_ERROR_ALIGNMENT;
        // }
    }
    else
    {
        status = NV_MEMORY_STATUS_ERROR_BOUNDARY;
    }

    return status;
}

/*
    Unit is granularity. UnitsPerCmd is data size each cmd cycle
*/
// NvMemory_SetOpSizeNUnitsPerCmd(NvMemory_T * p_this, size_t opSize, size_t unitSize)
// static inline size_t CalcUnitsPerCmd(size_t opSize, uint8_t unitSize)
// {
//     return opSize / unitSize;
// }

NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_this, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status = (CheckAligned(opSize, unitSize) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT;
    if(status == NV_MEMORY_STATUS_SUCCESS) { p_this->OpSize = opSize; }
    return status;
}

/* Align Down, for write. Remainder handled in calling function */
NvMemory_Status_T NvMemory_SetOpSizeAlignDown(NvMemory_T * p_this, size_t opSize, size_t unitSize)
{
    NvMemory_Status_T status;

    if(p_this->IsForceAlignEnable == true)
    {
        p_this->OpSize = CalcAlignDown(opSize, unitSize);
        p_this->ForceAlignBytes = opSize - CalcAlignDown(opSize, p_this->OpSize);
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

    if(p_this->IsForceAlignEnable == true)
    {
        p_this->OpSize = CalcAlignUp(opSize, unitSize);
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
    if(status == NV_MEMORY_STATUS_SUCCESS) { memcpy(p_this->CONFIG.P_BUFFER, p_source, size);  p_this->p_OpData = p_this->CONFIG.P_BUFFER; }
    return status;
}

/*
    Accepts p_source == 0 as NV_MEMORY_STATUS_SUCCESS
        use to indicate preliminary set, for dest boundary checking
*/
NvMemory_Status_T NvMemory_SetOpDataWrite(NvMemory_T * p_this, const uint8_t * p_source, size_t size)
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
// static NvMemory_Status_T ProcOp_Blocking(NvMemory_T * p_this) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
// static NvMemory_Status_T ProcOp_Blocking(NvMemory_T * p_this)
// {
//     NvMemory_Status_T status;

//     p_this->CONFIG.START_CMD(p_this->CONFIG.P_HAL);
//     if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false)
//     {
//         while(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == false)
//         {
//             if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == true) { status = NV_MEMORY_STATUS_ERROR_CMD; break; }
//             if(p_this->Yield != 0U) { p_this->Yield(p_this->p_CallbackData); }
//         }
//         status = NV_MEMORY_STATUS_SUCCESS;
//     }
//     else
//     {
//         status = NV_MEMORY_STATUS_ERROR_CMD;
//     }

//     return status;
// }

NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_this)
{
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true)
    {
        p_this->CONFIG.CLEAR_ERROR_FLAGS(p_this->CONFIG.P_HAL);

        for(size_t opIndex = 0U; opIndex < p_this->OpSize; opIndex += p_this->BytesPerCmd)
        {
            // if(SetOpCmd(p_this, opIndex) == true)
            // {
            //     status = ProcOp_Blocking(p_this);
            //     if(status == NV_MEMORY_STATUS_SUCCESS)
            //     {
            //         if(p_this->FinalizeCmd != 0U)
            //         {
            //             status = p_this->FinalizeCmd(p_this, opIndex);
            //             if(status != NV_MEMORY_STATUS_SUCCESS) { break; }
            //         }
            //     }
            //     else { break; }
            // }
            // else { break; }

            if(StartOpCmd(p_this, opIndex) == true)
            {
                while(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == false)
                {
                    if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == true) { break; }
                    if(p_this->Yield != 0U) { p_this->Yield(p_this->p_CallbackData); }
                }

                if(p_this->FinalizeCmd != 0U)
                {
                    status = p_this->FinalizeCmd(p_this, opIndex);
                    if(status != NV_MEMORY_STATUS_SUCCESS) { break; }
                }
            }
            else
            {
                break; /* status = NV_MEMORY_STATUS_ERROR_CMD; */
            }
        }

        if (p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == true)
        {
            status = (p_this->ParseCmdError != 0U) ? p_this->ParseCmdError(p_this) : NV_MEMORY_STATUS_ERROR_CMD;
        }
        else  /* if (p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false) */
        {
            status = p_this->FinalizeOp(p_this);

            if(status == NV_MEMORY_STATUS_START_VERIFY)
            {
                /* FinalizeOp must set Verify Cmd */
                /* MISRA violation. single recursive call. Rationale: Known to terminate after a single call. */
                status = NvMemory_ProcOp_Blocking(p_this);
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
size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_this) { return p_this->OpSize - p_this->OpIndex; }

bool NvMemory_ReadIsOpComplete(NvMemory_T * p_this)
{
    return (p_this->State == NV_MEMORY_STATE_IDLE) && (p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true);
}

/*
    returns true when complete
*/
bool NvMemory_ProcOp(NvMemory_T * p_this)
{
    bool isComplete = false;

    switch(p_this->State)
    {
        case NV_MEMORY_STATE_IDLE:
            break;

        case NV_MEMORY_STATE_ACTIVE:
            //multithread use mutex
            if(p_this->CONFIG.READ_ERROR_FLAGS(p_this->CONFIG.P_HAL) == false)
            {
                if(p_this->CONFIG.READ_COMPLETE_FLAG(p_this->CONFIG.P_HAL) == true)
                {
                    //check first time
//                    if (p_this->OpIndex > 0U)
//                    {
    //                    if (p_this->OpType == NV_MEMORY_OPERATION_READ_ONCE)
    //                    {
    //                        HAL_NvMemory_ReadOnceData(p_this->CONFIG.P_HAL, &p_this->Buffer[p_this->OpIndex]);
    //                    }
//                    }

                    if(p_this->OpIndex < p_this->OpSize)
                    {
                        if(StartOpCmd(p_this, p_this->OpIndex) == true)
                        {
                            p_this->OpIndex += p_this->BytesPerCmd;
                        }
                    }
                    else  //all pages complete
                    {
                        isComplete = true;
                    }
                }
            }
            else //error occurred during command operation
            {
                isComplete = true;
            }

            if(isComplete == true)
            {
                p_this->Status = p_this->FinalizeOp(p_this);

                if(p_this->Status == NV_MEMORY_STATUS_START_VERIFY)
                {
                    p_this->Status = NV_MEMORY_STATUS_PROCESSING;
                    p_this->OpIndex = 0U;
                    isComplete = false;
                }
                else
                {
                    p_this->State = NV_MEMORY_STATE_IDLE;
                    isComplete = true;
                }
            }
            break;

        default:
            break;
    }

    return isComplete;
}

NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_this)
{
    if(NvMemory_ReadIsOpComplete(p_this) == true)
    {
        p_this->CONFIG.CLEAR_ERROR_FLAGS(p_this->CONFIG.P_HAL);
        p_this->State = NV_MEMORY_STATE_ACTIVE;
        p_this->Status = NV_MEMORY_STATUS_PROCESSING;
        p_this->OpIndex = 0U;
    }
    else
    {
        p_this->Status = NV_MEMORY_STATUS_ERROR_BUSY;
    }

    return p_this->Status;
}



/******************************************************************************/
/*!
    Virtual - creates copy of flash in buffer
*/
/******************************************************************************/
 //void Flash_OpenVirtual(Flash_T * p_flash, const uint8_t * p_physical, size_t size)
 //{
 //    memcpy(&p_flash->Buffer[0U], p_physical, size);
 //
 //    p_flash->p_OpDest = CalcOpCmdAddress(p_flash, p_physical);
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


