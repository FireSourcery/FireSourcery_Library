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

*/
/*******************************************************************************/
#include "NvMemory.h"
#include <string.h>

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
static inline uintptr_t OpCmdAddress(const NvMemory_T * p_context, const NvMemory_State_T * p_state, uintptr_t address) NV_MEMORY_ATTRIBUTE_RAM_SECTION; /* incase inline disabled by optimization level */
static inline uintptr_t OpCmdAddress(const NvMemory_T * p_context, const NvMemory_State_T * p_state, uintptr_t address)
{
#ifdef NV_MEMORY_HW_OP_ADDRESS_RELATIVE
    return (address + p_state->p_OpPartition->OP_ADDRESS_OFFSET);
#elif defined(NV_MEMORY_HW_OP_ADDRESS_ABSOLUTE)
    (void)p_context; (void)p_state; return address;
#endif
}

static inline bool StartOpCmd(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex) NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline bool StartOpCmd(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex)
{
    /* Start command validated by caller. No null pointer check */
    p_state->p_OpControl->START_CMD(p_context->P_HAL, OpCmdAddress(p_context, p_state, p_state->OpAddress + opIndex), &((const uint8_t *)p_state->p_OpData)[opIndex], 1U);
    return (p_context->READ_ERROR_FLAGS(p_context->P_HAL) == false);
}

/******************************************************************************/
/*!
    Protected - Call from child-class only
    Set - Common Blocking Non Blocking
*/
/******************************************************************************/
/******************************************************************************/
/*!
    Blocking Implementations
*/
/******************************************************************************/
/*
    Store in RAM for case of Flash.
*/
// static __attribute__((noinline)) void ProcCmd_Blocking(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex) NV_MEMORY_ATTRIBUTE_RAM_SECTION;
// static __attribute__((noinline)) void ProcCmd_Blocking(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex)
static void ProcCmd_Blocking(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex) NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static void ProcCmd_Blocking(const NvMemory_T * p_context, const NvMemory_State_T * p_state, size_t opIndex)
{
    if (StartOpCmd(p_context, p_state, opIndex) == true)
    {
        while (p_context->READ_COMPLETE_FLAG(p_context->P_HAL) == false)
        {
            if (p_context->READ_ERROR_FLAGS(p_context->P_HAL) == true) { break; }
            if (p_state->Yield != NULL) { p_state->Yield(p_state->p_CallbackContext); }
        }
    }
}

// NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_context) NV_MEMORY_ATTRIBUTE_RAM_SECTION; // potentially remove from RAM
NvMemory_Status_T NvMemory_ProcOp_Blocking(NvMemory_T * p_context)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_OpControl_T * p_opControl = p_state->p_OpControl;
    volatile NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if (p_context->READ_COMPLETE_FLAG(p_context->P_HAL) == true)
    {
        p_context->CLEAR_ERROR_FLAGS(p_context->P_HAL);
        for (size_t opIndex = 0U; opIndex < p_state->OpSizeAligned; opIndex += p_opControl->UNIT_SIZE)
        {
            ProcCmd_Blocking(p_context, p_state, opIndex);
            if (p_context->READ_ERROR_FLAGS(p_context->P_HAL) == true)
            {
                status = (p_opControl->PARSE_CMD_ERROR != NULL) ? p_opControl->PARSE_CMD_ERROR(p_context->P_HAL) : NV_MEMORY_STATUS_ERROR_CMD;
                break;
            }
            if (p_opControl->FINALIZE_CMD != NULL)
            {
                p_opControl->FINALIZE_CMD(p_context->P_HAL, OpCmdAddress(p_context, p_state, p_state->OpAddress + opIndex), 1U, &((uint8_t *)p_state->p_OpData)[opIndex]);
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
    Base Init
*/
/******************************************************************************/
void NvMemory_Init(NvMemory_T * p_context)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    p_state->IsOpBuffered = (p_context->BUFFER_SIZE > 0U);
    p_state->IsVerifyEnable = true;
    p_state->IsForceAlignEnable = false;

    // p_state->Status = NV_MEMORY_STATUS_SUCCESS;
    // p_state->State = NV_MEMORY_STATE_IDLE;
    // p_state->OpAddress = 0U;
    // p_state->OpSizeAligned = 0U;
    // p_state->OpIndex = 0U;
}

/******************************************************************************/
/*!   */
/******************************************************************************/
/* When hw verify is not implemented */
NvMemory_Status_T NvMemory_MemCompare(const void * p_dest, const void * p_data, size_t size)
{
    return (memcmp(p_dest, p_data, size) == 0U) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_VERIFY;
}

// static uint32_t Checksum(const uint8_t * p_data, size_t size)
// {
//     uint32_t sum = 0U;
//     for (size_t iByte = 0U; iByte < size; iByte++) { sum += p_data[iByte]; }
//     return sum;
// }

// /*
//     Non buffered must maintain source data pointed by p_OpData
// */
// bool NvMemory_CheckOpChecksum(const NvMemory_T * p_context, const void * p_data, size_t size)
// {
//     return (Checksum((const uint8_t *)p_state->OpAddress, size) == Checksum((const uint8_t *)p_data, size));
// }

/******************************************************************************/
/*!
    Conditional Sets
    Alternatively save errors to state var
*/
/******************************************************************************/
static inline bool CheckAddressBoundary(uintptr_t targetStart, size_t targetSize, uintptr_t boundaryStart, size_t boundarySize)
{
    return ((targetStart >= boundaryStart) && ((targetStart + targetSize) <= (boundaryStart + boundarySize)));
}

static inline bool CheckPartitionBoundary(const NvMemory_Partition_T * p_partition, uintptr_t address, size_t size)
{
    return CheckAddressBoundary(address, size, p_partition->ADDRESS, p_partition->SIZE);
}

static inline const NvMemory_Partition_T * SearchPartitionTable(const NvMemory_Partition_T * p_partitionTable, uint8_t partitionCount, uintptr_t address, size_t size)
{
    const NvMemory_Partition_T * p_partition = NULL;

    for (uint8_t iPartition = 0U; iPartition < partitionCount; iPartition++)
    {
        if (CheckPartitionBoundary(&p_partitionTable[iPartition], address, size) == true) { p_partition = &p_partitionTable[iPartition]; break; }
    }

    return p_partition;
}

/* Check all partitions for op fit */
static inline bool ValidateOpPartition(NvMemory_T * p_context, uintptr_t address, size_t size)
{
    // p_state->p_OpPartition = SearchPartitionTable(p_context->P_PARTITIONS, p_context->PARTITION_COUNT, address, size);
    // return (p_state->p_OpPartition != 0U);
    return (SearchPartitionTable(p_context->P_PARTITIONS, p_context->PARTITION_COUNT, address, size) != NULL);
}


/*
    Checks Boundary and Dest Align
    p_state->p_OpControl must be set
*/
NvMemory_Status_T NvMemory_SetOpAddress(NvMemory_T * p_context, uintptr_t address, size_t size)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    p_state->OpAddress = address;
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = (ValidateOpPartition(p_context, address, size) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_BOUNDARY; }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = (nvmemory_is_aligned(address, p_state->p_OpControl->UNIT_SIZE) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT; }
    return status;
}

NvMemory_Status_T NvMemory_SetOpSize(NvMemory_T * p_context, size_t size)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status;
    p_state->OpSize = size;
    if ((p_state->IsForceAlignEnable == true) && (p_state->p_OpControl->FORCE_ALIGN != NULL))
    {
        p_state->OpSizeAligned = p_state->p_OpControl->FORCE_ALIGN(size, p_state->p_OpControl->UNIT_SIZE); // alternatively use getter
        status = NV_MEMORY_STATUS_SUCCESS;
    }
    else
    {
        p_state->OpSizeAligned = p_state->OpSize;  /* Not in effect when status returns error */
        status = (nvmemory_is_aligned(size, p_state->p_OpControl->UNIT_SIZE) == true) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_ALIGNMENT;
    }
    return status;
}

/*
    For Write, or buffered Read
    Accepts p_data == NULL as NV_MEMORY_STATUS_SUCCESS
*/
NvMemory_Status_T NvMemory_SetOpData(NvMemory_T * p_context, const void * p_data, size_t size)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;

    if ((p_data != NULL) && (p_state->IsOpBuffered == true))
    {
        status = (size <= p_context->BUFFER_SIZE) ? NV_MEMORY_STATUS_SUCCESS : NV_MEMORY_STATUS_ERROR_BUFFER;
        if (status == NV_MEMORY_STATUS_SUCCESS)
        {
            memcpy(p_context->P_BUFFER, p_data, size);
            p_state->p_OpData = p_context->P_BUFFER;
        }
    }
    else
    {
        p_state->p_OpData = p_data;
    }

    return status;
}


NvMemory_Status_T NvMemory_SetOpControl(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size)
{
    assert(p_opControl != NULL); // p_opControl must not be NULL

    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    p_state->p_OpControl = p_opControl;
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpAddress(p_context, address, size); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpSize(p_context, size); }
    return status;
}

NvMemory_Status_T NvMemory_SetOpControl_Read(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, size_t size, void * p_data)
{
    NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpControl(p_context, p_opControl, address, size); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { p_state->p_OpData = p_data; }
    return status;
}


NvMemory_Status_T NvMemory_SetOpControl_Write(NvMemory_T * p_context, const NvMemory_OpControl_T * p_opControl, uintptr_t address, const void * p_data, size_t size)
{
    // NvMemory_State_T * p_state = p_context->P_STATE;
    NvMemory_Status_T status = NV_MEMORY_STATUS_SUCCESS;
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpControl(p_context, p_opControl, address, size); }
    if (status == NV_MEMORY_STATUS_SUCCESS) { status = NvMemory_SetOpData(p_context, p_data, size); }
    return status;
}



// determine bytes per cmd
// NvMemory_SetOpCmdSize(NvMemory_T * p_context, size_t unitSize, uint8_t unitsPerCmd)
// NvMemory_Status_T NvMemory_SetOpSizeUnitsPerCmd(NvMemory_T * p_context, size_t opSize)
// {
//     //overwrite bytepercmd
// }




/******************************************************************************/
/*!
    Non Blocking - Experimental
*/
/******************************************************************************/
// else  /* if (p_context->READ_ERROR_FLAGS(p_context->P_HAL) == false) */
// {
//     status = p_state->FinalizeOp(p_state);

//     if(status == NV_MEMORY_STATUS_START_VERIFY)
//     {
//         /* FinalizeOp must set Verify Cmd */
//         status = NvMemory_ProcOp_Blocking(p_state);
//     }
// }

// size_t NvMemory_GetOpBytesRemaining(NvMemory_T * p_context) { return p_state->OpSizeAligned - p_state->OpIndex; }

// bool NvMemory_ReadIsOpComplete(NvMemory_T * p_context)
// {
//     return (p_state->State == NV_MEMORY_STATE_IDLE) && (p_context->READ_COMPLETE_FLAG(p_context->P_HAL) == true);
// }

// /*
//     returns true when complete
// */
// bool NvMemory_ProcOp(NvMemory_T * p_context)
// {
//     bool isComplete = false;

//     switch(p_state->State)
//     {
//         case NV_MEMORY_STATE_IDLE:
//             break;

//         case NV_MEMORY_STATE_ACTIVE:
//             //multithread use mutex
//             if(p_context->READ_ERROR_FLAGS(p_context->P_HAL) == false)
//             {
//                 if(p_context->READ_COMPLETE_FLAG(p_context->P_HAL) == true)
//                 {
//                     //check first time
// //                    if (p_state->OpIndex > 0U)
// //                    {
//     //                    if (p_state->OpType == NV_MEMORY_OPERATION_READ_ONCE)
//     //                    {
//     //                        HAL_NvMemory_ReadOnceData(p_context->P_HAL, &p_state->Buffer[p_state->OpIndex]);
//     //                    }
// //                    }

//                     if(p_state->OpIndex < p_state->OpSizeAligned)
//                     {
//                         if(StartOpCmd(p_state, p_state->OpIndex) == true)
//                         {
//                             p_state->OpIndex += p_state->BytesPerCmd;
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
//                 p_state->Status = p_state->FinalizeOp(p_state);

//                 if(p_state->Status == NV_MEMORY_STATUS_START_VERIFY)
//                 {
//                     p_state->Status = NV_MEMORY_STATUS_PROCESSING;
//                     p_state->OpIndex = 0U;
//                     isComplete = false;
//                 }
//                 else
//                 {
//                     p_state->State = NV_MEMORY_STATE_IDLE;
//                     isComplete = true;
//                 }
//             }
//             break;

//         default:
//             break;
//     }

//     return isComplete;
// }

// NvMemory_Status_T NvMemory_StartOp(NvMemory_T * p_context)
// {
//     if(NvMemory_ReadIsOpComplete(p_state) == true)
//     {
//         p_context->CLEAR_ERROR_FLAGS(p_context->P_HAL);
//         p_state->State = NV_MEMORY_STATE_ACTIVE;
//         p_state->Status = NV_MEMORY_STATUS_PROCESSING;
//         p_state->OpIndex = 0U;
//     }
//     else
//     {
//         p_state->Status = NV_MEMORY_STATUS_ERROR_BUSY;
//     }

//     return p_state->Status;
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
 //void Flash_ReadVirtual(Flash_T * p_flash, uint8_t * address, const uint8_t * p_physical, size_t size)
 //{
 //    uint32_t offset;
 //
 //    if (p_physical >= p_flash->OpAddress && p_physical <= p_flash->OpAddress + p_flash->OpSizeAligned)
 //    {
 //        offset = p_physical - p_flash->OpAddress;
 //        memcpy(address, &p_flash->Buffer[offset], size);
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


