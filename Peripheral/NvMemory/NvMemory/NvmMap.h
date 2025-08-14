#include "NvMemory.h"

/******************************************************************************/
/*
*/
/******************************************************************************/
typedef const struct NvMemory_MapEntry
{
    NvMemory_Partition_T NVM;
    void * const RAM_ADDRESS; /* RAM address */
    // const uint32_t checksum_seed;    /* For validation */
}
NvMemory_MapEntry_T;

typedef const struct NvMemory_Map
{
    const NvMemory_MapEntry_T * P_ENTRIES; /* Array of entries */
    size_t ENTRY_COUNT; /* Number of entries */
}
NvMemory_Map_T;


// static NvMemory_Status_T SaveEntry_Blocking(const NvMemory_T * p_motNvm, const NvMemory_MapEntry_T * p_entry)
// {
//     // assert(p_entry != NULL);
//     // assert(p_entry->NVM_ADDRESS != NULL);
//     // assert(p_entry->RAM_ADDRESS != NULL);
//     // assert(p_entry->SIZE > 0U);

//     return MotNvm_Write_Blocking(p_motNvm, p_entry->NVM_ADDRESS, p_entry->RAM_ADDRESS, p_entry->SIZE);
// }

// NvMemory_Status_T MotNvm_SaveConfigAll_Blocking(const NvMemory_T * p_motNvm)
// {
//     NvMemory_Status_T status;

//     /* Flash Erase Full block */
//     status = Flash_Erase_Blocking(p_motNvm->P_FLASH, p_motNvm->MAIN_CONFIG_ADDRESS, p_motNvm->MAIN_CONFIG_SIZE);
//     if (status != NV_MEMORY_STATUS_SUCCESS) { return status; }

//     for (size_t i = 0U; i < p_motNvm->PARTITION_COUNT; i++)
//     {
//         status = SaveEntry_Blocking(p_motNvm, &p_motNvm->P_PARTITIONS[i]);
//         if (status != NV_MEMORY_STATUS_SUCCESS) { break; }
//     }

//     return status;
// }