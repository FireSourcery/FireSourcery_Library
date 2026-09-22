#include "CAN.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Program every hardware bank from Config. Hardware accepts a frame when ANY bank matches,
    so a bank left open would defeat the others — banks beyond RxFilterCount repeat the last filter.
*/
static inline uint8_t RxFilterCountOf(uint32_t count) { return (uint8_t)((count < CAN_RX_FILTER_COUNT) ? count : CAN_RX_FILTER_COUNT); }

static void ApplyRxFilters(CAN_T * p_can)
{
    const CAN_Config_T * p_config = &p_can->P_STATE->Config;
    uint8_t count = RxFilterCountOf(p_config->RxFilterCount);

    if (count == 0U) { HAL_CAN_SetRxFilterAcceptAll(p_can->P_HAL); return; }

    for (uint8_t bank = 0U; bank < CAN_RX_FILTER_COUNT; bank++)
    {
        const CAN_RxFilter_T * p_filter = &p_config->RxFilters[(bank < count) ? bank : (count - 1U)];
        if (p_filter->Id.Eff) { HAL_CAN_SetRxFilterExtended(p_can->P_HAL, bank, p_filter->Id.Id, p_filter->Mask); }
        else                  { HAL_CAN_SetRxFilterStandard(p_can->P_HAL, bank, p_filter->Id.Id, p_filter->Mask); }
    }
}

void CAN_Init(CAN_T * p_can)
{
    /* No config source keeps the prior behavior: service enabled, accept all */
    p_can->P_STATE->Config = (p_can->P_NVM_CONFIG != NULL) ? *p_can->P_NVM_CONFIG : (CAN_Config_T) { .IsEnabled = true };

    HAL_CAN_Init(p_can->P_HAL);
    ApplyRxFilters(p_can);
    HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL); /* after every init-mode window — MSCAN holds CANRIER in reset there */

    // p_can->P_STATE->ServiceHandler = CAN_ProcServiceDisabled;
    p_can->P_STATE->p_Service = (p_can->P_STATE->Config.IsEnabled) ? p_can->P_SERVICE : NULL; /* runtime-swappable via CAN_Enable/CAN_SetService */
}

/* Runtime reconfiguration. Aborts any pending Tx on platforms that need init mode to rewrite filters. */
void CAN_SetRxFilters(CAN_T * p_can, const CAN_RxFilter_T * p_filters, uint8_t count)
{
    CAN_Config_T * p_config = &p_can->P_STATE->Config;
    p_config->RxFilterCount = RxFilterCountOf(count);
    for (uint8_t i = 0U; i < p_config->RxFilterCount; i++) { p_config->RxFilters[i] = p_filters[i]; }
    ApplyRxFilters(p_can);
}

void CAN_InitBaudRate(CAN_T * p_can, uint32_t bitRate)
{
    HAL_CAN_InitBaudRate(p_can->P_HAL, bitRate);
}


/******************************************************************************/
/*!
    Var Id interface
*/
/******************************************************************************/
static_assert(CAN_RX_FILTER_COUNT >= 2U, "CAN_ConfigId enumerates two Rx filter banks");

int _CAN_ConfigId_Get(const CAN_Config_T * p_config, CAN_ConfigId_T id)
{
    switch (id)
    {
        case CAN_CONFIG_IS_ENABLED:                 return p_config->IsEnabled;
        case CAN_CONFIG_RX_FILTER_COUNT:            return p_config->RxFilterCount;
        case CAN_CONFIG_RX_FILTER0_ID:              return (int)p_config->RxFilters[0U].Id.Id;
        case CAN_CONFIG_RX_FILTER0_MASK:            return (int)p_config->RxFilters[0U].Mask;
        case CAN_CONFIG_RX_FILTER0_IS_EXTENDED:     return p_config->RxFilters[0U].Id.Eff;
        case CAN_CONFIG_RX_FILTER1_ID:              return (int)p_config->RxFilters[1U].Id.Id;
        case CAN_CONFIG_RX_FILTER1_MASK:            return (int)p_config->RxFilters[1U].Mask;
        case CAN_CONFIG_RX_FILTER1_IS_EXTENDED:     return p_config->RxFilters[1U].Id.Eff;
    }
    return 0;
}

int CAN_ConfigId_Get(CAN_T * p_can, CAN_ConfigId_T id)
{
    if (p_can == NULL) { return 0; }
    return _CAN_ConfigId_Get(&p_can->P_STATE->Config, id);
}

void _CAN_ConfigId_Set(CAN_Config_T * p_config, CAN_ConfigId_T id, int value)
{
    switch (id)
    {
        case CAN_CONFIG_IS_ENABLED:                 p_config->IsEnabled = (value != 0);                         break;
        case CAN_CONFIG_RX_FILTER_COUNT:            p_config->RxFilterCount = RxFilterCountOf((uint32_t)value); break;
        case CAN_CONFIG_RX_FILTER0_ID:              p_config->RxFilters[0U].Id.Id = (uint32_t)value;            break;
        case CAN_CONFIG_RX_FILTER0_MASK:            p_config->RxFilters[0U].Mask = (uint32_t)value;             break;
        case CAN_CONFIG_RX_FILTER0_IS_EXTENDED:     p_config->RxFilters[0U].Id.Eff = (value != 0);              break;
        case CAN_CONFIG_RX_FILTER1_ID:              p_config->RxFilters[1U].Id.Id = (uint32_t)value;            break;
        case CAN_CONFIG_RX_FILTER1_MASK:            p_config->RxFilters[1U].Mask = (uint32_t)value;             break;
        case CAN_CONFIG_RX_FILTER1_IS_EXTENDED:     p_config->RxFilters[1U].Id.Eff = (value != 0);              break;
    }
}

void CAN_ConfigId_Set(CAN_T * p_can, CAN_ConfigId_T id, int value)
{
    if (p_can == NULL) { return; }
    _CAN_ConfigId_Set(&p_can->P_STATE->Config, id, value);
}


/*
    State tracking with Tx interrupt for Remote requests
*/
/* enforce frame interface for remote */
// void CAN_Tx(CAN_T * p_can, CAN_Frame_T * p_frame)
// {
//     if (p_frame->CanId.Rtr == true) { p_can->P_STATE->Channel[0].State = CAN_BUFFER_RX_WAIT_REMOTE; }
//     HAL_CAN_WriteTxMessage(p_can->P_HAL, p_frame);
//     HAL_CAN_EnableTxEmptyInterrupt(p_can->P_HAL);
// }

// void CAN_ExpectRx(CAN_T * p_can, can_id_t rxId)
// {
//     p_can->P_STATE->Channel[0].State = CAN_BUFFER_RX_WAIT_DATA;
//     HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
// }

/* ISR without callback version */
// size_t CAN_PollRequest(CAN_T * p_can, uint32_t * p_rxId, uint8_t * p_rxData )
// {
//     CAN_Buffer_T * p_buf = &p_can->P_STATE->Channel[0];

//     if (p_buf->State == CAN_BUFFER_RX_WAIT_SERVICE)
//     {
//         *p_rxId = p_buf->Frame.CanId.Id;
//         memcpy(p_rxData, &p_buf->Frame.Data[0], p_buf->Frame.DataLength);
//         p_buf->State = CAN_BUFFER_IDLE;
//     }
//     return 0;
// }


