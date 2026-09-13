#include "CAN.h"

#include <stdint.h>
#include <stdbool.h>


void CAN_Init(CAN_T * p_can)
{
    HAL_CAN_Init(p_can->P_HAL);
    HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
    // p_can->P_STATE->ServiceHandler = CAN_ProcServiceDisabled;
    p_can->P_STATE->p_Service = p_can->P_SERVICE; /* default active service; runtime-swappable via CAN_Enable/CAN_SetService */
}

void CAN_InitBaudRate(CAN_T * p_can, uint32_t bitRate)
{
    HAL_CAN_InitBaudRate(p_can->P_HAL, bitRate);
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


