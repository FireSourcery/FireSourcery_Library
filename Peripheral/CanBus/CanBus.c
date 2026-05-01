#include "CanBus.h"

#include <stdint.h>
#include <stdbool.h>


void CanBus_Init(CanBus_T * p_can)
{
    HAL_CAN_Init(p_can->P_HAL);
    HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
}

void CanBus_InitBaudRate(CanBus_T * p_can, uint32_t bitRate)
{
    HAL_CAN_InitBaudRate(p_can->P_HAL, bitRate);
}

/* Tx a Data frame */
void CanBus_TxData(CanBus_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length)
{
    HAL_CAN_WriteTxId(p_can->P_HAL, id);
    HAL_CAN_WriteTxData(p_can->P_HAL, p_txData, length); /* includes Transmit */
}

// void CanBus_TxRequestMapResponse(CanBus_T * p_can, uint32_t tx_id, uint8_t * data, uint32_t rx_id, uint32_t timeout_ms)
// {
// }

/*
    State tracking with Tx interrupt for Remote requests
*/
/* TxRequest */
/* enforce frame interface for remote */
void CanBus_Tx(CanBus_T * p_can, CAN_Frame_T * p_frame)
{
    if (p_frame->CanId.Rtr == true) { p_can->P_STATE->Channel[0].State = CAN_BUS_BUFFER_RX_WAIT_REMOTE; }
    HAL_CAN_WriteTxMessage(p_can->P_HAL, p_frame);
    HAL_CAN_EnableTxEmptyInterrupt(p_can->P_HAL);
}

// void CanBus_ExpectRx(CanBus_T * p_can, can_id_t rxId)
// {
//     p_can->P_STATE->Channel[0].State = CAN_BUS_BUFFER_RX_WAIT_DATA;
//     HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
// }

/* ISR without callback version */
// size_t CanBus_PollRequest(CanBus_T * p_can, uint32_t * p_rxId, uint8_t * p_rxData )
// {
//     CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[0];

//     if (p_buf->State == CAN_BUS_BUFFER_RX_WAIT_SERVICE)
//     {
//         *p_rxId = p_buf->Frame.CanId.Id;
//         memcpy(p_rxData, &p_buf->Frame.Data[0], p_buf->Frame.DataLength);
//         p_buf->State = CAN_BUS_BUFFER_IDLE;
//     }
//     return 0;
// }


