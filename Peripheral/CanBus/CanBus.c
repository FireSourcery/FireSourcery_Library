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

void CanBus_SendData(CanBus_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length)
{
    HAL_CAN_WriteTxId(p_can->P_HAL, id);
    HAL_CAN_WriteTxData(p_can->P_HAL, p_txData, length); /* includes Transmit */
}

// void CanBus_SendRequestMapResponse(CanBus_T * p_can, uint32_t tx_id, uint8_t * data, uint32_t rx_id, uint32_t timeout_ms)
// {

// }

/*
    State tracking with Tx interrupt for Remote requests
*/
void CanBus_Send(CanBus_T * p_can, can_id_t id, const uint8_t * p_txData, size_t length)
{
    if (id.Rtr == true) { p_can->P_STATE->Channel[0].State = CAN_BUS_BUFFER_RX_WAIT_REMOTE; }
    HAL_CAN_WriteTxId(p_can->P_HAL, id);
    HAL_CAN_WriteTxData(p_can->P_HAL, p_txData, length); /* remote still calls to launch */
    HAL_CAN_EnableTxEmptyInterrupt(p_can->P_HAL);
}

/* polling */
size_t CanBus_ReceiveData(CanBus_T * p_can, can_id_t * p_rxId, uint8_t * p_rxData, size_t length)
{
    if (HAL_CAN_ReadRxFullFlag(p_can->P_HAL))
    {
        *p_rxId = HAL_CAN_ReadRxId(p_can->P_HAL);
        return HAL_CAN_ReadRxData(p_can->P_HAL, p_rxData);
    }
    return 0;
}

void CanBus_StartReceive(CanBus_T * p_can, can_id_t rxId)
{
    p_can->P_STATE->Channel[0].State = CAN_BUS_BUFFER_RX_WAIT_DATA;
    HAL_CAN_EnableRxFullInterrupt(p_can->P_HAL);
}

size_t CanBus_ReceiveRequest(CanBus_T * p_can, uint32_t * p_rxId, uint8_t * p_rxData, size_t length)
{
    CanBus_Buffer_T * p_buf = &p_can->P_STATE->Channel[0];

    if (p_buf->State == CAN_BUS_BUFFER_RX_WAIT_SERVICE)
    {
        *p_rxId = p_buf->Frame.CanId.Id;
        memcpy(p_rxData, &p_buf->Frame.Data[0], p_buf->Frame.DataLength);
        p_buf->State = CAN_BUS_BUFFER_IDLE;
    }
    return 0;
}


