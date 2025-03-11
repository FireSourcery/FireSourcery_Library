#include "../Xcvr/Xcvr.h"
#include "Serial.h"


const Xcvr_Interface_T SERIAL_XCVR =
{
    .TX_BYTE = Serial_SendByte,
    .RX_BYTE = Serial_RecvByte,
    .TX_MAX = Serial_SendMax,
    .RX_MAX = Serial_RecvMax,
    .TX_N = Serial_SendN,
    .RX_N = Serial_RecvN,
    .GET_TX_EMPTY_COUNT = Serial_GetTxEmptyCount,
    .GET_RX_FULL_COUNT = Serial_GetRxFullCount,
    .CONFIG_BAUD_RATE = Serial_ConfigBaudRate,
};