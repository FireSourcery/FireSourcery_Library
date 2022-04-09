#ifndef HAL_CAN_BUS_H
#define HAL_CAN_BUS_H

#include "../../../../../[Workspace]/Kelly_Library/CanBus/CanType.h"

#include "S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

//static uint32_t FreqClockSource;

/*temp*/
#define CAN_RX_PIN            0x01U /*!< Rx pin mask */
#define CAN_TX_PIN            0x02U /*!< Tx pin mask */

#define CAN_ON_FULL_RXBUFFER  0x01U /*!< OnFullRxBuffer event mask */
#define CAN_ON_FREE_TXBUFFER  0x02U /*!< OnFreeTxBuffer event mask */
#define CAN_ON_BUSOFF         0x04U /*!< OnBusOff event mask */
#define CAN_ON_TXWARNING      0x08U /*!< OnTransmitterWarning event mask */
#define CAN_ON_RXWARNING      0x10U /*!< OnReceiverWarning event mask */
#define CAN_ON_ERROR          0x20U /*!< OnError event mask */
#define CAN_ON_WAKEUP         0x40U /*!< OnWakeUp event mask */

#define CAN_BIT0_ERROR        0x01UL /*!< Bit0 error detect error mask */
#define CAN_BIT1_ERROR        0x02UL /*!< Bit1 error detect error mask */
#define CAN_ACK_ERROR         0x04UL /*!< Acknowledge error detect error mask */
#define CAN_CRC_ERROR         0x08UL /*!< Cyclic redundancy check error detect error mask */
#define CAN_FORM_ERROR        0x10UL /*!< Message form error detect error mask */
#define CAN_STUFFING_ERROR    0x20UL /*!< Bit stuff error detect error mask */
#define CAN_RX_OVERRUN_ERROR  0x40UL /*!< Receiver overrun detect error mask */

#define CAN_MESSAGE_ID_EXT    0x80000000UL /*!< Value specifying extended Mask, ID */

#define CAN_RUN_MODE          0x00UL /*!< CAN device in run mode mask */
#define CAN_FREEZE_MODE       0x01UL /*!< CAN device in freeze (initialization) mode mask */
#define CAN_SLEEP_MODE        0x02UL /*!< CAN device in sleep mode mask */

typedef uint8_t CAN_MBIndex_t;      /*!< CAN message buffer index */
typedef uint8_t CAN_TElementIndex; /*!< Type specifying the filter table mask element index */
typedef uint32_t CAN_TAccMask;     /*!< Type specifying the acceptance mask variable */
typedef uint32_t CAN_TAccCode;     /*!< Type specifying the acceptance code variable */
typedef uint32_t CAN_TMessageID;   /*!< Type specifying the ID mask variable */
typedef uint8_t CAN_TErrorCounter; /*!< Type specifying the error counter variable */
typedef uint32_t CAN_TErrorMask;   /*!< Type specifying the error mask variable */
typedef uint8_t CAN_TBufferMask;   /*!< Type specifying the message buffer mask variable */
typedef uint8_t CAN_TIDHitFilterIndex; /*!< Type specifying the Rx FIFO hit filter index variable */
typedef uint8_t CAN_TRunModeMask;  /*!< Type specifying the run mode variable */



typedef CAN_Type HAL_CanBus_T;

static inline bool HAL_CanBus_GetTxBufferEmptyFlag(HAL_CanBus_T * p_canBus)
{

}

static inline bool HAL_CanBus_GetRxBufferFullFlag(HAL_CanBus_T * p_canBus)
{

}

static inline void HAL_CanBus_EnableTxBufferEmptyInterrupt(HAL_CanBus_T * p_canBus)
{

}

static inline void HAL_CanBus_EnableRxBufferFullInterrupt(HAL_CanBus_T * p_canBus)
{

}

static inline void HAL_CanBus_DisableTxBufferEmptyInterrupt(HAL_CanBus_T * p_canBus)
{

}

static inline void HAL_CanBus_DisableRxBufferFullInterrupt(HAL_CanBus_T * p_canBus)
{

}

static inline void HAL_CanBus_ClearRxBufferFullFlag(HAL_CanBus_T * p_canBus)
{
}


static inline void HAL_CanBus_SetBaudRate(HAL_CanBus_T * p_canBus, uint32_t baudRate)
{
}


static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_canBus, CanBus_Frame_T * p_txFrame)
{

}

static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_canBus, CanBus_Frame_T * p_rxFrame)
{

}

static inline void HAL_CanBus_Init(uint32_t busFreq)
{

}

#endif
