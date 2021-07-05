//#include "Peripheral/Serial/Serial_IO.h"
//#include "Peripheral/Serial/Serial.h"
//
//uint8_t SerialTxBuffer1[100];
//uint8_t SerialRxBuffer1[100];
//
//Serial_Init_T SERIAL_INIT1 =
//{
//	.P_HAL_SERIAL 		= LPUART1,
//	.P_TX_BUFFER		= SerialTxBuffer1,
//	.P_RX_BUFFER		= SerialRxBuffer1,
//	.TX_BUFFER_SIZE		= 100U,
//	.RX_BUFFER_SIZE		= 100U,
//};
//
//void Serial1_Init(Serial_T * p_serial)
//{
//	/* LPUART module initialization */
////	status = LPUART_DRV_Init(0U, &lpUartState0, &lpuart_0_InitConfig0);
////	INT_SYS_EnableIRQ(LPUART0_RxTx_IRQn);
//
////	status = LPUART_DRV_Init(0U, &lpUartState1, &lpuart_1_InitConfig0);
////	INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);
//
//	Serial_Init
//	(
//		p_serial,
//		&SERIAL_INIT1
//	);
//}
