//Set all frame information including data
//void CanBus_SetTxFrame(CanBus_T * p_can,  CAN_FrameFormat_t format, CAN_FrameType_t type, CAN_ID_t id, uint8_t length, uint8_t * p_data)
//{
//    /* Prepare Tx Frame for sending. */
//	p_can->TxFrame.ID 		= id;
//	p_can->TxFrame.Format 	= format;
//	p_can->TxFrame.Type 		= type;
//	p_can->TxFrame.DLR 		= length;
//
//	for (uint8_t i = 0; i < length; i++)
//	{
//		p_can->TxFrame.DSR[i]  = p_data[i];
//	}
//}
//
////Set all frame information expect data
//void CanBus_SetTxFrameControl(CanBus_T * p_can, CAN_ID_t id, CAN_FrameFormat_t format, CAN_FrameType_t type )
//{
//	p_can->TxFrame.ID 		= id;
//	p_can->TxFrame.Format 	= format;
//	p_can->TxFrame.Type 		= type;
//}
//
//
//void CanBus_SetTxFrameDataByte(CanBus_T * p_can, uint8_t dataByte, uint8_t dataIndex)
//{
//	p_can->TxFrame.DSR[dataIndex]  = dataByte;
//}
//
//void CanBus_SetTxFrameData(CanBus_T * p_can, uint8_t * p_data, uint8_t length)
//{
//	p_can->TxFrame.DLR = length;
//
//	for (uint8_t i = 0; i < length; i++)
//	{
//		p_data[i] = p_can->TxFrame.DSR[i];
//	}
//}
//
//
//void CanBus_GetRxFrameData(CanBus_T * p_can, uint32_t * p_id, uint8_t * p_data)
//{
//	p_id = p_can->RxFrame.ID.ID;
//
//	for (uint8_t i = 0; i < p_can->RxFrame.DLR; i++)
//	{
//		p_data[i] = p_can->RxFrame.DSR[i];
//	}
//}
