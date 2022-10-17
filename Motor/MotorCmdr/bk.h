// StopAll_BuildReq,
// Ping_BuildReq,
// InitUnits_BuildReq,
// SaveNvm_BuildReq,
// WriteVar_BuildReq,
// ReadVar_BuildReq,
// Control_BuildReq,
// Monitor_BuildReq,

static void Ping_BuildReq(MotPacket_PingReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_PingReq_Build(p_txPacket);
	*p_respLength = MotPacket_PingReq_GetRespLength();
}
static void StopAll_BuildReq(MotPacket_StopReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_StopReq_Build(p_txPacket);
	*p_respLength = MotPacket_StopReq_GetRespLength();
}
static void SaveNvm_BuildReq(MotPacket_SaveNvmReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_SaveNvmReq_Build(p_txPacket);
	*p_respLength = MotPacket_SaveNvmReq_GetRespLength();
}
static void InitUnits_BuildReq(MotPacket_InitUnitsReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	(void)p_app;
	*p_txLength = MotPacket_InitUnitsReq_Build(p_txPacket);
	*p_respLength = MotPacket_InitUnitsReq_GetRespLength();
}

static void Control_BuildReq(MotPacket_ControlReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	switch(p_app->ControlIdActive)
	{
		case MOT_PACKET_CONTROL_THROTTLE:				*p_txLength = MotPacket_ControlReq_Throttle_Build(p_txPacket, p_app->MotorCmdValue); 	break;
		case MOT_PACKET_CONTROL_BRAKE: 				*p_txLength = MotPacket_ControlReq_Brake_Build(p_txPacket, p_app->MotorCmdValue); 		break;
		case MOT_PACKET_CONTROL_RELEASE: 				*p_txLength = MotPacket_ControlReq_Release_Build(p_txPacket); 							break;
		case MOT_PACKET_CONTROL_DIRECTION_FORWARD: 	*p_txLength = MotPacket_ControlReq_DirectionForward_Build(p_txPacket); 					break;
		case MOT_PACKET_CONTROL_DIRECTION_REVERSE: 	*p_txLength = MotPacket_ControlReq_DirectionReverse_Build(p_txPacket); 					break;
		case MOT_PACKET_CONTROL_DIRECTION_NEUTRAL: 	*p_txLength = MotPacket_ControlReq_DirectionNeutral_Build(p_txPacket); 					break;
		default: break;
	}
	*p_respLength = MotPacket_ControlReq_GetRespLength(p_app->ControlIdActive);
}
static void Monitor_BuildReq(MotPacket_MonitorReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_MonitorReq_Build(p_txPacket, p_app->MonitorIdActive);
	*p_respLength = MotPacket_MonitorReq_GetRespLength(p_app->MonitorIdActive);
}
static void WriteVar_BuildReq(MotPacket_WriteVarReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_WriteVarReq_Build(p_txPacket, p_app->MotorReadWriteVarId, p_app->MotorReadWriteVarValue);
	*p_respLength = MotPacket_WriteVarReq_GetRespLength();
}
static void ReadVar_BuildReq(MotPacket_ReadVarReq_T * p_txPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
{
	*p_txLength = MotPacket_ReadVarReq_Build(p_txPacket, p_app->MotorReadWriteVarId);
	*p_respLength = MotPacket_ReadVarReq_GetRespLength();
}