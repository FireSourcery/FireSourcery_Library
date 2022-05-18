
/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
	@file 	MotProtocol.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorCmdr.h"
#include "Utility/Protocol/Concrete/MotProtocol/MotProtocol.h"

/*
	Composition, app uses its own structure
*/
// static void BuildReq_Control(MotProtocol_ReqPacket_Control_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
// {
// 	/* App uses its own structure */
//     switch(p_app->ReqActive)
//     {
//         case MOTOR_CMDR_WRITE_THROTTLE:
// 			*p_txLength = MotProtocol_ReqPacket_Control_BuildThrottle(p_reqPacket, p_app->Throttle);
// 			*p_respLength = MotProtocol_GetControlRespLength(MOTPROTOCOL_CONTROL_THROTTLE);
//             break;
//     }
// }

// static void ParseResp(MotorCmdr_T * p_app, const uint8_t * p_rxPacket)
// {

// }


/* App implements interface - config protocol with p_app->Interface */
/*  */
// Mot_Protocol_Cmdr_BuildReq_Control(p_reqPacket, p_txLength, p_respLength, p_app->Interface);