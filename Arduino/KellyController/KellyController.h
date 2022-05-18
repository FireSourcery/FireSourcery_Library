/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / Kelly Controls Inc

	This file is part of Arduino_KellyController (https://github.com/FireSourcery/Arduino_KellyController).

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
	@file 	KellyController.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef KELLY_CONTROLLER_H
#define KELLY_CONTROLLER_H

#include "MotorCmdr/MotorCmdr.h"

// #include <Arduino.h>
#include <stdint.h>

#define PROTOCOL_PACKET_BUFFER_SIZE 64U

class KellyController
{
	private:
	// Stream * p_serialStream;
	HardwareSerial * p_serial;

	uint8_t protocolId;
	uint8_t txPacket[PROTOCOL_PACKET_BUFFER_SIZE];
	uint8_t rxPacket[PROTOCOL_PACKET_BUFFER_SIZE];
	int32_t Speed_Frac16;

	MotorCmdr_T motorCmdr;

	MotorCmdr_T motorCmdr =
	{
		.Protocol.CONFIG = PROTOCOL_CONFIG
		(
			rxPacket, txPacket, PROTOCOL_PACKET_BUFFER_SIZE,
			&motorCmdr.Interface, 0U,
			&MOT_PROTOCOL_CMDR_SPECS, 1U,
			0U, 0U,
			&arduinoMillis, 0U
		),
	};



	public:
	KellyController(HardwareSerial & port);
	virtual ~KellyController();
	void begin(uint32_t baudRate);
	void end(void);

	void update(void);
	void writeThrottle(uint16_t throttle);
};

#endif