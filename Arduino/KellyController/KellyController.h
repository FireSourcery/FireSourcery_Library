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

extern "C"
{
	#include "MotorCmdr/MotorCmdr.h"
};

#include "HardwareSerial.h"
#include <Arduino.h>
#include <cstdint>

#define PROTOCOL_PACKET_BUFFER_SIZE 64U

class KellyController
{
private:
	static uint32_t millisTimer;
	// Stream * p_serialStream;
	HardwareSerial * p_serial;
	uint8_t txPacket[PROTOCOL_PACKET_BUFFER_SIZE];
	uint8_t rxPacket[PROTOCOL_PACKET_BUFFER_SIZE];
	MotorCmdr_T motorCmdr = MOTOR_CMDR_DEFINE(&motorCmdr, rxPacket, txPacket, 0U, 0U, &millisTimer);

public:
	KellyController(HardwareSerial & serial);
	virtual ~KellyController(void);
	void begin(void);
	void end(void);
	// void update(void);

	bool pollRx(void);
	void writeThrottle(uint16_t throttle);

	uint8_t * getTxPacket(void) { return txPacket; }

};

#endif