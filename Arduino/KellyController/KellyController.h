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
	#include "Motor/MotorCmdr/MotorCmdr.h"
};

#include "HardwareSerial.h"
#include <Arduino.h>
#include <cstdint>

typedef enum
{
	KELLY_CONTROLLER_RX_WAITING,
	KELLY_CONTROLLER_RX_SUCCESS,
	KELLY_CONTROLLER_RX_ERROR,
	// KELLY_CONTROLLER_RX_ERROR_SYNC,
	KELLY_CONTROLLER_RX_TIMEOUT,
}
KellyController_Status_T;

class KellyController
{
private:
// public:
	static uint32_t millisTimer;
	// Stream * p_serialStream;
	HardwareSerial * p_serial;
	MotorCmdr_T motorCmdr = MOTOR_CMDR_DEFINE(&motorCmdr, 0U, 0U, &millisTimer);
	uint8_t errorLength;

public:
	KellyController(HardwareSerial & serial);
	virtual ~KellyController(void);
	void begin(void);
	void end(void);

	KellyController_Status_T poll(void);
	void ping(void);
	void writeStopAll(void);
	void writeInitUnits(void);
	void writeThrottle(uint16_t throttle);
	void writeBrake(uint16_t brake);
	void writeSaveNvm(void);
	void writeRelease(void);
	void writeDirectionForward(void);
	void writeDirectionReverse(void);
	void writeDirectionNeutral(void);
	void readSpeed(void);
	void readIFoc(void);

	int32_t getReadSpeed(void) { return MotorCmdr_GetReadSpeed(&motorCmdr); }
	int16_t getReadIa(void) { return MotorCmdr_GetReadIa(&motorCmdr); }
	int16_t getReadIb(void) { return MotorCmdr_GetReadIb(&motorCmdr); }
	int16_t getReadIc(void) { return MotorCmdr_GetReadIc(&motorCmdr); }
	int16_t getReadIalpha(void) { return MotorCmdr_GetReadIalpha(&motorCmdr); }
	int16_t getReadIbeta(void) { return MotorCmdr_GetReadIbeta(&motorCmdr); }
	int16_t getReadId(void) { return MotorCmdr_GetReadId(&motorCmdr); }
	int16_t getReadIq(void) { return MotorCmdr_GetReadIq(&motorCmdr); }

	// Debuging use
	uint8_t getTxLength(void) { return _MotorCmdr_GetReqLength(&motorCmdr); }
	uint8_t * getPtrTxPacket(void) { return motorCmdr.TxPacket; }
	uint8_t getRxLength(void) { return _MotorCmdr_GetRespLength(&motorCmdr); }
	uint8_t * getPtrRxPacket(void) { return motorCmdr.RxPacket; }

	uint8_t getErrorRxLength(void) { return errorLength; }
};

#endif