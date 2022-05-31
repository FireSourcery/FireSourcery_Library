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
	@file 	KellyController.cpp
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "KellyController.h"

uint32_t KellyController::millisTimer;

/******************************************************************************/
/*!
	@brief Class Constructor
*/
/******************************************************************************/
KellyController::KellyController(HardwareSerial & serial)
{
	millisTimer = millis();
	// p_serialStream = &serial;
	p_serial = &serial;
	MotorCmdr_Init(&motorCmdr);
}


/******************************************************************************/
/*!
	@brief Class Destructor
*/
/******************************************************************************/
KellyController::~KellyController()
{
	end();
}

/******************************************************************************/
/*!
	@brief Init Arduino Serial
*/
/******************************************************************************/
void KellyController::begin()
{
	if(p_serial != 0U) { p_serial->begin(motorCmdr.Protocol.p_Specs->BAUD_RATE_DEFAULT); }
	//delay?
	MotorCmdr_InitUnits(&motorCmdr);
}

/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/
void KellyController::end(void)
{
	if(p_serial != 0U) { p_serial->end(); }
}


/******************************************************************************/
/*!
	@brief Call regularly.
		Sends ping when no Req sent
	@return true when new data is available
*/
/******************************************************************************/
KellyController_Status_T KellyController::poll(void)
{
	millisTimer = millis();
	KellyController_Status_T status;

	if(_MotorCmdr_PollTimeout(&motorCmdr) == false)
	{
		if(p_serial->available() >= _MotorCmdr_GetRespLength(&motorCmdr))
		{
			if(p_serial->peek() != MOT_PACKET_START_BYTE)  /* Serial Rx out of sync */
			{
				for(uint8_t iChar = 0U; iChar < p_serial->available(); iChar++)
				{
					if(p_serial->peek() != MOT_PACKET_START_BYTE) 	{ _MotorCmdr_GetPtrRxPacket(&motorCmdr)[iChar] = p_serial->read(); }
					else 											{ errorLength = iChar; break; }
				}
				status = KELLY_CONTROLLER_RX_ERROR;
			}
			else
			{
				p_serial->readBytes(_MotorCmdr_GetPtrRxPacket(&motorCmdr), _MotorCmdr_GetRespLength(&motorCmdr));
				status = _MotorCmdr_ParseResp(&motorCmdr) ? KELLY_CONTROLLER_RX_SUCCESS : KELLY_CONTROLLER_RX_ERROR; /* new data is ready, returns false if crc error */
			}
		}
		else
		{
			status = KELLY_CONTROLLER_RX_WAITING;
		}
	}
	else
	{
		status = KELLY_CONTROLLER_RX_TIMEOUT;
	}

	// _MotorCmdr_ProcTxIdle(&motorCmdr);

	return status;
}

/******************************************************************************/
/*!
	Tx Reqs
*/
/******************************************************************************/
void KellyController::ping(void)
{
	_MotorCmdr_Ping(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeStopAll(void)
{
	_MotorCmdr_StopMotors(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeInitUnits(void)
{
	MotorCmdr_InitUnits(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeThrottle(uint16_t throttle)
{
	_MotorCmdr_WriteThrottle(&motorCmdr, throttle);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeBrake(uint16_t brake)
{
	_MotorCmdr_WriteBrake(&motorCmdr, brake);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeSaveNvm(void)
{
	_MotorCmdr_SaveNvm(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeRelease(void)
{
	_MotorCmdr_WriteRelease(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeDirectionForward(void)
{
	_MotorCmdr_WriteDirectionForward(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeDirectionReverse(void)
{
	_MotorCmdr_WriteDirectionReverse(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::writeDirectionNeutral(void)
{
	_MotorCmdr_WriteDirectionNeutral(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::readSpeed(void)
{
	_MotorCmdr_StartReadSpeed(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::readIFoc(void)
{
	_MotorCmdr_StartReadIFoc(&motorCmdr);
	p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
}



// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
// size_t KellyController::write(uint8_t c)
// {
// 	return p_serial->write(c);
// }

// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
// int KellyController::available(void)
// {
// 	return p_serial->available();
// }

// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
// int KellyController::read(void)
// {
// 	int c = p_serial->read();
// 	return c;
// }

// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
// int KellyController::peek(void)
// {
// 	return p_serial->peek();
// }

// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
// void KellyController::flush(void)
// {
// 	p_serial->flush();
// }