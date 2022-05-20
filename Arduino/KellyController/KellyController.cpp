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
	//   p_serial->setTimeout( );
}

/******************************************************************************/
/*!
	@brief Call regularly
*/
/******************************************************************************/
// void KellyController::Proc( )
// {

// }

/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/
void KellyController::end(void)
{
	if(p_serial != 0U) { p_serial->end(); }
}


// /******************************************************************************/
// /*!
// 	@brief
// */
// /******************************************************************************/
bool KellyController::pollRx(void)
{
	this->millisTimer = millis();
	if(p_serial->available() >= _MotorCmdr_GetRespLength(&motorCmdr))
	{
		p_serial->readBytes(rxPacket, _MotorCmdr_GetRespLength(&motorCmdr));

		_MotorCmdr_CaptureResp(&motorCmdr); /* new data is ready, returns false if crc error */
		return true;
	}
	else
	{
		return false;
	}
}

void KellyController::writeThrottle(uint16_t throttle)
{
	_MotorCmdr_WriteThrottle(&motorCmdr, throttle);
	txPacket[0] = 'S';
	p_serial->write(txPacket, _MotorCmdr_GetReqLength(&motorCmdr));
}




// void KellyController::getReadSpeed(void) { return MotorCmdr_GetReadSpeed(&motorCmdr); }


// void KellyController::startReadMonitor( )
// {
// }


// void KellyController::getReadMonitor( )
// {
// }


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