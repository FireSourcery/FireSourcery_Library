#include "KellyController.h"


#include "MotorCmdr/MotorCmdr.h"

/******************************************************************************/
/*!
	@brief Class Constructor
*/
/******************************************************************************/
KellyController::KellyController(HardwareSerial & serial)
{
	// p_serialStream = &serial;
	p_serial = &serial;
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
void KellyController::Proc( )
{

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
	@brief
*/
/******************************************************************************/
bool KellyController::pollRx(void)
{
	if(p_serial->available() >= _MotorCmdr_GetRespLength(&motorCmdr))
	{
		p_serial->readBytes(rxPacket, _MotorCmdr_GetRespLength(&motorCmdr));

		_MotorCmdr_ParseResp(&motorCmdr); /* new data is ready */
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
	p_serial->write(txPacket, _MotorCmdr_GetReqLength(&motorCmdr));
}

void KellyController::getReadSpeed(void) { return MotorCmdr_GetReadSpeed(&motorCmdr); }


void KellyController::startReadMonitor( )
{
}


void KellyController::getReadMonitor( )
{
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