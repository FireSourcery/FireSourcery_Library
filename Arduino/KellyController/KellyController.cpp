/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   KellyController.cpp
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "KellyController.h"

uint32_t KellyController::millisTimer;

/* Wrappers */
static size_t HardwareSerial_ReadBytes(HardwareSerial * p_context, uint8_t * p_destBuffer, size_t max) { return p_context->readBytes(p_destBuffer, max); }
static bool HardwareSerial_Write(HardwareSerial * p_context, const uint8_t * p_src, size_t length) { return (p_context->write(p_src, length) == length); } // no error handling on write success return

const Xcvr_Interface_T XCVR_INTERFACE_ARDUINO_SERIAL =
{
    .RX_MAX = (Xcvr_Interface_RxMax_T)HardwareSerial_ReadBytes,
    .TX_N = (Xcvr_Interface_TxN_T)HardwareSerial_Write,
};

const Xcvr_Entry_T KELLY_XCVR_TABLE[KELLY_XCVR_COUNT] =
{
    [0] = XCVR_ENTRY_INIT_INTERFACE(&Serial1, &XCVR_INTERFACE_ARDUINO_SERIAL),
};

/******************************************************************************/
/*!
    @brief Class Constructor
*/
/******************************************************************************/
KellyController::KellyController(void)
{
    MotorCmdr_Init(&motorCmdr);
    p_serial = (HardwareSerial *)motorCmdr.Protocol.Xcvr.p_Xcvr->P_CONTEXT;
    p_serial->setTimeout(0);
    millisTimer = millis();
}


/******************************************************************************/
/*!
    @brief Class Destructor
*/
/******************************************************************************/
KellyController::~KellyController(void)
{
    end();
}

/******************************************************************************/
/*!
    @brief Init Arduino Serial
*/
/******************************************************************************/
void KellyController::begin(void)
{
    if(p_serial != 0U) { p_serial->begin(motorCmdr.Protocol.p_Specs->BAUD_RATE_DEFAULT); }

    // ((HardwareSerial *)XCVR_TABLE[0].P_CONTEXT)->begin(motorCmdr.Protocol.p_Specs->BAUD_RATE_DEFAULT);

    //delay?
    // MotorCmdr_InitUnits(&motorCmdr);
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
    Upper layer control TxRx
*/
/******************************************************************************/

/******************************************************************************/
/*!
    @brief Call regularly.
        Sends ping when no Req sent
    @return true when new data is available
*/
/******************************************************************************/
// Protocol_RxCode_T KellyController::poll(void)
// {
//     millisTimer = millis();
//     Protocol_RxCode_T status;

//     // if(_MotorCmdr_PollTimeout(&motorCmdr) == false)
//     // {
//     //     if(p_serial->available() >= _MotorCmdr_GetRespLength(&motorCmdr))
//     //     {
//     //         if(p_serial->peek() != MOT_PACKET_START_BYTE)  /* Serial Rx out of sync */
//     //         {
//     //             for(uint8_t iChar = 0U; iChar < p_serial->available(); iChar++)
//     //             {
//     //                 if(p_serial->peek() != MOT_PACKET_START_BYTE)     { _MotorCmdr_GetPtrRxPacket(&motorCmdr)[iChar] = p_serial->read(); }
//     //                 else                                             { errorLength = iChar; break; }
//     //             }
//     //             status = KELLY_CONTROLLER_RX_ERROR;
//     //         }
//     //         else
//     //         {
//     //             p_serial->readBytes(_MotorCmdr_GetPtrRxPacket(&motorCmdr), _MotorCmdr_GetRespLength(&motorCmdr));
//     //             status = _MotorCmdr_ParseResp(&motorCmdr) ? KELLY_CONTROLLER_RX_SUCCESS : KELLY_CONTROLLER_RX_ERROR; /* new data is ready, returns false if crc error */
//     //         }
//     //     }
//     //     else
//     //     {
//     //         status = KELLY_CONTROLLER_RX_WAITING;
//     //     }
//     // }
//     // else
//     // {
//     //     status = KELLY_CONTROLLER_RX_TIMEOUT;
//     // }

//     // _MotorCmdr_ProcTxIdle(&motorCmdr);

//     MotorCmdr_Proc_Thread(&motorCmdr);

//     return status;
// }

/******************************************************************************/
/*!
    Tx Reqs
*/
/******************************************************************************/
// void KellyController::ping(void)
// {
//     _MotorCmdr_Ping(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeStopAll(void)
// {
//     _MotorCmdr_StopMotors(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeInitUnits(void)
// {
//     MotorCmdr_InitUnits(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeThrottle(uint16_t throttle)
// {
//     _MotorCmdr_WriteThrottle(&motorCmdr, throttle);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeBrake(uint16_t brake)
// {
//     _MotorCmdr_WriteBrake(&motorCmdr, brake);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeSaveNvm(void)
// {
//     _MotorCmdr_SaveNvm(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeRelease(void)
// {
//     _MotorCmdr_WriteRelease(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeDirectionForward(void)
// {
//     _MotorCmdr_WriteDirectionForward(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeDirectionReverse(void)
// {
//     _MotorCmdr_WriteDirectionReverse(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::writeDirectionNeutral(void)
// {
//     _MotorCmdr_WriteDirectionNeutral(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::readSpeed(void)
// {
//     _MotorCmdr_ReadSpeed(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }

// void KellyController::readIFoc(void)
// {
//     _MotorCmdr_ReadIFoc(&motorCmdr);
//     p_serial->write(_MotorCmdr_GetPtrTxPacket(&motorCmdr), _MotorCmdr_GetReqLength(&motorCmdr));
// }


