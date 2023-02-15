/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / Kelly Controls Inc

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
    @file     KellyController.h
    @author FireSourcery
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

#define KELLY_XCVR_COUNT (1U)

extern const Xcvr_Xcvr_T KELLY_XCVR_TABLE[KELLY_XCVR_COUNT];

class KellyController
{
private:
    static uint32_t millisTimer;
    MotorCmdr_T motorCmdr = MOTOR_CMDR_INIT(&motorCmdr, KELLY_XCVR_TABLE, KELLY_XCVR_COUNT, &millisTimer);
    HardwareSerial * p_serial; /* Must be public for wrapper access. Handle used for setup only. */
    // Stream * p_serialStream;

public:
    KellyController(void);
    virtual ~KellyController(void);
    void begin(void);
    void end(void);
    void flush(void) { p_serial->flush(); }

    Protocol_RxCode_T procRxReqTxResp(void)
    {
        if(millis() > millisTimer)
        {
            millisTimer = millis();
            MotorCmdr_Proc(&motorCmdr);
            return Protocol_GetRxStatus(&motorCmdr.Protocol);
        }
        else
        {
            return PROTOCOL_RX_CODE_WAIT_PACKET;
        }
    };

    void ping(void)                                 { MotorCmdr_Ping(&motorCmdr); };
    void writeStopAll(void)                         { MotorCmdr_StopMotors(&motorCmdr); };
    void writeInitUnits(void)                         { MotorCmdr_InitUnits(&motorCmdr); };
    void writeThrottle(uint16_t throttle)             { MotorCmdr_WriteThrottle(&motorCmdr, throttle); };
    void writeBrake(uint16_t brake)                 { MotorCmdr_WriteBrake(&motorCmdr, brake); };
    void writeVar(MotVarId_T id, uint32_t value)     { MotorCmdr_WriteVar(&motorCmdr, id, value); };
    // void writeSaveNvm(void)                         { MotorCmdr_SaveNvm(&motorCmdr); };
    // void writeRelease(void)                         { MotorCmdr_WriteRelease(&motorCmdr); };
    // void writeDirectionForward(void)                 { MotorCmdr_WriteDirectionForward(&motorCmdr); };
    // void writeDirectionReverse(void)                 { MotorCmdr_WriteDirectionReverse(&motorCmdr); };
    // void writeDirectionNeutral(void)                 { MotorCmdr_WriteDirectionNeutral(&motorCmdr); };
    // void writeToggleAnalogIn(void)                     { MotorCmdr_WriteToggleAnalogIn(&motorCmdr); };
    void readSpeed(void)                             { MotorCmdr_ReadSpeed(&motorCmdr); };
    // void readIFoc(void)                             { MotorCmdr_ReadIFoc(&motorCmdr); };

    int32_t getReadSpeed(void) { return MotorCmdr_GetReadSpeed(&motorCmdr); }
    int16_t getReadIa(void) { return MotorCmdr_GetReadIa(&motorCmdr); }
    int16_t getReadIb(void) { return MotorCmdr_GetReadIb(&motorCmdr); }
    int16_t getReadIc(void) { return MotorCmdr_GetReadIc(&motorCmdr); }
    int16_t getReadIalpha(void) { return MotorCmdr_GetReadIalpha(&motorCmdr); }
    int16_t getReadIbeta(void) { return MotorCmdr_GetReadIbeta(&motorCmdr); }
    int16_t getReadId(void) { return MotorCmdr_GetReadId(&motorCmdr); }
    int16_t getReadIq(void) { return MotorCmdr_GetReadIq(&motorCmdr); }

    // Debuging use
    uint8_t getTxLength(void) { return motorCmdr.Protocol.TxLength; }
    uint8_t * getPtrTxPacket(void) { return motorCmdr.TxPacket; }
    uint8_t getRxLength(void) { return motorCmdr.Protocol.RxIndex; }
    uint8_t * getPtrRxPacket(void) { return motorCmdr.RxPacket; }


};



#endif