/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   PDO.c
    @author FireSourcery
    @brief  CANopen PDO parameter objects
*/
/******************************************************************************/
#include "PDO.h"


/******************************************************************************/
/*
    Parameter objects — the PDO records of the communication area

    The index names the record and the channel; the record names the direction. CiA 301 rules, identical for both
    directions:
      COB-ID    11-bit only; not enabled with nothing mapped; a valid PDO's CAN-ID cannot move — disable first
      type      event-driven only — SYNC is not implemented
      mapping   editable only while the PDO is invalid, entries only while COUNT is 0; writing COUNT checks each entry
                is supported and the total fits one frame. The mapped objects are the dictionary's to resolve when the
                PDO runs — an entry it refuses is not written, or is sent as 0.
*/
/******************************************************************************/
/* The channel an index addresses, or NULL. */
static inline PDO_Channel_T * _PDO_ChannelOf(PDO_Tables_T * p_pdos, uint16_t index)
{
    PDO_Table_T * p_table = NULL;
    switch (PDO_OdRecord(index))
    {
        case PDO_OD_RPDO_COMM_PARAM:
        case PDO_OD_RPDO_MAPPING_PARAM:  p_table = &p_pdos->RX; break;
        case PDO_OD_TPDO_COMM_PARAM:
        case PDO_OD_TPDO_MAPPING_PARAM:  p_table = &p_pdos->TX; break;
        default:                         return NULL;
    }
    return (PDO_OdChannel(index) < p_table->COUNT) ? &p_table->P_CHANNELS[PDO_OdChannel(index)] : NULL;
}

static inline OD_Status_T _PDO_Channel_SetCobId(PDO_Channel_T * p_channel, PDO_CobId_T cobId)
{
    bool isAccepted = PDO_CobId_IsSupported(cobId)
        && ((cobId.Invalid == 1U) || ((p_channel->MapCount != 0U) && ((p_channel->CobId.Invalid == 1U) || (cobId.CanId == p_channel->CobId.CanId))));
    if (isAccepted) { p_channel->CobId = cobId; }
    return isAccepted ? OD_OK : OD_ERR_VALUE_RANGE;
}

static inline OD_Status_T _PDO_Channel_SetTransmission(PDO_Channel_T * p_channel, uint8_t type)
{
    bool isAccepted = PDO_Transmission_IsSupported(type);
    if (isAccepted) { p_channel->Transmission = type; }
    return isAccepted ? OD_OK : OD_ERR_VALUE_RANGE;
}

static inline OD_Status_T _PDO_Channel_SetMapCount(PDO_Channel_T * p_channel, uint8_t count)
{
    uint16_t bits = 0U;
    if (count > PDO_MAP_MAX) { return OD_ERR_PDO_LENGTH; }
    for (uint8_t i = 0U; i < count; i++)
    {
        if (PDO_MapEntry_IsSupported(p_channel->Map[i]) == false) { return OD_ERR_NOT_MAPPABLE; }
        bits += p_channel->Map[i].BitLength;
    }
    if (bits > PDO_BITS_MAX) { return OD_ERR_PDO_LENGTH; }
    p_channel->MapCount = count;
    return OD_OK;
}

static inline OD_Status_T _PDO_Channel_SetMapping(PDO_Channel_T * p_channel, uint8_t subindex, uint32_t value)
{
    if (p_channel->CobId.Invalid == 0U) { return OD_ERR_UNSUPPORTED_ACCESS; }            /* disable the PDO first */
    if (subindex == PDO_OD_MAPPING_COUNT) { return _PDO_Channel_SetMapCount(p_channel, (uint8_t)value); }
    if (p_channel->MapCount != 0U) { return OD_ERR_UNSUPPORTED_ACCESS; }                 /* set COUNT to 0 first */
    p_channel->Map[subindex - PDO_OD_MAPPING_ENTRY] = (PDO_MapEntry_T) { .Value = value };
    return OD_OK;
}

/******************************************************************************/
/*
    Object Dictionary Parameters
*/
/******************************************************************************/
static inline OD_Info_T PDO_OdGetInfo(PDO_Tables_T * p_pdos, uint16_t index, uint8_t subindex)
{
    static const OD_Info_T U8_RO  = { .Type = OD_TYPE_U8,  .Access = OD_ACCESS_RO, .Size = 1U };
    static const OD_Info_T U8_RW  = { .Type = OD_TYPE_U8,  .Access = OD_ACCESS_RW, .Size = 1U };
    static const OD_Info_T U16_RW = { .Type = OD_TYPE_U16, .Access = OD_ACCESS_RW, .Size = 2U };
    static const OD_Info_T U32_RW = { .Type = OD_TYPE_U32, .Access = OD_ACCESS_RW, .Size = 4U };
    static const OD_Info_T ABSENT = { .Type = OD_TYPE_NONE };
    PDO_OdRecord_T record = PDO_OdRecord(index);

    if (_PDO_ChannelOf(p_pdos, index) == NULL) { return ABSENT; }
    if (_PDO_OdRecord_IsMapping(record))
    {
        return (subindex == PDO_OD_MAPPING_COUNT) ? U8_RW : (subindex < PDO_OD_MAPPING_ENTRY + PDO_MAP_MAX) ? U32_RW : ABSENT;
    }
    switch (subindex)
    {
        case PDO_OD_COMM_HIGHEST:        return U8_RO;
        case PDO_OD_COMM_COB_ID:         return U32_RW;
        case PDO_OD_COMM_TRANSMISSION:   return U8_RW;
        case PDO_OD_COMM_EVENT_TIMER:    return _PDO_OdRecord_IsTx(record) ? U16_RW : ABSENT;
        default:                         return ABSENT;
    }
}

static inline OD_Status_T PDO_OdGet(PDO_Tables_T * p_pdos, uint16_t index, uint8_t subindex, int32_t * p_value)
{
    const PDO_Channel_T * p_channel = _PDO_ChannelOf(p_pdos, index);
    PDO_OdRecord_T record = PDO_OdRecord(index);

    if (PDO_OdGetInfo(p_pdos, index, subindex).Type == OD_TYPE_NONE) { return OD_ERR_NO_OBJECT; }

    if (_PDO_OdRecord_IsMapping(record))
    {
        *p_value = (subindex == PDO_OD_MAPPING_COUNT) ? p_channel->MapCount : (int32_t)p_channel->Map[subindex - PDO_OD_MAPPING_ENTRY].Value;
    }
    else
    {
        switch (subindex)
        {
            case PDO_OD_COMM_HIGHEST:        *p_value = _PDO_OdRecord_IsTx(record) ? PDO_OD_COMM_EVENT_TIMER : PDO_OD_COMM_TRANSMISSION; break;
            case PDO_OD_COMM_COB_ID:         *p_value = (int32_t)p_channel->CobId.Value;     break;
            case PDO_OD_COMM_TRANSMISSION:   *p_value = p_channel->Transmission;             break;
            default:                         *p_value = p_channel->EventTimer;               break;
        }
    }
    return OD_OK;
}

static inline OD_Status_T PDO_OdSet(PDO_Tables_T * p_pdos, uint16_t index, uint8_t subindex, int32_t value)
{
    PDO_Channel_T * p_channel = _PDO_ChannelOf(p_pdos, index);
    OD_Info_T info = PDO_OdGetInfo(p_pdos, index, subindex);

    if (info.Type == OD_TYPE_NONE) { return OD_ERR_NO_OBJECT; }
    if (info.Access == OD_ACCESS_RO) { return OD_ERR_READ_ONLY; }

    if (_PDO_OdRecord_IsMapping(PDO_OdRecord(index))) { return _PDO_Channel_SetMapping(p_channel, subindex, (uint32_t)value); }
    switch (subindex)
    {
        case PDO_OD_COMM_COB_ID:         return _PDO_Channel_SetCobId(p_channel, (PDO_CobId_T) { .Value = (uint32_t)value });
        case PDO_OD_COMM_TRANSMISSION:   return _PDO_Channel_SetTransmission(p_channel, (uint8_t)value);
        default:                         p_channel->EventTimer = (uint16_t)value; return OD_OK;
    }
}

/******************************************************************************/
/*
    PDO parameters — the communication area over a node's channels, [PDO_Tables_T]
*/
/******************************************************************************/
/* Both directions' PDO parameters; the index names which. Context: the node's [PDO_Tables_T]. */
const OD_Interface_T PDO_PARAM_OD =
{
    .GetInfo = (OD_GetInfoFn_T)PDO_OdGetInfo,
    .Get     = (OD_GetFn_T)PDO_OdGet,
    .Set     = (OD_SetFn_T)PDO_OdSet,
};
