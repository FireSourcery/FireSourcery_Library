// #pragma once

// /******************************************************************************/
// /*!
//     @file   Protocol_FlashLoader_Direct.h
//     @author FireSourcery
//     @brief  The same flash loader, written without the generic DataMode engine.

//     Two handlers against Flash_T directly - no ops table, no interface indirection. The
//     cursor, the chunking and the status replies are written out here instead of being reached
//     through Protocol_DataMode_Ops_T.

//     Wire compatible with the generic version: same request and response payloads, same ids,
//     same pass traces. Register one or the other, never both.

//     See the foot of Protocol_FlashLoader.h for when each is the better choice.
// */
// /******************************************************************************/
// #include "Protocol_DataMode.h"
// #include "../Protocol_Request.h"
// #include "../Packet.h"
// #include "Peripheral/NvMemory/Flash/Flash.h"
// #include "Math/math_general.h"

// #include <stdint.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <string.h>

// /******************************************************************************/
// /*!
//     Wire payloads - identical to the generic engine's, so a host speaks to either.
// */
// /******************************************************************************/
// typedef struct PACKET_PACKED Protocol_FlashLoader_Req  { uint32_t Address; uint32_t Size; uint32_t Config; } Protocol_DataMode_Req_T;
// typedef struct PACKET_PACKED Protocol_FlashLoader_Resp { uint16_t Status; }                                  Protocol_DataMode_Resp_T;

// #define PROTOCOL_FLASH_LOADER_STATUS_OK         (0U)        /* == NV_MEMORY_STATUS_SUCCESS */
// #define PROTOCOL_FLASH_LOADER_STATUS_MALFORMED  (0xE001U)   /* Opening request shorter than its payload type */
// #define PROTOCOL_FLASH_LOADER_STATUS_OVERRUN    (0xE002U)   /* A chunk past the declared size */

// /*
//     Single default instance
// */
// #ifndef PROTOCOL_FLASH_LOADER_READ_ID
// #define PROTOCOL_FLASH_LOADER_READ_ID    (0xDAU)
// #endif

// #ifndef PROTOCOL_FLASH_LOADER_WRITE_ID
// #define PROTOCOL_FLASH_LOADER_WRITE_ID   (0xDBU)
// #endif

// #ifndef PROTOCOL_FLASH_LOADER_DATA_ID
// #define PROTOCOL_FLASH_LOADER_DATA_ID    (0xDDU)
// #endif

// #ifndef PROTOCOL_FLASH_LOADER_DATA_LENGTH
// #define PROTOCOL_FLASH_LOADER_DATA_LENGTH (32U)
// #endif

// /******************************************************************************/
// /*!
//     Context and sub-state

//     The context is what the ops table carried in the generic version, minus the ops: the
//     media instance and the three ids that classify a pass.
// */
// /******************************************************************************/
// // typedef const struct Protocol_FlashLoader
// // {
// //     Flash_T * P_FLASH;
// //     packet_id_t READ_ID;        /* Opens a read, and labels its status replies */
// //     packet_id_t WRITE_ID;       /* Opens a write, and labels its status replies */
// //     packet_id_t DATA_ID;        /* Carries a raw chunk in either direction */
// //     packet_size_t CHUNK_MAX;    /* Bounded by the format's payload capacity */
// // }
// // Protocol_FlashLoader_T;

// // #define PROTOCOL_FLASH_LOADER_DIRECT(p_Flash, ReadId, WriteId, DataId, ChunkMax) \
// // (Protocol_FlashLoader_T)                                                        \
// // {                                                                               \
// //     .P_FLASH    = (p_Flash),                                                    \
// //     .READ_ID    = (packet_id_t)(ReadId),                                        \
// //     .WRITE_ID   = (packet_id_t)(WriteId),                                       \
// //     .DATA_ID    = (packet_id_t)(DataId),                                        \
// //     .CHUNK_MAX  = (packet_size_t)(ChunkMax),                                    \
// // }

// /*!
//     The cursor, held in the socket's P_REQ_CONTEXT buffer.

//     No step field, for the same reason the generic engine has none: P_REQ_CONTEXT is never
//     cleared between exchanges, so stored progress would start each transfer holding whatever
//     the previous one left behind. The pass is derived from the arriving id and the cursor.
// */
// // typedef struct Protocol_FlashLoader_State
// // {
// //     uintptr_t Address;
// //     size_t Size;
// //     size_t Index;
// //     packet_id_t ReqId;      /* Echoed on the status replies */
// //     uint16_t Status;
// // }
// // Protocol_DataMode_State_T;

// /******************************************************************************/
// /*!
//     Shared steps
// */
// /******************************************************************************/
// /*! Stage a status reply. A non-OK status always closes the exchange. */
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Reply(const Protocol_DataMode_State_T * p_state, Packet_Xfer_T * p_xfer, Protocol_DataMode_Resp_T * p_txPayload, uint16_t status, Protocol_ReqCode_T onOk)
// {
//     ((Protocol_DataMode_Resp_T *)p_txPayload)->Status = status;
//     p_xfer->p_TxMeta->Id     = p_state->ReqId;
//     p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);

//     return (status == PROTOCOL_FLASH_LOADER_STATUS_OK) ? onOk : PROTOCOL_REQ_DONE;
// }

// /*!
//     Rewrite the cursor whole and arm the flash controller.

//     Both directions arm the continue-write cursor. A read never advances it, so arming on a
//     read costs nothing and keeps the opening path single.
// */
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Open(Flash_T * p_app, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_txPayload)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     if (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T))
//     {
//         p_state->ReqId = p_xfer->p_RxMeta->Id;
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, PROTOCOL_FLASH_LOADER_STATUS_MALFORMED, PROTOCOL_REQ_DONE);
//     }

//     p_state->Address = (uintptr_t)p_req->Address;
//     p_state->Size    = (size_t)p_req->Size;
//     p_state->Count   = 0U;
//     p_state->ReqId   = p_xfer->p_RxMeta->Id;
//     p_state->Status  = (uint16_t)Flash_SetContinueWrite(p_app, p_state->Address, p_state->Size);

//     /* Nothing to stream: the opening status is also the closing one. */
//     return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, p_state->Status, (p_state->Size == 0U) ? PROTOCOL_REQ_DONE : PROTOCOL_REQ_RESPOND);
// }

// /******************************************************************************/
// /*!
//     Read - device streams flash to the host, paced by the host's acks.

//     Flash is memory mapped for reads, so a chunk is a copy straight into the tx payload.
// */
// /******************************************************************************/
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Read(void * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
// {
//     Protocol_FlashLoader_T * p_app = p_context;
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     /* The opening request. Anything else is a pacing ack - the floor is free, send the next chunk. */
//     if (p_xfer->p_RxMeta->Id == p_app->READ_ID)
//     {
//         return Protocol_FlashLoaderDirect_Open(p_app, p_xfer, (const Protocol_DataMode_Req_T *)p_rxPayload, (Protocol_DataMode_Resp_T *)p_txPayload);
//     }

//     if (p_state->Count >= p_state->Size)
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, PROTOCOL_FLASH_LOADER_STATUS_OK, PROTOCOL_REQ_DONE);
//     }

//     packet_size_t chunk = (packet_size_t)math_min(p_state->Size - p_state->Index, (size_t)p_app->CHUNK_MAX);

//     memcpy(p_txPayload, (const void *)(p_state->Address + p_state->Index), chunk);
//     p_state->Index += chunk;

//     p_xfer->p_TxMeta->Id     = p_app->DATA_ID;
//     p_xfer->p_TxMeta->Length = chunk;
//     return PROTOCOL_REQ_RESPOND;
// }

// /******************************************************************************/
// /*!
//     Write - host streams to flash, paced by its own data frames.

//     Only the opening request and the last chunk earn a reply; the rest are answered by
//     ACCEPT, which is the engine's ack and nothing more.
// */
// /******************************************************************************/
// static inline Protocol_ReqCode_T Protocol_FlashLoaderDirect_Write(Flash_T * p_context, Packet_Xfer_T * p_xfer, const void * restrict p_rxPayload, void * restrict p_txPayload)
// {
//     Protocol_DataMode_State_T * p_state = _DataMode_StateOf(p_xfer->p_Substate);

//     if (p_xfer->p_RxMeta->Id == PROTOCOL_FLASH_LOADER_WRITE_ID)
//     {
//         return Protocol_FlashLoaderDirect_Open(p_context, p_xfer, (const Protocol_DataMode_Req_T *)p_rxPayload, (Protocol_DataMode_Resp_T *)p_txPayload);
//     }

//     /* The ack of the opening reply. Nothing to absorb. */
//     if (p_xfer->p_RxMeta->Id != PROTOCOL_FLASH_LOADER_DATA_ID) { return PROTOCOL_REQ_AWAIT; }

//     /* The remote is ahead of the size it declared. Report rather than overrun. */
//     if ((size_t)p_xfer->p_RxMeta->Length > (p_state->Size - p_state->Index))
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, PROTOCOL_FLASH_LOADER_STATUS_OVERRUN, PROTOCOL_REQ_DONE);
//     }

//     /* Writes continue from the cursor armed by Open, so the address is the controller's to track. */
//     p_state->Status = (uint16_t)Flash_ContinueWrite_Blocking(((Flash_T *)p_context), p_rxPayload, p_xfer->p_RxMeta->Length);

//     if (p_state->Status != PROTOCOL_FLASH_LOADER_STATUS_OK)
//     {
//         return Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, p_state->Status, PROTOCOL_REQ_DONE);
//     }

//     p_state->Index += p_xfer->p_RxMeta->Length;

//     /* Silent while the cursor lasts; the last chunk carries the transfer's outcome back. */
//     return (p_state->Index < p_state->Size)
//          ? PROTOCOL_REQ_ACCEPT
//          : Protocol_FlashLoaderDirect_Reply(p_state, p_xfer, p_txPayload, PROTOCOL_FLASH_LOADER_STATUS_OK, PROTOCOL_REQ_DONE);
// }



// /******************************************************************************/
// /*!
//     Erase - a stateless request, not a transfer.

//     Blocking, and can run to seconds on a large range, so it belongs on a socket whose
//     REQ_TIMEOUT accommodates it or behind a handler that yields. Kept here rather than in the
//     generic engine because erase has no cursor and no chunks - it is one call and one reply.
// */
// /******************************************************************************/
// static inline Protocol_ReqCode_T Protocol_FlashLoader_Erase_Blocking(Flash_T * p_flash, Packet_Xfer_T * p_xfer, const Protocol_DataMode_Req_T * p_req, Protocol_DataMode_Resp_T * p_resp)
// {
//     // if (p_xfer->p_RxMeta->Length < sizeof(Protocol_DataMode_Req_T)) { p_resp->Status = PROTOCOL_DATA_MODE_STATUS_MALFORMED; } // should be caught by parser
//     p_resp->Status = (uint16_t)Flash_Erase_Blocking(p_flash, (uintptr_t)p_req->Address, (size_t)p_req->Size);
//     p_xfer->p_TxMeta->Id = p_xfer->p_RxMeta->Id;
//     p_xfer->p_TxMeta->Length = sizeof(Protocol_DataMode_Resp_T);
//     return PROTOCOL_REQ_DONE;
// }

// // static const Protocol_Req_T PROTOCOL_FLASH_LOADER_DIRECT_READ = { .ID = {.ID = (packet_id_t)(Id), .FRAME_FORMAT = (Format) }, .PROC = (Protocol_ProcReqResp_T)(Proc), .ACK = AckPolicy, };
// // static const Protocol_Req_T PROTOCOL_FLASH_LOADER_DIRECT_WRITE = { .ID = {.ID = (packet_id_t)(Id), .FRAME_FORMAT = (Format) }, .PROC = (Protocol_ProcReqResp_T)(Proc), .ACK = AckPolicy, };
// // static const Protocol_Req_T PROTOCOL_FLASH_LOADER_DIRECT_ERASE = { .ID = {.ID = (packet_id_t)(Id), .FRAME_FORMAT = (Format) }, .PROC = (Protocol_ProcReqResp_T)(Proc), .ACK = AckPolicy, };


// /******************************************************************************/
// /*
//     Registration

//         static Protocol_FlashLoader_T FLASH_LOADER =
//             PROTOCOL_FLASH_LOADER_DIRECT(&Flash, MOT_PACKET_DATA_MODE_READ, MOT_PACKET_DATA_MODE_WRITE,
//                                          MOT_PACKET_DATA_MODE_DATA, MOT_PACKET_PAYLOAD_LENGTH_MAX);

//         static const Protocol_Req_T REQ_TABLE[] =
//         {
//             PROTOCOL_REQ(MOT_PACKET_DATA_MODE_READ,  &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Read,  PROTOCOL_ACK_ON_REQ),
//             PROTOCOL_REQ(MOT_PACKET_DATA_MODE_WRITE, &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Write, PROTOCOL_ACK_ON_REQ),
//             PROTOCOL_REQ(MOT_PACKET_DATA_MODE_ERASE, &MOT_FRAME_DATA, Protocol_FlashLoaderDirect_Erase_Blocking, PROTOCOL_ACK_NONE),
//         };

//     P_REQ_CONTEXT must be at least sizeof(Protocol_DataMode_State_T), and P_APP_CONTEXT is
//     the interface above.
// */
// /******************************************************************************/
