#include "Packet.h"



typedef int (*RequestFn_T)(void * p_context, uint8_t * p_buffer);
typedef const struct
{
    RequestFn_T REQUEST;
}
Request_T;

typedef Request_T * (*RequestMapper_T)(void * p_context, uintptr_t id);

typedef struct
{
    // Protocol_ReqState_T ReqState;
    // Protocol_ReqCode_T ReqStatus;    /* Returned from child function, also return to caller. updated per proc. store as way of retaining 2 return values */
    Request_T * p_ReqActive;    /* */
    uint32_t ReqTimeStart;           /* Set on Req Start and Complete */
    uint32_t ReqSubStateIndex; // track ext req index internally?
    // Protocol_ReqContext_T ReqContext; // buffer for passing req parameters.

} RequestHandler_T;


