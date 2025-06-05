#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

/*
    UserT implements VarAccess
        VarAccess is an interface and Mixin
        UserT is the container of VarAccess
*/
/******************************************************************************/
/*
    Const Struct Handler
*/
/******************************************************************************/
/*
    So long as P_BASE is defined at compile time. the performace should be the same as container_of
        unless UserT_T runtime contains VarAccess_T
    Verbosity is kept to struct initialization
*/

/*
    Interface.h/c
*/

/* Virtual Interface */
typedef const struct
{
    int (*GET)(const void * p_base, int varId);
    void (*SET)(void * p_base, int varId, int varValue);
    bool (*TEST_)(const void * p_base, int varId);
}
VarAccess_VTable_T;

/* Runtime state - Module Mixin functions */
typedef struct
{
    int LastAccess;
    int Mode; /* Enable/Disable */
}
VarAccess_RunTime_T;

/* Handler - Conveniently, a point to split Opaque Object */
typedef const struct
{
    void * const P_BASE;
    VarAccess_VTable_T * const P_VTABLE;
    VarAccess_RunTime_T * const P_RUNTIME;
}
VarAccess_T;

/* as run time state only */
// typedef const struct
// {
//     void * const p_Base;
//     VarAccess_VTable_T * const p_VTable;
//     VarAccess_RunTime_T RunTime;
// }
// VarAccess_T;

/* API _Interface_  */
int VarAccess_Get(VarAccess_T * p_varAccess, int varId)
{
    int varValue = 0;
    if (p_varAccess->P_VTABLE[varId].GET != NULL)
    {
        varValue = p_varAccess->P_VTABLE[varId].GET(p_varAccess->P_BASE, varId);
    }
    return varValue;
}

/*
    UserT.h/c
*/

typedef struct
{
    int UserValue1;
    int UserValue2;
}
UserT_T;

typedef const struct
{
    VarAccess_T VAR_ACCESS;
    // VarAccess_T VAR_ACCESS_2;
    UserT_T * P_USER;
}
UserT_Const_T;

int _UserT_GetValue(UserT_T * p_user, int varId)
{
    switch (varId)
    {
        case 0: return p_user->UserValue1;
        case 1: return p_user->UserValue2;
        default: return 0;
    }
}

const VarAccess_VTable_T USER_VAR_ACCESS =
{
    .GET = (int (*)(const void *, int))_UserT_GetValue,
    .SET = NULL,
    .TEST_ = NULL,
};


/*
    main.c
*/

UserT_T user1 = { 0 };

const UserT_Const_T USER_CONST =
{
    .P_USER = &user1,
    .VAR_ACCESS =
    {
        .P_BASE = &user1,
        .P_VTABLE = &USER_VAR_ACCESS,
        .P_RUNTIME = &(VarAccess_RunTime_T) { 0 },
    }
};

void main()
{
    volatile int value = VarAccess_Get(&USER_CONST.VAR_ACCESS, 1);
}

