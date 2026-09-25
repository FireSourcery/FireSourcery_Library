#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>


/******************************************************************************/
/*
    container_of
*/
/******************************************************************************/

#define	__DEQUALIFY(type, var)	((type)(uintptr_t)(const volatile void *)(var))
#define	__containerof(x, s, m) __DEQUALIFY(s *, (const volatile char *)(x) - offsetof(s, m))

/*
    Notably Virtual functions pass the interface type
*/
typedef struct VarAccess VarAccess_T;

typedef const struct
{
    int (*GET)(const VarAccess_T * p_context, int varId);
    void (*SET)(VarAccess_T * p_context, int varId, int varValue);
    bool (*TEST_)(const VarAccess_T * p_context, int varId);
}
VarAccess_VTable_T;

typedef struct
{
    int LastAccess;
    int Mode; /* Enable/Disable */
}
VarAccess_RunTime_T;

/* Handler */
typedef struct VarAccess
{
    VarAccess_VTable_T * P_VTABLE;
    VarAccess_RunTime_T * P_RUNTIME;
}
VarAccess_T;

int VarAccess_Get(const VarAccess_T * p_varAccess, int varId)
{
    int varValue = 0;
    if (p_varAccess->P_VTABLE[varId].GET != NULL)
    {
        varValue = p_varAccess->P_VTABLE[varId].GET(p_varAccess, varId);
    }
    return varValue;
}

/* UserT.h/c */
typedef struct
{
    int UserValue1;
    int UserValue2;
}
UserT_T;

typedef const struct
{
    VarAccess_T VAR_ACCESS;
    // VarAccess_Const_T VAR_ACCESS_2;
    UserT_T * P_USER;
}
UserT_Const_T;

int _UserT_GetValue(VarAccess_T * p_varAccess, int varId)
{
    UserT_Const_T * p_userHandle = __containerof(p_varAccess, UserT_Const_T, VAR_ACCESS);
    UserT_T * p_user = p_userHandle->P_USER;
    switch (varId)
    {
        case 0: return p_user->UserValue1;
        case 1: return p_user->UserValue2;
        default: return 0;
    }
}

const VarAccess_VTable_T USER_VAR_ACCESS =
{
    .GET = (int (*)(const VarAccess_T * p_context, int varId))_UserT_GetValue,
    .SET = NULL,
    .TEST_ = NULL,
};

/* main.c */
const UserT_Const_T USER_CONST =
{
    .P_USER = &(UserT_T) { 0 },
    .VAR_ACCESS =
    {
        .P_VTABLE = &USER_VAR_ACCESS,
        .P_RUNTIME = &(VarAccess_RunTime_T) { 0 },
    }
};

void main()
{
    volatile int value = VarAccess_Get(&USER_CONST.VAR_ACCESS, 1);
}