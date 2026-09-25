/******************************************************************************/
/*
    function parameter
*/
/******************************************************************************/
/*
    Virtual Interface
*/
typedef const struct
{
    int (*GET)(const void * p_context, int varId);
    void (*SET)(void * p_context, int varId, int varValue);
    bool (*TEST_)(const void * p_context, int varId);
}
VarAccess_VTable_T;

typedef struct
{
    int LastAccess;
    int Mode; /* Enable/Disable */
}
VarAccess_RunTime_T;


/* Handler */
typedef const struct
{
    VarAccess_VTable_T * const P_VTABLE;
    VarAccess_RunTime_T * const P_RUNTIME;
}
VarAccess_T;


/* API Module _Interface_ */
int VarAccess_Get(VarAccess_T * p_varAccess, const void * p_context, int varId)
{
    int varValue = 0;
    if (p_varAccess->P_VTABLE[varId].GET != NULL)
    {
        varValue = p_varAccess->P_VTABLE[varId].GET(p_context);
    }
    return varValue;
}


/* UserT */
typedef struct
{
    int UserValue1;
    int UserValue2;
    // VarAccess_Const_T VarAccess;
}
UserT_T;

typedef const struct
{
    VarAccess_T VAR_ACCESS;
    // VarAccess_Const_T VAR_ACCESS_2;
    UserT_T * P_USER;
}
UserT_Const_T;


int _UserT_GetValue(UserT_Const_T * p_userHandle, int varId)
{
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
    .GET = (int (*)(const void *, int))_UserT_GetValue,
    .SET = NULL,
    .TEST_ = NULL,
};


/* main.c */
UserT_Const_T USER_CONST =
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
    int value = VarAccess_Get(&USER_CONST.VAR_ACCESS, &USER_CONST, 1);
}