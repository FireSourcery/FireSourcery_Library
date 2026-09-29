
/******************************************************************************/
/*
    struct hold container context
*/
/******************************************************************************/
// typedef struct
// {
//     // VarAccess_Type_T * P_VTABLE;
//     struct { VarAccess_Var_T * P_VTABLE; size_t TABLE_COUNT; };
//     void * P_CONTAINER;
// }
// VarAccessHoldContext_T;

typedef struct
{
    VarAccess_VTable_T * p_VTable;
    void * p_Container;
    int Mode;
}
VarAccessHoldContext_T;

int VarAccess_Get(VarAccessHoldContext_T * p_varAccess, int varId)
{
    int varValue = 0;
    if (p_varAccess->P_VTABLE[varId].GET != NULL)
    {
        varValue = p_varAccess->P_VTABLE[varId].GET(p_varAccess->P_CONTAINER);
    }
    return varValue;
}

// int _UserT_GetValue1(UserT_T * p_user)
// {
//    return p_user->Value1;
// }

// VarAccess_Var_T UserT_Table[] =
// {
//    [0] = { .ID = 0, .GET = NULL, .SET = NULL, .ON_SET = NULL },
// };

// int UserT_Var_Get(UserT_T * p_user, int varId)
// {
//    return VarAccess_Get(&p_user->VarAccess, varId);
// }

