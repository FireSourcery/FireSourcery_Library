#ifndef CMD_H
#define CMD_H

#include <stdint.h>

typedef int (*Cmd_Function_T)(int argc, char * argv[]);
typedef int (*Cmd_ContextFunction_T)(void * p_context, int argc, char * argv[]);

//enum
//{
// shell
// userdata
// fixedScope
//} Cmd_Type_T

typedef const struct
{
    const char * const P_NAME;
    const char * const P_HELP;
    const Cmd_Function_T FUNCTION;
//    bool IS_CONTEXT_TYPE;
//    Cmd_Type_T
    // ArgCMax;
    //Cmd_Mode_T loop/single loop time
#ifdef CONFIG_SHELL_USE_LIST
    LIST_NODE_T Link;
#endif
}
Cmd_T;


#define CMD_RESERVED_RETURN_CODE_SUCCESS 		(0U)
#define CMD_RESERVED_RETURN_CODE_INVALID_ARGS 	(-1) //Must be implement at cmd function level / per cmd function
#define CMD_RESERVED_RETURN_CODE_ARGS_ERROR		(-2)

//typedef enum
//{
//	CMD_RESERVED_RETURN_CODE_SUCCESS = 0,
//	CMD_RESERVED_RETURN_CODE_INVALID_ARGS = -1,
//}
//Cmd_ReservedReturnCode_T;

typedef const struct
{
	const int CODE_ID;
	const char * const P_STRING;

	//errorhandlingfunction
//#ifdef SHELL_OPTION_USE_LIST
//    LIST_NODE_T Link;
//#endif
}
Cmd_Return_T;


//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
#define CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) { (#CmdName), (CmdHelpString), (CmdFunction) }
#define CMD_RETURN_ENTRY(ReturnCode) { (ReturnCode), (#ReturnCode) }
//#endif

#endif


//#ifdef SHELL_OPTION_USE_LIST
//#define DEFINE_CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) CMDLINE_ENTRY_T CmdEntry##CmdName = { (#CmdName), (CmdHelpString), (CmdFunction), {0} }
//#define DEFINE_RETURN_ENTRY(ReturnCode)	RESULT_ENTRY_T ReturnEntry##ReturnCode = { (ReturnCode), (#ReturnCode) }
//void Shell_RegisterReturnCodeEntry(RETURN_CODE_ENTRY_T * result);
//void Shell_RegisterCmdLineEntry(CMDLINE_ENTRY_T * cmd);
//#endif
