#ifndef CMD_H
#define CMD_H

#include <stdint.h>

#define CMD_RESERVED_RETURN_CODE_SUCCESS 0U

typedef int (*Cmd_Function_T)(int argc, char *argv[]);

typedef const struct
{
	int ReturnCode;
    char * p_ReturnCodeString;
    //errorhandlingfunction

//#ifdef SHELL_OPTION_USE_LIST
//    LIST_NODE_T Link;
//#endif
}
Cmd_ReturnCode_T;

typedef const struct
{
    const char * p_CmdName;
    const char * p_CmdHelp;
    Cmd_Function_T CmdFunction;
    //ExpectedArgCount;
    //Cmd_Mode_T
#ifdef CONFIG_SHELL_USE_LIST
    LIST_NODE_T Link;
#endif
}
Cmd_T;

//#ifdef SHELL_OPTION_USE_ARRAY_TABLE
#define CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) { (#CmdName), (CmdHelpString), (CmdFunction) }
#define RETURN_ENTRY(ReturnCode) { (ReturnCode), (#ReturnCode) }

//#endif

#endif

//
//#ifdef SHELL_OPTION_USE_LIST
//#define DEFINE_CMD_ENTRY(CmdName, CmdHelpString, CmdFunction) CMDLINE_ENTRY_T CmdEntry##CmdName = { (#CmdName), (CmdHelpString), (CmdFunction), {0} }
//#define DEFINE_RETURN_ENTRY(ReturnCode)	RESULT_ENTRY_T ReturnEntry##ReturnCode = { (ReturnCode), (#ReturnCode) }
//void Shell_RegisterReturnCodeEntry(RETURN_CODE_ENTRY_T * result);
//void Shell_RegisterCmdLineEntry(CMDLINE_ENTRY_T * cmd);
//#endif
