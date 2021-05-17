#ifndef CONFIG_SHELL_H
#define CONFIG_SHELL_H


//const CMD_T CMD_TABLE[] =
//{
//
//};

//typedef enum
//{
//	RETURN_CODE_0,
//	RETURN_CODE_ERROR_1,
//} RETURN_CODE_T;

//const RETURN_CODE_T RETURN_CODE_TABLE[] =
//{
//
//};
//
//const RETURN_CODE_T RETURN_CODE_TABLE[] =
//{
//	RESULT_ENTRY(RETURN_CODE_0),
//	RESULT_ENTRY(RETURN_CODE_ERROR_1),
//};


#if 	defined(CONFIG_SHELL_CMD_LIST)

#elif 	defined(CONFIG_SHELL_CMD_ARRAY)

#else
	#define CONFIG_SHELL_CMD_ARRAY
#endif

#endif
