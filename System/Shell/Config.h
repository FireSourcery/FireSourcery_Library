#ifndef CONFIG_SHELL_H
#define CONFIG_SHELL_H

#if 	defined(CONFIG_SHELL_CMD_LIST)

#elif 	defined(CONFIG_SHELL_CMD_ARRAY)

#else
	#define CONFIG_SHELL_CMD_ARRAY
#endif


#define CONFIG_SHELL_TERMINAL_CONNECT_SERIAL

#endif
