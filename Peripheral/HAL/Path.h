#ifndef PATH_HAL_PERIPHERAL_H
#define PATH_HAL_PERIPHERAL_H

#define XSTR(String) #String
#define STR(String) XSTR(String)

// HAL_Peripheral.h
// CONFIG_HAL_PERIPHERAL_PATH
// CONFIG_HAL_PERIPHERAL_PLATFORM
// HAL_PERIPHERAL_PATH(File)

#if 	defined(CONFIG_PATH_HAL_PERIPHERAL) 			/* External directory */
	#define PATH_HAL_PERIPHERAL(File) STR(CONFIG_PATH_HAL_PERIPHERAL/File)
#elif 	defined(CONFIG_PATH_HAL_PERIPHERAL_PLATFORM)	/* Library platform directory */
	#define PATH_HAL_PERIPHERAL(File) STR(Peripheral/HAL/Platform/CONFIG_PATH_HAL_PERIPHERAL_PLATFORM/File)
#endif

#endif
