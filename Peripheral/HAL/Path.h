#ifndef PATH_HAL_PERIPHERAL_H
#define PATH_HAL_PERIPHERAL_H

#define XSTR(String) #String
#define STR(String) XSTR(String)

#if defined(CONFIG_PATH_HAL_PERIPHERAL)
	#define PATH_HAL_PERIPHERAL(File) STR(CONFIG_PATH_HAL_PERIPHERAL/File)

#elif defined(CONFIG_HAL_PLATFORM)
	#define PATH_HAL_PERIPHERAL(File) STR(Peripheral/HAL/Platform/CONFIG_HAL_PLATFORM/File)
#endif

#endif
