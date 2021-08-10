#ifndef PATH_H
#define PATH_H

//HAL Macros

#define XSTR(V...) #V
#define HAL_PATH(Path, File)  XSTR(Path/File)

#define HAL_PATH_BOARD(Path, File)  	HAL_PATH(Path/Board/CONFIG_HAL_BOARD, File)
#define HAL_PATH_PLATFORM(Path, File)  	HAL_PATH(Path/Platform/CONFIG_HAL_PLATFORM, File)
#define HAL_PATH_USER(File) 			XSTR(CONFIG_HAL_USER_DIRECTORY/File)

//#if defined(CONFIG_HAL_LIBRARY_DEFINED)
//
//#elif defined(CONFIG_HAL_USER_DEFINED)
//
//#endif


#endif


