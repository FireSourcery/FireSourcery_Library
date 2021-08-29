#ifndef PATH_H
#define PATH_H

//HAL Macros

#define XSTR(V...) #V
#define HAL_PATH(Path, File)  XSTR(Path/File)


//PATH_HAL_BOARD()
#define HAL_PATH_BOARD(Path, File)  	HAL_PATH(Path/Board/CONFIG_HAL_BOARD, File)
#define HAL_PATH_PLATFORM(Path, File)  	HAL_PATH(Path/Platform/CONFIG_HAL_PLATFORM, File)
#define HAL_PATH_USER(File) 			XSTR(CONFIG_HAL_USER_DIRECTORY/File)


//#define CONFIG_HAL_BOARD 			Demo
//#define CONFIG_HAL_PLATFORM 		Demo
//#define CONFIG_HAL_USER_DIRECTORY 	Demo/HAL/Board/Demo

//#if defined(CONFIG_HAL_LIBRARY_DEFINED)
//
//#elif defined(CONFIG_HAL_USER_DEFINED)
//
//#endif


#endif


