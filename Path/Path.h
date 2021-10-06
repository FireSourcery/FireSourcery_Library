/*******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	Path.h
    @author FireSoucery
    @brief  Path Macros fo HAL
    @version V0
*/
/*******************************************************************************/
#ifndef PATH_H
#define PATH_H

#define XSTR(V...) #V
#define PATH(Root, File)  XSTR(Root/File)

#define PATH_BOARD(Root, File)  	PATH(Root/Board/CONFIG_PATH_BOARD, File)
#define PATH_PLATFORM(Root, File)  	PATH(Root/Platform/CONFIG_PATH_PLATFORM, File)
#define PATH_USER(File) 			PATH(CONFIG_PATH_USER, File)

#define HAL_PATH_BOARD(Root, File)  	PATH(Root/Board/CONFIG_PATH_BOARD, File)
#define HAL_PATH_PLATFORM(Root, File)  	PATH(Root/Platform/CONFIG_PATH_PLATFORM, File)
#define HAL_PATH_USER(File) 			PATH(CONFIG_PATH_USER, File)

/* user defines the following path macros */
/*
	#define CONFIG_PATH_BOARD 			Demo_Board
	#define CONFIG_PATH_PLATFORM 		Demo_Platform
	#define CONFIG_PATH_USER 			Demo/HAL/Board/
*/

#endif
