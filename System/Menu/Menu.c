/******************************************************************************/
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
/******************************************************************************/
/******************************************************************************/
/*!
	@file 	Menu.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "Menu.h"

Menu_T	*	p_MenuSelect;

Menu_T * Menu_GetMenu()
{
	return p_MenuSelect;
}

void Menu_SetMenu(Menu_T * target)
{
	p_MenuSelect = target;
}

void Menu_SetNextMenu()
{
	if(p_MenuSelect->NextMenu) p_MenuSelect = p_MenuSelect->NextMenu;
}

void Menu_StartMenu(Menu_T * target)
{
	p_MenuSelect = target;
	if(p_MenuSelect->InitFunction) p_MenuSelect->InitFunction();
}


void Menu_StartNextMenu()
{
	if(p_MenuSelect->NextMenu)
	{
		p_MenuSelect = p_MenuSelect->NextMenu;
		if(p_MenuSelect->InitFunction)	p_MenuSelect->InitFunction();
	}
}

void Menu_ProcFunction(uint8_t num)
{
	if (p_MenuSelect->FunctionMap[num]) p_MenuSelect->FunctionMap[num]();
}

void Menu_ProcMenuFunction(Menu_T * target, uint8_t num)
{
	if (target->FunctionMap[num]) target->FunctionMap[num]();
}
