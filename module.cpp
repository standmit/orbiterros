/*
 * Copyright (C) 2020, Andrey Stepanov
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * \file
 * \author Andrey Stepanov
 * \copyright GNU General Public License v3.0
 */

#define STRICT
#define ORBITER_MODULE
#include "windows.h"
#include "orbitersdk.h"
#include "resource.h"
#include <stdio.h>
#include "ROSBridge.h"
#include "ExtraMenu.h"


ros_bridge::ROSBridge* ros_bridge_ptr;
ros_bridge::ExtraMenuItem* extra_menu_item_ptr;


/**
 * \brief Initialization of module
 * \detail
 * This function is called when Orbiter starts or when the module
 * is activated.
 */
DLLCLBK void InitModule (HINSTANCE hModule)
{
	ros_bridge_ptr = new ros_bridge::ROSBridge(hModule);

	extra_menu_item_ptr = new ros_bridge::ExtraMenuItem(hModule);
	oapiRegisterLaunchpadItem(extra_menu_item_ptr);

	oapiRegisterModule(ros_bridge_ptr);
}

/**
 * \brief	Close module
 * \detail
 * This function is called when Orbiter shuts down or when the
 * module is deactivated
 */
DLLCLBK void ExitModule (HINSTANCE hDLL)
{
	oapiUnregisterLaunchpadItem(extra_menu_item_ptr);
	delete extra_menu_item_ptr;
}