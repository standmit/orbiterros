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

#pragma once

#include "orbitersdk.h"
#include <windows.h>

namespace ros_bridge {

/**
 * \brief	Menu paragraph
 */
class ExtraMenuItem : public LaunchpadItem {
	public:
		ExtraMenuItem(const HINSTANCE& hDLL);

		char* Name() override;
		char* Description() override;
		bool clbkOpen(HWND hLaunchpad) override;
		int clbkWriteConfig() override;

	protected:
		HINSTANCE hModule;
};

}