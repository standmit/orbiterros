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

#include "ExtraMenu.h"
#include "resource.h"
#include <string>

namespace ros_bridge {

const std::string cfg_filename("ROSBridge.cfg");
const std::string cfg_rosmaster_ip("ROS_MASTER_IP");

extern std::string rosmaster_ip;

ExtraMenuItem::ExtraMenuItem(const HINSTANCE& hDLL):
		LaunchpadItem(),
		hModule(hDLL)
{
	FILEHANDLE hFile = oapiOpenFile(cfg_filename.c_str(), FILE_IN, CONFIG);
	char strbuf[255];
	oapiReadItem_string(hFile, const_cast<char*>(cfg_rosmaster_ip.c_str()), strbuf);
	rosmaster_ip = strbuf;
	oapiCloseFile(hFile, FILE_IN);
}

char* ExtraMenuItem::Name() {
	return "ROS Bridge";
}

char* ExtraMenuItem::Description() {
	return "Parameters of ROS Bridge";
}

BOOL CALLBACK ParamDlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam) {
	switch (uMsg) {
		case WM_INITDIALOG:
			SetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_IP),  rosmaster_ip.c_str());
			return true;

		case WM_COMMAND:
			switch (LOWORD(wParam)) {
				case IDOK:
					char  strbuf[255];
					GetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_IP), strbuf, 22);
					rosmaster_ip = strbuf;
					EndDialog(hDlg, 0);
					return true;

				case IDCANCEL:
					EndDialog(hDlg, 0);
					return true;

				default:
					break;
			}
			break;

		default:
			break;
	}

	return oapiDefDialogProc(hDlg, uMsg, wParam, lParam);
}

bool ExtraMenuItem::clbkOpen(HWND hLaunchpad) {
	DialogBox(hModule, MAKEINTRESOURCE(IDD_PARAMS_DIALOG), hLaunchpad, ParamDlgProc);
	return true;
}

int ExtraMenuItem::clbkWriteConfig() {
	FILEHANDLE hFile = oapiOpenFile(cfg_filename.c_str(), FILE_OUT, CONFIG);
	oapiWriteItem_string(hFile, const_cast<char*>(cfg_rosmaster_ip.c_str()), const_cast<char*>(rosmaster_ip.c_str()));
	oapiCloseFile(hFile, FILE_OUT);
	return 0;
}

}