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

namespace ros_bridge {

const char cfg_filename[] = "ROSBridge.cfg";
char cfg_rosmaster_ip[] = "ROS_MASTER_IP";
char cfg_rosmaster_port[] = "ROS_MASTER_PORT";
char cfg_ros_ip[] = "ROS_IP";

extern char rosmaster_ip[16];
extern char rosmaster_port[6];
extern char ros_ip[16];

ExtraMenuItem::ExtraMenuItem(const HINSTANCE& hDLL):
		LaunchpadItem(),
		hModule(hDLL)
{
	FILEHANDLE hFile = oapiOpenFile(cfg_filename, FILE_IN, CONFIG);
	oapiReadItem_string(hFile, cfg_rosmaster_ip, rosmaster_ip);
	oapiReadItem_string(hFile, cfg_rosmaster_port, rosmaster_port);
	oapiReadItem_string(hFile, cfg_ros_ip, ros_ip);
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
			SetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_IP),  rosmaster_ip);
			SetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_PORT), rosmaster_port);
			SetWindowText(GetDlgItem(hDlg, IDC_PARAMS_ROS_IP), ros_ip);
			return true;

		case WM_COMMAND:
			switch (LOWORD(wParam)) {
				case IDOK:
					GetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_IP), rosmaster_ip, 16);
					GetWindowText(GetDlgItem(hDlg, IDC_PARAMS_MASTER_PORT), rosmaster_port, 6);
					GetWindowText(GetDlgItem(hDlg, IDC_PARAMS_ROS_IP), ros_ip, 16);
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
	FILEHANDLE hFile = oapiOpenFile(cfg_filename, FILE_OUT, CONFIG);
	oapiWriteItem_string(hFile, cfg_rosmaster_ip, rosmaster_ip);
	oapiWriteItem_string(hFile, cfg_rosmaster_port, rosmaster_port);
	oapiWriteItem_string(hFile, cfg_ros_ip, ros_ip);
	oapiCloseFile(hFile, FILE_OUT);
	return 0;
}

}