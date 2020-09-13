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

#include "ROSBridge.h"
#include "resource.h"

namespace ros_bridge {

extern char rosmaster_ip[16];
extern char rosmaster_port[6];
extern char ros_ip[16];

ROSBridge::ROSBridge(const HINSTANCE& hDLL):
		oapi::Module(hDLL),
		captionId(0),
		last_simt(0.0)
		/* TODO ROS
		,
		nh(),
		clock_pub(nh.advertise("/clock", 1)
		*/
{}

void ROSBridge::updateTimeCaption(const double& utc) const {
	if (captionId == 0)
		return;

	char timestr[12];
	sprintf(timestr, "%.0f", utc);
	SetWindowText(captionId, timestr);
}

void ROSBridge::attachDlg(const HWND& hDlg){
	captionId = GetDlgItem(hDlg, IDC_ROSBRIDGE_TIME);
	char ip_str[38];
	sprintf(ip_str, "%s:%s %s", rosmaster_ip, rosmaster_port, ros_ip);
	SetWindowText(GetDlgItem(hDlg, IDC_ROSBRIDGE_IP), ip_str);
}

void ROSBridge::detachDlg(){
	captionId = 0;
}

void ROSBridge::clbkPostStep(double simt, double, double mjd) {
	if ((simt - last_simt) > 1.0) {
		const double utc = getUTC(mjd);
		updateTimeCaption(utc);
		/* TODO ROS
		std_msgs::Time clock_msg;
		clock_msg.data.fromSec(utc);
		clock_pub.publish(clock_msg);
		*/
		last_simt = simt;
	}
}

double ROSBridge::getUTC() const {
	return getUTC(GetSimMJD());
}

double ROSBridge::getUTC(const double& mjd) const {
	return 86400.0 * (mjd - 40587.0);
}

}