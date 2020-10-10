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
#include <string>

namespace ros_bridge {

extern std::string rosmaster_ip;

ROSBridge::ROSBridge(const HINSTANCE& hDLL):
		oapi::Module(hDLL),
		nh(),
		clock_msg(),
		clock_pub("/clock", &clock_msg)
{}

void ROSBridge::clbkPostStep(double, double, double mjd) {
	const double utc = getUTC(mjd);
	clock_msg.clock.fromSec(utc);
	clock_pub.publish(&clock_msg);
	nh.spinOnce();
}

double ROSBridge::getUTC() const {
	return getUTC(GetSimMJD());
}

double ROSBridge::getUTC(const double& mjd) const {
	return 86400.0 * (mjd - 40587.0);
}

void ROSBridge::clbkSimulationStart(RenderMode) {
	nh.initNode(const_cast<char*>(rosmaster_ip.c_str()));
	nh.advertise(clock_pub);
}

}