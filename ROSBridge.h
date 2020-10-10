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

//////////////////////////////////////
// Avoiding build error
#ifdef ERROR
#ifdef ERROR_BACKUP
#error Cant backup error def
#endif
#define ERROR_BACKUP ERROR
#undef ERROR
#endif

#include "ros.h"

#ifdef ERROR_BACKUP
#define ERROR ERROR_BACKUP
#undef ERROR_BACKUP
#endif
//////////////////////////////////////


#include "rosgraph_msgs/Clock.h"


namespace ros_bridge {

/**
 * \brief	Interface for communication with ROS
 */
class ROSBridge : public oapi::Module {
	public:
		/**
		 * \brief		Main constructor
		 * \param hDLL	ID of this module
		 */
		ROSBridge(const HINSTANCE& hDLL);

		/**
		 * \brief	Callback on simulation step
		 */
		void clbkPostStep(double, double, double mjd) override;

		/**
		 * \brief	Callback on simulation start
		 */
		void clbkSimulationStart(RenderMode) override;

	protected:

		double getUTC() const;	///< Get simulated time in unix format

		/**
		 * \brief	    Convert time from MJD to Unix
		 * \param mjd	Time in MJD format
		 */
		double getUTC(const double& mjd) const;

		ros::NodeHandle nh;			///< NodeHandle for ROS

		rosgraph_msgs::Clock clock_msg;		///< Clock message
		ros::Publisher clock_pub;			///< Publisher for clock message
};



}