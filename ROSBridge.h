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
/* TODO ROS
#include <ros/ros.h>
#include <std_msgs/Time.h>
*/

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
		 * \brief	Set Dialog to print time on
		 */
		void attachDlg(const HWND& hDlg);

		/**
		 * \brief	Unset Dialog to print time on
		 * \detail	Call it when dialog is closed
		 */
		void detachDlg();

		/**
		 * \brief	Callback on simulation step
		 */
		void clbkPostStep(double simt, double simdt, double mjd) override;

	protected:
		HWND captionId;			///< Where to print time

		double last_simt;		///< When time was printed last time

		double getUTC() const;	///< Get simulated time in unix format

		/**
		 * \brief	    Convert time from MJD to Unix
		 * \param mjd	Time in MJD format
		 */
		double getUTC(const double& mjd) const;

		/**
		 * \brief	Print current time on window caption (if available)
		 */
		void updateTimeCaption(const double& utc) const;

		/* TODO ROS
		ros::NodeHandle nh;			///< NodeHandle for ROS

		ros::Publisher clock_pub;	///< Publisher for clock message
		*/
};



}