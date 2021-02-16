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
#include "tf2_msgs/TFMessage.h"
#include <vector>


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

		/**
		 * \brief	NodeHandle for ROS
		 * \details
		 * Count of subscribers can't be less than 1.
		 * Two publishers: '/clock' and '/tf'
		 * Input buffer size: Topics request (8 bytes) + Time message (16 bytes) = 24 bytes.
		 * Output buffer size: Topics count response (12 bytes) + Clock topic info (79 bytes) + TF topic info (75 bytes) + Clock message (16 bytes) + Time message(16 bytes) + TF message(4814 bytes for all Solar system's objects and 20 ships nemed by 5 chars names (SH-01)) = 5012 bytes
		 */
		ros::NodeHandle_<WindowsSocket, 1, 2, 24, 5012> nh;

		rosgraph_msgs::Clock clock_msg;		///< Clock message
		ros::Publisher clock_pub;			///< Publisher for clock message

		tf2_msgs::TFMessage tf2_msg;								///< TF2 message
		ros::Publisher tf2_pub;										///< Publisher for TF2
		std::vector<geometry_msgs::TransformStamped> transforms;	///< Container for transforms
		std::vector<OBJHANDLE> objects;
		void publishTF2();
};



}