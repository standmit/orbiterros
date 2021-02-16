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

ROSBridge::ROSBridge(const HINSTANCE& hDLL) :
	oapi::Module(hDLL),
	nh(),
	clock_msg(),
	clock_pub("/clock", &clock_msg),
	tf2_msg(),
	tf2_pub("/tf", &tf2_msg)
{}

inline
 void convert(const VECTOR3& v, geometry_msgs::Vector3& gv) {
	 gv.x = v.x;
	 gv.y = v.y;
	 gv.z = v.z;
}

inline void convert(const MATRIX3& m, geometry_msgs::Quaternion& q) {
	double trace = m.m11 + m.m22 + m.m33 + 1.0;
	if (trace > DBL_EPSILON) {
		const double s = 0.5 / sqrt(trace);
		q.w = 0.25 / s;
		q.x = (m.m32 - m.m23) * s;
		q.y = (m.m13 - m.m31) * s;
		q.z = (m.m21 - m.m12) * s;
	}
	else {
		if (m.m11 > m.m22 && m.m11 > m.m33) {
			const double s = 2.0 * sqrt(1.0 + m.m11 - m.m22 - m.m33);
			q.x = 0.25 * s;
			q.y = (m.m12 + m.m21) / s;
			q.z = (m.m13 + m.m31) / s;
			q.w = (m.m23 - m.m32) / s;
		}
		else if (m.m22 > m.m33) {
			const double s = 2.0 * sqrt(1.0 + m.m22 - m.m11 - m.m33);
			q.x = (m.m12 + m.m21) / s;
			q.y = 0.25 * s;
			q.z = (m.m23 + m.m32) / s;
			q.w = (m.m13 - m.m31) / s;
		}
		else {
			const double s = 2.0 * sqrt(1.0 + m.m33 - m.m11 - m.m22);
			q.x = (m.m13 + m.m31) / s;
			q.y = (m.m23 + m.m32) / s;
			q.z = 0.25 * s;
			q.w = (m.m12 - m.m21) / s;
		}
	}
}

void GetGlobalTransform(const OBJHANDLE& hObj, geometry_msgs::Transform& transform) {
	VECTOR3 position;
	oapiGetGlobalPos(hObj, &position);
	convert(position, transform.translation);

	MATRIX3 rotation;
	oapiGetRotationMatrix(hObj, &rotation);
	switch (oapiGetObjectType(hObj)) {
		case OBJTP_STAR:
		case OBJTP_PLANET: {
			static const MATRIX3 toECEF{
				1.0, 0.0, 0.0,
				0.0, 0.0, 1.0,
				0.0, -1.0, 0.0
			};
			rotation = mul(rotation, toECEF);
			break;
		}
		default: {
			static const MATRIX3 toROSREP103{
				0.0, 1.0, 0.0,
				0.0, 0.0, 1.0,
				1.0, 0.0, 0.0
			};
			rotation = mul(rotation, toROSREP103);
		}
	}
	convert(rotation, transform.rotation);
}

double ROSBridge::getUTC() const {
	return getUTC(GetSimMJD());
}

double ROSBridge::getUTC(const double& mjd) const {
	return 86400.0 * (mjd - 40587.0);
}

void ROSBridge::clbkPostStep(double, double, double mjd) {
	const double utc = getUTC(mjd);

	clock_msg.clock.fromSec(utc);
	clock_pub.publish(&clock_msg);

	{
		std::vector<geometry_msgs::TransformStamped>::iterator transform_it = transforms.begin();
		std::vector<OBJHANDLE>::const_iterator obj_it = objects.cbegin();

		while (obj_it != objects.cend()) {
			transform_it->header.stamp.fromSec(getUTC(mjd));
			GetGlobalTransform(*obj_it, transform_it->transform);
			++obj_it;
			++transform_it;
		}

		publishTF2();
	}

	nh.spinOnce();
}

const char* getObjectName(const OBJHANDLE& hObj) {
	static char obj_name[255];
	oapiGetObjectName(hObj, obj_name, 255);
	char* short_name = reinterpret_cast<char*>(calloc(strlen(obj_name)+1, sizeof(char)));
	strcpy(short_name, obj_name);
	return const_cast<const char*>(short_name);
}

const char* global_frame_id = "world";

void ROSBridge::clbkSimulationStart(RenderMode) {
	nh.initNode(const_cast<char*>(rosmaster_ip.c_str()));
	nh.advertise(clock_pub);
	nh.advertise(tf2_pub);

	{
		std_msgs::Header transform_header;
		transform_header.frame_id = global_frame_id;
		transform_header.stamp.fromSec(getUTC());
		
		const uint32_t obj_count = oapiGetObjectCount();
		transforms.resize(obj_count);
		std::vector<geometry_msgs::TransformStamped>::iterator transform_it = transforms.begin();
		objects.reserve(obj_count);
		for (uint32_t i = 0; i < obj_count; ++i) {
			const OBJHANDLE hObj = oapiGetObjectByIndex(i);
			geometry_msgs::TransformStamped& transform = *transform_it;
			transform.header = transform_header;
			transform.child_frame_id = getObjectName(hObj);
			GetGlobalTransform(hObj, transform.transform);
			++transform_it;
			objects.push_back(hObj);
		}

		publishTF2();
	}

	nh.spinOnce();
}

void ROSBridge::publishTF2() {
	tf2_msg.transforms = transforms.data();
	tf2_msg.transforms_length = transforms.size();
	tf2_pub.publish(&tf2_msg);
}

}