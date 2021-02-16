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
	tf2_pub("/tf", &tf2_msg),
	static_tf2_pub("ostf", &tf2_msg),
	send_statc_tf_service("send_static_tf", &ROSBridge::send_static_tf_cb, this)
{}

inline
 void convert(const VECTOR3& v, geometry_msgs::Vector3& gv) {
	 gv.x = v.x;
	 gv.y = v.y;
	 gv.z = v.z;
}

 void normalize(geometry_msgs::Quaternion& q) {
	 double length = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	 q.x /= length;
	 q.y /= length;
	 q.z /= length;
	 q.w /= length;
 }

void convert(const MATRIX3& m, geometry_msgs::Quaternion& q) {
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
	normalize(q);
}

const MATRIX3 toECEF{
	1.0,  0.0,  0.0,
	0.0,  0.0,  1.0,
	0.0, -1.0,  0.0
};

const MATRIX3 fromECEF{
	1.0,  0.0,  0.0,
	0.0,  0.0, -1.0,
	0.0,  1.0,  0.0
};

const MATRIX3 toROSREP103{
	0.0,  1.0,  0.0,
	0.0,  0.0,  1.0,
	1.0,  0.0,  0.0
};

void GetGlobalTransform(const OBJHANDLE& hObj, geometry_msgs::Transform& transform) {
	VECTOR3 position;
	oapiGetGlobalPos(hObj, &position);
	convert(position, transform.translation);

	MATRIX3 rotation;
	oapiGetRotationMatrix(hObj, &rotation);
	switch (oapiGetObjectType(hObj)) {
		case OBJTP_STAR:
		case OBJTP_PLANET:
			rotation = mul(rotation, toECEF);
			break;
		default:
			rotation = mul(rotation, toROSREP103);
	}
	convert(rotation, transform.rotation);
}

inline void Transpose(MATRIX3& matrix) {
	std::swap(matrix.m12, matrix.m21);
	std::swap(matrix.m13, matrix.m31);
	std::swap(matrix.m23, matrix.m32);
}

void EquToENU(const double& lng, const double& lat, const double& rad, VECTOR3& T, MATRIX3& R) {
	const double sinlng = sin(lng);
	const double coslng = cos(lng);
	const double coslat = cos(-lat + PI05);
	const double sinlat = sin(-lat + PI05);
	
	T.x = rad * coslng * cos(lat);
	T.y = rad * sinlng * cos(lat);
	T.z = rad * sin(lat);

	R.m11 = -1.0 * sinlng;
	R.m12 = -1.0 * coslat * coslng;
	R.m13 = sinlat * coslng;
	R.m21 = coslng;
	R.m22 = -1.0 * coslat * sinlng;
	R.m23 = sinlat * sinlng;
	R.m31 = 0.0;
	R.m32 = sinlat;
	R.m33 = coslat;
}

void EquToENU(const double& lng, const double& lat, const double& rad, geometry_msgs::Transform& transform) {
	VECTOR3 T;
	MATRIX3 R;

	EquToENU(lng, lat, rad, T, R);

	convert(T, transform.translation);
	convert(R, transform.rotation);
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

constexpr char* global_frame_id = "world";

std::vector<std::string> corrected_strings;

const char* removeNonUTF8Symbols(const char* str) {
	bool bad_symbol = false;
	for (uint8_t i = 0; i < strlen(str); ++i) {
		if (str[i] < 0) {
			bad_symbol = true;
			break;
		}
	}
	if (bad_symbol) {
		corrected_strings.emplace_back(str);
		std::string& corr_str = corrected_strings.back();
		std::string::const_iterator it = corr_str.cbegin();
		while (it != corr_str.cend()) {
			if (*it < 0) {
				corr_str.erase(it);
				it = corr_str.cbegin();
			}
			else {
				++it;
			}
		}
		return corr_str.data();
	}
	else {
		return str;
	}
}

std::vector<std::shared_ptr<std::string>> pad_names;

void ROSBridge::clbkSimulationStart(RenderMode) {
	nh.initNode(const_cast<char*>(rosmaster_ip.c_str()));
	nh.advertise(clock_pub);
	nh.advertise(tf2_pub);
	nh.advertise(static_tf2_pub);
	nh.advertiseService(send_statc_tf_service);

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
			geometry_msgs::TransformStamped& obj_transform = *transform_it;
			obj_transform.header = transform_header;
			obj_transform.child_frame_id = removeNonUTF8Symbols(getObjectName(hObj));
			GetGlobalTransform(hObj, obj_transform.transform);
			++transform_it;
			objects.push_back(hObj);

			const uint32_t base_count = oapiGetBaseCount(hObj);
			{
				uint32_t reserve_count = base_count;
				for (uint32_t bi = 0; bi < base_count; ++bi) {
					const OBJHANDLE hBase = oapiGetBaseByIndex(hObj, bi);
					reserve_count += oapiGetBasePadCount(hBase);
				}
				static_transforms.reserve(static_transforms.size() + reserve_count);
			}
			for (uint32_t j = 0; j < base_count; ++j) {
				const OBJHANDLE hBase = oapiGetBaseByIndex(hObj, j);
				static_transforms.emplace_back();
				geometry_msgs::TransformStamped& base_transform = static_transforms.back();
				base_transform.header.frame_id = obj_transform.child_frame_id;
				base_transform.header.stamp = obj_transform.header.stamp;
				base_transform.child_frame_id = removeNonUTF8Symbols(getObjectName(hBase));
				VECTOR3 Tbase;
				MATRIX3 Rbase;
				{
					double lng, lat, rad;
					oapiGetBaseEquPos(hBase, &lng, &lat, &rad);
					EquToENU(lng, lat, rad, Tbase, Rbase);
					convert(Tbase, base_transform.transform.translation);
					convert(Rbase, base_transform.transform.rotation);
				}
				MATRIX3 RbaseT = Rbase;
				Transpose(RbaseT);
				const uint32_t pad_count = oapiGetBasePadCount(hBase);
				pad_names.reserve(pad_names.size() + pad_count);
				for (uint32_t k = 0; k < pad_count; ++k) {
					double lng, lat, rad;
					oapiGetBasePadEquPos(hBase, k, &lng, &lat, &rad);
					VECTOR3 Tpad;
					MATRIX3 Rpad;
					EquToENU(lng, lat, rad, Tpad, Rpad);
					Tpad -= Tbase;
					Tpad = mul(RbaseT, Tpad);
					Rpad = mul(RbaseT, Rpad);
					static_transforms.emplace_back();
					geometry_msgs::TransformStamped& pad_transform = static_transforms.back();
					pad_transform.header.frame_id = base_transform.child_frame_id;
					pad_transform.header.stamp = base_transform.header.stamp;
					pad_names.emplace_back(
						std::move(
							std::make_shared<std::string>(
								std::move(
									std::string(base_transform.child_frame_id) + "/Pad" + std::to_string(k+1)
								)
							)
						)
					);
					pad_transform.child_frame_id = pad_names.back()->c_str();
					convert(Tpad, pad_transform.transform.translation);
					convert(Rpad, pad_transform.transform.rotation);
				}
			}
		}
	}

	publishTF2();

	nh.spinOnce();
}

void ROSBridge::publishTF2() {
	tf2_msg.transforms = transforms.data();
	tf2_msg.transforms_length = transforms.size();
	tf2_pub.publish(&tf2_msg);
}

inline void ROSBridge::publishStaticTF2() {
	tf2_msg.transforms = static_transforms.data();
	tf2_msg.transforms_length = static_transforms.size();
	static_tf2_pub.publish(&tf2_msg);
}

const char* send_static_tf_response = "hello world";

void ROSBridge::send_static_tf_cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	publishStaticTF2();
}

}