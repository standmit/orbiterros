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
#include <functional>

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

VECTOR3 toRightHand(const VECTOR3& v) {
	return { v.x, v.z, v.y };
}

MATRIX3 toRightHand(const MATRIX3& m) {
	return {
		m.m11, m.m13, m.m12,
		m.m31, m.m33, m.m32,
		m.m21, m.m23, m.m22
	};
}

void GetGlobalTransform(const OBJHANDLE& hObj, geometry_msgs::Transform& transform) {
	VECTOR3 position;
	oapiGetGlobalPos(hObj, &position);
	position = toRightHand(position);
	convert(position, transform.translation);

	MATRIX3 rotation;
	oapiGetRotationMatrix(hObj, &rotation);
	rotation = toRightHand(rotation);
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
	clock_msg.clock.fromSec(getUTC(mjd));
	clock_pub.publish(&clock_msg);

	{
		const ros::Time& timestamp = clock_msg.clock;
		auto transform_it = transforms.begin();
		for (const OBJHANDLE& hObj: objects) {
			transform_it->header.stamp = timestamp;
			GetGlobalTransform(hObj, transform_it->transform);
			++transform_it;
		}
	}

	publishTF2();

	nh.spinOnce();
}

void removeNonUTF8Symbols(std::string& str) {
	static const std::function<std::string::const_iterator(const std::string&)> findBadSymbol(
		[](const std::string& str) {
			return std::find_if(
				str.cbegin(),
				str.cend(),
				[](const char& symbol) {
					return (symbol < 0);
				}
			);
		}
	);
	auto bad_symbol = findBadSymbol(str);
	while (bad_symbol != str.cend()) {
		str.erase(bad_symbol);
		bad_symbol = findBadSymbol(str);
	}
}

std::string& ROSBridge::getObjectName(const OBJHANDLE& hObj) {
	auto name_it = names.find(hObj);
	if (name_it == names.cend()) {
		char obj_name[255];
		oapiGetObjectName(hObj, obj_name, 255);
		name_it = names.insert(
			std::move(
				std::make_pair(
					hObj,
					std::move(std::string(obj_name))
				)
			)
		).first;
		removeNonUTF8Symbols(name_it->second);
	}
	return name_it->second;
}

std::string& ROSBridge::getPadName(const OBJHANDLE& hBase, const uint32_t& pad) {
	auto key = std::make_pair(hBase, pad);
	auto name_it = pad_names.find(key);
	if (name_it == pad_names.cend()) {
		name_it = pad_names.insert(
			std::move(
				std::make_pair(
					std::move(key),
					std::move(
						getObjectName(hBase) + "/Pad" + std::to_string(pad + 1)
					)
				)
			)
		).first;
	}
	return name_it->second;
}

constexpr char* global_frame_id = "world";

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
			obj_transform.child_frame_id = getObjectName(hObj).c_str();
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
				base_transform.child_frame_id = getObjectName(hBase).c_str();
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
					pad_transform.child_frame_id = getPadName(hBase, k).c_str();
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

void ROSBridge::send_static_tf_cb(const std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
	publishStaticTF2();
}

}