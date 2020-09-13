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


#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "orbiterros_static_transform_publisher");
    ros::NodeHandle nh;

    tf2_ros::StaticTransformBroadcaster tf_broadcaster;

    ros::ServiceClient request_tf_service = nh.serviceClient<std_srvs::Empty>("send_static_tf");

    ros::WallTimer request_tf_timer = nh.createWallTimer(
        ros::WallDuration(2.0),
        [&request_tf_service] (const ros::WallTimerEvent&) mutable {
            if (request_tf_service.exists()) {
                std_srvs::Empty srv;
                constexpr char logger_name[] = "ros.roscpp";
                if (ros::console::set_logger_level(logger_name, ros::console::levels::Fatal))
                    ros::console::notifyLoggerLevelsChanged();
                request_tf_service.call(srv);
                if (ros::console::set_logger_level(logger_name, ros::console::levels::Error))
                    ros::console::notifyLoggerLevelsChanged();
            }
        }
    );

    ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>(
        "ostf",
        1,
        [&tf_broadcaster, &request_tf_timer, request_tf_service] (const tf2_msgs::TFMessageConstPtr& msg) mutable {
            request_tf_timer.stop();
            request_tf_service.shutdown();
            tf_broadcaster.sendTransform(msg->transforms);
            ROS_DEBUG_NAMED("static_transform_publisher", "Static TF from Orbiter received");
        }
    );

    ros::spin();
    return 0;
}
