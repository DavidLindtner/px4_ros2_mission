#ifndef __DRONE_BASE_MAVLINK_HPP__
#define __DRONE_BASE_MAVLINK_HPP__


#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

class DroneMavlink : public rclcpp::Node
{
public:
	DroneMavlink();

	enum class FlightMode
	{ mOffboard, mTakeOff, mLand, mReturnToLaunch, mHold, mMission };

	mavros_msgs::msg::State currentState;
	sensor_msgs::msg::NavSatFix gpsPos;

    void arm();
	void disarm();
    void setFlightMode(FlightMode mode);

	void publish_traj_setp_position(float x, float y, float z, float yaw);
	void publish_traj_setp_speed(float vx, float vy, float vz, float yawspeed);
	void publish_traj_setp_geo(float lat, float lon, float alt);

private:
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

	std::atomic<uint64_t> _timestamp;

	// MAVROS SUB AND PUB AND CLI
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr _state_sub;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_sub;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pos_setp_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _vel_setp_pub;
	rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr _geo_setp_pub;

	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _cmd_cli;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _mode_cli;
};


#endif /*__DRONE_BASE_MAVLINK_HPP__*/