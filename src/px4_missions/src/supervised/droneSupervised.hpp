#ifndef __DRONE_SUPERVISED_HPP__
#define __DRONE_SUPERVISED_HPP__

#include "../base/droneBase.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

class DroneSupervised : public Drone
{
public:
	DroneSupervised();

private:

	std::atomic<float> _x;
	std::atomic<float> _y;
	std::atomic<float> _z;
	std::atomic<float> _yaw;

	bool _relPosPub_Active = false;
	bool _velPub_Active = false;

	bool _resPosPub_stopped = false;
	bool _velPub_stopped = false;

	rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr _quat_pos_sub;
	rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr _quat_vel_sub;

	void timerCallback();

	void timerActiveCallback1();
	void timerActiveCallback2();
	
	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::TimerBase::SharedPtr _timerActive1;
	rclcpp::TimerBase::SharedPtr _timerActive2;
	uint64_t _offboard_setpoint_counter;
};


#endif /*__DRONE_SUPERVISED_HPP__*/