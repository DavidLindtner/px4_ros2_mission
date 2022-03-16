#ifndef __DRONE_SUPERVISED_HPP__
#define __DRONE_SUPERVISED_HPP__

#include "../base/droneBase.hpp"

#include <missions_interfaces/msg/rel_position.hpp>

class DroneSupervised : public Drone
{
public:
	DroneSupervised(std::string vehicleName = "");

private:

	std::atomic<float> _pos_X;
	std::atomic<float> _pos_Y;
	std::atomic<float> _pos_Z;
	std::atomic<float> _yaw;

	bool publisherNotActive = false;

	rclcpp::Subscription<missions_interfaces::msg::RelPosition>::SharedPtr _rel_position_sub;

	void timerCallback();
	void timerActiveCallback();
	
	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::TimerBase::SharedPtr _timerActive;
	uint64_t _offboard_setpoint_counter;
};


#endif /*__DRONE_SUPERVISED_HPP__*/