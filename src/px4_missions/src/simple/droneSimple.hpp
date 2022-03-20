#ifndef __DRONE_SIMPLE_HPP__
#define __DRONE_SIMPLE_HPP__

#include "../base/droneBase.hpp"

class DroneSimple : public Drone
{
public:
	DroneSimple();

private:

	void flight_mode_timer_callback();
	rclcpp::TimerBase::SharedPtr _timer;
	uint64_t _offboard_setpoint_counter;
};


#endif /*__DRONE_SIMPLE_HPP__*/