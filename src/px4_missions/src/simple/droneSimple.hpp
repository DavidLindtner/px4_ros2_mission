#ifndef __DRONE_SIMPLE_HPP__
#define __DRONE_SIMPLE_HPP__

#include "../base/droneBaseRTPS.hpp"
#include "../base/droneBaseMavlink.hpp"

class DroneSimple : public DroneMavlink
{
public:
	DroneSimple();

private:

	void flight_mode_timer_callback();
	rclcpp::TimerBase::SharedPtr _timer;
	uint64_t _offboard_setpoint_counter;
};


#endif /*__DRONE_SIMPLE_HPP__*/