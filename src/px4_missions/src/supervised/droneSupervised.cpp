#include "droneSupervised.hpp"

DroneSupervised::DroneSupervised(std::string vehicleName) : Drone(vehicleName)
{
	_rel_position_sub = this->create_subscription<missions_interfaces::msg::RelPosition>(
									"relPosition",
									10,
									[this](const missions_interfaces::msg::RelPosition::ConstSharedPtr msg){
										_pos_X = msg->x;
										_pos_Y = msg->y;
										_pos_Z = -1*msg->z;
										_yaw = msg->yaw;
									});

	_timer = this->create_wall_timer(100ms, std::bind(&DroneSupervised::flight_mode_timer_callback, this));
}


void DroneSupervised::flight_mode_timer_callback()
{
	if(_offboard_setpoint_counter == 1 )
	{
		this->arm();
		this->setFlightMode(FlightMode::mOffboard);
	}

	this->publish_offboard_control_mode();
	this->publish_trajectory_setpoint(_pos_X, _pos_Y, _pos_Z, _yaw);

	if(publisherNotActive)
		this->setFlightMode(FlightMode::mLand);

	_offboard_setpoint_counter++;
}
