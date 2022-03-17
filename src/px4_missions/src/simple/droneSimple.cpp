#include "droneSimple.hpp"

DroneSimple::DroneSimple(std::string vehicleName) : Drone(vehicleName)
{
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSimple::flight_mode_timer_callback, this));
}


void DroneSimple::flight_mode_timer_callback()
{
	if (_offboard_setpoint_counter == 25)
	{
		this->setFlightMode(FlightMode::mTakeOff);
		this->arm();
	}

	if (_offboard_setpoint_counter == 150 )
	{
		this->setFlightMode(FlightMode::mOffboard);
	}

	if (_offboard_setpoint_counter < 400 )
	{
		this->publish_offboard_control_mode(OffboardControl::oVelocity);
		this->publish_traj_setp_speed(1.5, 0.5, -0.2, 0.1);
	}


	if (_offboard_setpoint_counter > 400 )
	{
		this->publish_offboard_control_mode(OffboardControl::oRelPos);
		this->publish_traj_setp_position(10.0, -10.0, -10.0, 0.0);
	}


	if (_offboard_setpoint_counter == 600)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	_offboard_setpoint_counter++;
}
