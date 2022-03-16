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

	if (_offboard_setpoint_counter == 190 )
	{
		this->setFlightMode(FlightMode::mOffboard);
	}


	if (_offboard_setpoint_counter < 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0.0, 0.0, -5.0, 1.6);
	}

	if (_offboard_setpoint_counter < 300 && _offboard_setpoint_counter > 200)
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(10, 0.0, -5.0, 0);
	}

	if (_offboard_setpoint_counter < 400 && _offboard_setpoint_counter > 300) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(10, 10, -5.0, 1.6);
	}

	if (_offboard_setpoint_counter < 500 && _offboard_setpoint_counter > 400) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0, 10, -5.0, 3.14);
	}

	if (_offboard_setpoint_counter < 600 && _offboard_setpoint_counter > 500) 
	{
		this->publish_offboard_control_mode();
		this->publish_trajectory_setpoint(0, 0, -5.0, -1.6);
	}

	if (_offboard_setpoint_counter == 600)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	_offboard_setpoint_counter++;
}
