#include "droneSimple.hpp"

DroneSimple::DroneSimple() : Drone()
{
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSimple::flight_mode_timer_callback, this));
}


void DroneSimple::flight_mode_timer_callback()
{
/*
	if(_offboard_setpoint_counter % 10 == 0)
		RCLCPP_INFO(this->get_logger(), "Odometry\nx: %f\ny: %f\nz: %f\nvx: %f\nvy: %f\nvz: %f", odometry.x.load(), odometry.y.load(), odometry.z.load(), odometry.vx.load(), odometry.vy.load(), odometry.vz.load());
*/

	if (_offboard_setpoint_counter == 0)
		this->setFlightMode(FlightMode::mTakeOff);

	if (_offboard_setpoint_counter == 5)
		this->arm();

	if (_offboard_setpoint_counter == 100)
	{
		this->setFlightMode(FlightMode::mOffboard);
	}

	if (_offboard_setpoint_counter < 300)
	{
		this->publish_offboard_control_mode(OffboardControl::oVelocity);
		this->publish_traj_setp_speed(1.5, 0.5, -0.2, 0.1);
	}

	if (_offboard_setpoint_counter > 300 )
	{
		this->publish_offboard_control_mode(OffboardControl::oRelPos);
		this->publish_traj_setp_position(10.0, -10.0, -10.0, 0.0);
	}

	if (_offboard_setpoint_counter == 500)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	_offboard_setpoint_counter++;
}
