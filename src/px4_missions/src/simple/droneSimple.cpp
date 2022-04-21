#include "droneSimple.hpp"

DroneSimple::DroneSimple() : DroneMavlink()
{
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSimple::flight_mode_timer_callback, this));
}


void DroneSimple::flight_mode_timer_callback()
{

	if(_offboard_setpoint_counter % 20 == 0)
	{
		//RCLCPP_INFO(this->get_logger(), "Odometry\nx: %f\ny: %f\nz: %f\nvx: %f\nvy: %f\nvz: %f", odometry.x.load(), odometry.y.load(), odometry.z.load(), odometry.vx.load(), odometry.vy.load(), odometry.vz.load());
		//RCLCPP_INFO(this->get_logger(), "GPS:\nLat: %f\nLon: %f\nAlt: %f\n", this->gpsPos.latitude, this->gpsPos.longitude, this->gpsPos.altitude);
	}

	if (_offboard_setpoint_counter == 5)
		this->setFlightMode(FlightMode::mTakeOff);

	if (_offboard_setpoint_counter == 10)
		this->arm();

	if (_offboard_setpoint_counter == 80)
		this->setFlightMode(FlightMode::mOffboard);

	if (_offboard_setpoint_counter < 200)
	{
		this->publish_traj_setp_geo(49.328754, 16.573077, 293);
		//this->publish_traj_setp_speed(1.0, 1.0, 0.2, 0.1);
		//this->publish_traj_setp_position(10.0, 10.0, 10.0, 0.0);
	}

	if (_offboard_setpoint_counter > 200 )
	{
		this->publish_traj_setp_geo(49.228754, 16.573077, 298);
		//this->publish_traj_setp_speed(1.0, -1.0, 0.2, -0.1);
		//this->publish_traj_setp_position(10.0, -10.0, 10.0, 2.0);
	}

	if (_offboard_setpoint_counter == 500)
	{
		this->setFlightMode(FlightMode::mLand);
	}

	_offboard_setpoint_counter++;
}
