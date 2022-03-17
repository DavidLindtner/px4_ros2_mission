#include "droneSupervised.hpp"

DroneSupervised::DroneSupervised(std::string vehicleName) : Drone(vehicleName)
{
	_rel_position_sub = this->create_subscription<missions_interfaces::msg::RelPosition>(
									"relPosition",
									10,
									[this](const missions_interfaces::msg::RelPosition::ConstSharedPtr msg){
										_x = msg->x;
										_y = msg->y;
										_z = -1*msg->z;
										_yaw = msg->yaw;
										_relPosPub_Active = true;
										_timerActive1->cancel();
										_timerActive1 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback1, this));
									});

	_velocity_sub = this->create_subscription<missions_interfaces::msg::Velocity>(
									"Velocity",
									10,
									[this](const missions_interfaces::msg::Velocity::ConstSharedPtr msg){
										_x = msg->vx;
										_y = msg->vy;
										_z = -1*msg->vz;
										_yaw = msg->yawspeed;
										_velPub_Active = true;
										_timerActive2->cancel();
										_timerActive2 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback2, this));
									});

	_timerActive1 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback1, this));
	_timerActive2 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback2, this));
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSupervised::timerCallback, this));
}

void DroneSupervised::timerActiveCallback1() { _resPosPub_stopped = true; }
void DroneSupervised::timerActiveCallback2() { _velPub_stopped = true; }

void DroneSupervised::timerCallback()
{
	if(_offboard_setpoint_counter == 5)
		this->arm();

	if(_offboard_setpoint_counter == 10)
		this->setFlightMode(FlightMode::mOffboard);


	if(_relPosPub_Active)
	{
		this->publish_offboard_control_mode(OffboardControl::oRelPos);
		this->publish_traj_setp_position(_x, _y, _z, _yaw);
	}
	if(_velPub_Active)
	{
		this->publish_offboard_control_mode(OffboardControl::oVelocity);
		this->publish_traj_setp_speed(_x, _y, _z, _yaw);
	}
	//RCLCPP_INFO(this->get_logger(), "DATA: %f, %f, %f, %f, %d, %d", _x.load(), _y.load(), _z.load(), _yaw.load(), _relPosPub_Active, _velPub_Active);

	if((_velPub_Active && _velPub_stopped ) || (_relPosPub_Active && _resPosPub_stopped))
	{
		this->setFlightMode(FlightMode::mLand);
		rclcpp::shutdown();
	}

	_offboard_setpoint_counter++;
}
