#include "droneSupervised.hpp"

DroneSupervised::DroneSupervised() : DroneRTPS()
{
	_quat_pos_sub = this->create_subscription<geometry_msgs::msg::Quaternion>(
									"PositionQuat",
									10,
									[this](const geometry_msgs::msg::Quaternion::ConstSharedPtr msg){
										_x = msg->x;
										_y = msg->y;
										_z = -1*msg->z;
										_yaw = msg->w;
										_relPosPub_Active = true;
										_timerActive1->cancel();
										_timerActive1 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback1, this));
									});

	_quat_vel_sub = this->create_subscription<geometry_msgs::msg::Quaternion>(
									"VelocityQuat",
									10,
									[this](const geometry_msgs::msg::Quaternion::ConstSharedPtr msg){
										_x = msg->x;
										_y = msg->y;
										_z = -1*msg->z;
										_yaw = msg->w;
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
	if (_offboard_setpoint_counter == 0)
		this->setFlightMode(FlightMode::mTakeOff);

	if (_offboard_setpoint_counter == 5)
		this->arm();

	if(_offboard_setpoint_counter == 100)
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

	if((_velPub_Active && _velPub_stopped ) || (_relPosPub_Active && _resPosPub_stopped))
	{
		this->setFlightMode(FlightMode::mLand);
		rclcpp::shutdown();
	}

	_offboard_setpoint_counter++;
}
