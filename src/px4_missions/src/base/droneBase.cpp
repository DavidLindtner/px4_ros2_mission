#include "droneBase.hpp"

Drone::Drone(std::string vehicleName) : Node("Drone")
{
	RCLCPP_INFO(this->get_logger(),  "Vehicle name: " + vehicleName);

	_offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>(vehicleName + "fmu/offboard_control_mode/in", 10);
	_trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>(vehicleName + "fmu/trajectory_setpoint/in", 10);
	_vehicle_command_publisher = this->create_publisher<VehicleCommand>(vehicleName + "fmu/vehicle_command/in", 10);

	_timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>(
										vehicleName + "fmu/timesync/out", 
										10, 
										[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
											_timestamp.store(msg->timestamp);
										});

	_vehicle_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
										vehicleName + "fmu/vehicle_odometry/out",
										10,
										[this](px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
											odometry.x.store(msg->x);
											odometry.y.store(msg->y);
											odometry.z.store(msg->z);
											odometry.vx.store(msg->vx);
											odometry.vy.store(msg->vy);
											odometry.vz.store(msg->vz);
											odometry.rollspeed.store(msg->rollspeed);
											odometry.pitchspeed.store(msg->pitchspeed);
											odometry.yawspeed.store(msg->yawspeed);
										});


    //this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER, 175, 4);
}


void Drone::setFlightMode(FlightMode mode)
{
	switch (mode)
	{
		case FlightMode::mOffboard:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			RCLCPP_INFO(this->get_logger(), "Offboard flight mode set");
			break;

		case FlightMode::mTakeOff:
			//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, 0.0, 49.228754, 16.573077, 293);
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);
			RCLCPP_INFO(this->get_logger(), "TakeOff flight mode set");
			break;
			
		case FlightMode::mLand:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			RCLCPP_INFO(this->get_logger(), "Land flight mode set");
			break;
			
		case FlightMode::mReturnToLaunch:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
			RCLCPP_INFO(this->get_logger(), "Return to Launch flight mode set");
			break;
			
		default:
			RCLCPP_INFO(this->get_logger(), "No flight mode set");
	}
}


void Drone::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void Drone::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


void Drone::publish_offboard_control_mode(OffboardControl mode)
{
	OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = mode == OffboardControl::oRelPos;
	msg.velocity = mode == OffboardControl::oVelocity;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	_offboard_control_mode_publisher->publish(msg);
}


void Drone::publish_traj_setp_position(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = x;
	msg.y = y;
	msg.z = z;
	msg.yaw = yaw;
	msg.vx = std::numeric_limits<float>::quiet_NaN();
	msg.vy = std::numeric_limits<float>::quiet_NaN();
	msg.vz = std::numeric_limits<float>::quiet_NaN();
	msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
	_trajectory_setpoint_publisher->publish(msg);
}


void Drone::publish_traj_setp_speed(float vx, float vy, float vz, float yawspeed)
{
	TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = std::numeric_limits<float>::quiet_NaN();
	msg.y = std::numeric_limits<float>::quiet_NaN();
	msg.z = std::numeric_limits<float>::quiet_NaN();
	msg.yaw = std::numeric_limits<float>::quiet_NaN();
	msg.vx = vx;
	msg.vy = vy;
	msg.vz = vz;
	msg.yawspeed = yawspeed;
	_trajectory_setpoint_publisher->publish(msg);
}


void Drone::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
}