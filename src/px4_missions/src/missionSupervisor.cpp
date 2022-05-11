
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <memory>


using namespace std::chrono_literals;

class MissionSupervisor: public rclcpp::Node
{
public:
	MissionSupervisor() : Node("mission_supervisor")
	{
		this->declare_parameter("x");
		this->declare_parameter("y");
		this->declare_parameter("z");
		this->declare_parameter("yaw");
		this->declare_parameter("offboardMode");
		this->declare_parameter("time_s");


		_offboardMode = this->get_parameter("offboardMode").as_int();

		_x = this->get_parameter("x").as_double_array();
		_y = this->get_parameter("y").as_double_array();
		_z = this->get_parameter("z").as_double_array();
		_yaw = this->get_parameter("yaw").as_double_array();

		_time_s = this->get_parameter("time_s").as_double_array();

		// check data format
		if(_x.size() == _y.size() && _x.size() == _z.size() && _x.size() == _yaw.size() && _x.size() == _time_s.size())
			_dataLength = _x.size();
		else
			RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");

		// create publishers
		switch(_offboardMode)
		{
			case 1:
				_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("PositionSetp", 1);
				break;
			case 2:
				_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("VelocitySetp", 1);
				break;
			case 3:
				_geo_pos_pub = this->create_publisher<geographic_msgs::msg::GeoPoint>("GeoPositionSetp", 1);
				break;
			default:
				rclcpp::shutdown();
		}

		_absolute_time = _time_s[0];
		_timer = this->create_wall_timer(500ms, std::bind(&MissionSupervisor::cyclicTimer, this));
	}

private:
	void cyclicTimer()
	{
		if(_offboardMode == 1)
		{
			auto messagePose = geometry_msgs::msg::Pose();
			messagePose.position.x = _x[_sample_counter];
			messagePose.position.y = _y[_sample_counter];
			messagePose.position.z = _z[_sample_counter];
			tf2::Quaternion q;
			q.setRPY(0, 0, _yaw[_sample_counter]);
			messagePose.orientation = tf2::toMsg(q);
			_pose_pub->publish(messagePose);
		}
		else if(_offboardMode == 2)
		{
			auto messageVel = geometry_msgs::msg::Twist();
			messageVel.linear.x = _x[_sample_counter];
			messageVel.linear.y = _y[_sample_counter];
			messageVel.linear.z = _z[_sample_counter];
			messageVel.angular.z = _yaw[_sample_counter];
			_vel_pub->publish(messageVel);
		}
		else
		{
			auto messagePoint = geographic_msgs::msg::GeoPoint();
			messagePoint.latitude = _x[_sample_counter];
			messagePoint.longitude = _y[_sample_counter];
			messagePoint.altitude = _z[_sample_counter];
			_geo_pos_pub->publish(messagePoint);
		}

		if(_cycle_time >= _absolute_time)
		{
			_sample_counter++;
			_absolute_time += _time_s[_sample_counter];
		}

		if(_sample_counter >= _dataLength)
			rclcpp::shutdown();

		_cycle_time += 0.5;
	}

	std::vector<double> _x;
	std::vector<double> _y;
	std::vector<double> _z;
	std::vector<double> _yaw;
	std::vector<double> _time_s;

	int _offboardMode = 0;

	double _cycle_time = 0;
	double _absolute_time = 0;
	int _sample_counter = 0;
	int _dataLength = 0;

	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pose_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub;
	rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr _geo_pos_pub;

	rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MissionSupervisor>());
	rclcpp::shutdown();
	return 0;
}

