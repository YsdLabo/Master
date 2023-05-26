//
// DifferentialWheeledRobotController
//
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include"inwheel_motor_controller.h"

#define TRACK_WIDTH  0.276

class DifferentialWheeledRobotController
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Timer timer;
	InwheelMotorController inwheel_motor;
	double dist_r;
	double dist_l;
	double last_dist_r;
	double last_dist_l;

	// parameter
	double track_width;

	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		double linear_x = msg->linear.x;
		double angular_z = msg->angular.z;

		double turning = angular_z * track_width * 0.5;
		double vr = linear_x + turning;
		double vl = linear_x - turning;

		inwheel_motor.update(vr, vl);
	}

public:
	DifferentialWheeledRobotController()
	{
		ros::param::param<double>("~track_width", track_width, TRACK_WIDTH);
	}

	void run()
	{
		dist_r = last_dist_r = 0.0;
		dist_l = last_dist_l = 0.0;

		inwheel_motor.init();
		while(inwheel_motor.get_distance(&last_dist_r, &last_dist_l));
		sub = nh.subscribe("cmd_vel", 10, &DifferentialWheeledRobotController::cmd_vel_callback, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "diff_wheeled_robot_node");

	DifferentialWheeledRobotController  robot;
	robot.run();

	ros::spin();

	return 0;
}

