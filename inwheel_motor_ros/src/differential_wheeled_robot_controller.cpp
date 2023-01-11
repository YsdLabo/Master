//
// DifferentialWheeledRobotController
//
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include"inwheel_motor_controller.h"
#include"odometry_calculator.h"

#define TRACK_WIDTH  0.276

class DifferentialWheeledRobotController
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Timer timer;
	InwheelMotorController inwheel_motor;
	OdometryCalculator  odometry;//(TRACK_WIDTH);
	double dist_r;
	double dist_l;
	double last_dist_r;
	double last_dist_l;

	// parameter
	std::string _baudrate, _device;

	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		double linear_x = msg->linear.x;
		double angular_z = msg->angular.z;

		double turning = angular_z * TRACK_WIDTH * 0.5;
		double vr = linear_x + turning;
		double vl = linear_x - turning;

		inwheel_motor.update(vr, vl);
	}

	void timer_callback(const ros::TimerEvent& e)
	{
		if(inwheel_motor.get_distance(&dist_r, &dist_l)==0) {  // 累積移動距離が得られる
			double dr = dist_r - last_dist_r;
			double dl = dist_l - last_dist_l;
			last_dist_r = dist_r;
			last_dist_l = dist_l;

			odometry.update(dr, dl);  // 右・左の微小移動距離を与える
		}
	}

public:
	DifferentialWheeledRobotController()
	{
		dist_r = last_dist_r = 0.0;
		dist_l = last_dist_l = 0.0;

/*		ros::NodeHandle pnh("~");
		double track_width;
		if(pnh.getParamCached("track_width", track_width)) {
			if(track_width > 0.0) ROS_INFO("[%s] set_param::track_width::%lf[m]", ros::this_node::getName().c_str(), track_width);
			else {
				ROS_FATAL("[%s] Value in the track_width must be a positive number.", ros::this_node::getName().c_str());
				exit(1);
			}
		}
		else {
			ROS_FATAL("[%s] The track_width parameter must set a value.", ros::this_node::getName().c_str());
			exit(1);
		}
*/
		// トレッド
		odometry.set_track_width(TRACK_WIDTH);
	}

	void run()
	{
		inwheel_motor.init();
		while(inwheel_motor.get_distance(&last_dist_r, &last_dist_l));
		odometry.init();
		sub = nh.subscribe("cmd_vel", 10, &DifferentialWheeledRobotController::cmd_vel_callback, this);
		timer = nh.createTimer(ros::Duration(0.1), &DifferentialWheeledRobotController::timer_callback, this);
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

