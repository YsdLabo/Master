#include <ros/ros.h>
#include "odometry_calculator.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_node");
	ros::NodeHandle nh;

	OdometryCalculator oc;
	oc.run();

	ros::spin();

	return 0;
}
