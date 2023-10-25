#include<ros/ros.h>
#include"quat_to_euler.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_euler");
	QuatToEuler qte;
	qte.run();
	ros::spin();
	return 0;
}
