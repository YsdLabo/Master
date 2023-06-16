#include <ros/ros.h>
#include "sma_filter.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "digital_filter");
	
	SMA sma(5);
	sma.run();
	ros::spin();
	return 0;
}
