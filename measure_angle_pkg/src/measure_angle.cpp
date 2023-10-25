#include <ros/ros.h>
#include "riemann_sum.h"
//#include "trapezoidal_rule.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "measure_angle");

	RiemannSum riemann;
	riemann.run();
	
	//TrapezoidalRule  trape;
	//trape.run();
	
	ros::spin();
	return 0;
}
