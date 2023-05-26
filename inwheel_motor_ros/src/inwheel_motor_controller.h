// Inwheel Motor Controller Header

#include <ros/ros.h>
#include "inwheel_motor_driver.h"

#ifndef _INWHEEL_MOTOR_CONTROLLER_H_
#define _INWHEEL_MOTOR_CONTROLLER_H_

#define WHEEL_RADIUS  0.070
#define PulsePerRotate  4096.0

class InwheelMotorController
{
private:
	ros::NodeHandle nh;
	ros::Publisher pub_joint_states;
	ros::Timer joint_states_timer;

	InwheelMotorDriver *motor_right;
	InwheelMotorDriver *motor_left;

	double wheel_radius;
	double radian_per_ticks;
	double meter_per_ticks;
	double mps_to_rpm;

	int count_right, count_right_i;
	int count_left, count_left_i;

	double distance_right;
	double distance_left;

public:
	InwheelMotorController();
	~InwheelMotorController();

	void init();
	void update(double vr, double vl);
	//int get_distance(double *dist_r, double *dist_l);
};

#endif
