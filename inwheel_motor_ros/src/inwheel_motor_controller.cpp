// Inwheel Motor Controller Source

#include "inwheel_motor_controller.h"

InwheelMotorController::InwheelMotorController()
{
	//
	std::string _right_dev, _left_dev;
	ros::param::param<std::string>("~right_wheel_dev", _right_dev, "/dev/ttyAMA1");
	ros::param::param<std::string>("~left_wheel_dev", _left_dev, "/dev/ttyAMA2");

	motor_right = new InwheelMotorDriver(_right_dev.c_str());
	motor_left = new InwheelMotorDriver(_left_dev.c_str());

	wheel_radius = WHEEL_RADIUS;
	radian_per_ticks = 2.0 * M_PI / PulsePerRotate;
	meter_per_ticks = wheel_radius * radian_per_ticks;
	mps_to_rpm = 60.0 / (2.0 * wheel_radius * M_PI);
}

InwheelMotorController::~InwheelMotorController()
{
	delete motor_right;
	delete motor_left;
}

void InwheelMotorController::init()
{
	motor_right->motor_on();
	motor_left->motor_on();
	while(motor_right->get_pulse_count(&count_right_i)!=0);
	while(motor_left->get_pulse_count(&count_left_i)!=0);
	count_right = 0;
	count_left = 0;
	distance_right = 0.0;
	distance_left = 0.0;
}

void InwheelMotorController::update(double vr, double vl)
{
	double rpm_r = vr * mps_to_rpm;
	double rpm_l = - vl * mps_to_rpm;

	motor_right->move(rpm_r);
	motor_left->move(rpm_l);
}

int InwheelMotorController::get_distance(double *dist_r, double *dist_l)
{
	int count_r, count_l;
	if(motor_right->get_pulse_count(&count_r)==0 && motor_left->get_pulse_count(&count_l)==0)
	{
		count_right = count_r - count_right_i;
		count_left  = count_l - count_left_i;
		distance_right = count_right * meter_per_ticks;
		distance_left = count_left * meter_per_ticks;
		return 0;    // success
	}
	return -1;    // failed
}



