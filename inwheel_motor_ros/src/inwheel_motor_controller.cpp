// Inwheel Motor Controller Source

#include "inwheel_motor_controller.h"

InwheelMotorController::InwheelMotorController()
{
	//
	std::string _dev_wheel_right, _dev_wheel_left;
	ros::param::param<std::string>("~dev_wheel_right", _dev_wheel_right, "/dev/ttyAMA1");
	ros::param::param<std::string>("~dev_wheel_left", _dev_wheel_left, "/dev/ttyAMA2");

	motor_right = new InwheelMotorDriver(_dev_wheel_right.c_str());
	motor_left = new InwheelMotorDriver(_dev_wheel_left.c_str());

	ros::param::param<double>("~wheel_radius", wheel_radius, WHEEL_RADIUS);
	radian_per_ticks = 2.0 * M_PI / PulsePerRotate;
	meter_per_ticks = wheel_radius * radian_per_ticks;
	mps_to_rpm = 60.0 / (2.0 * wheel_radius * M_PI);

	rpm_r = 0.0;
	rpm_l = 0.0;
	pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1, this);
	joint_states_timer = nh.createTimer(ros::Duration(0.05), &InwheelMotorController::joint_states_callback, this);
}

InwheelMotorController::~InwheelMotorController()
{
	delete motor_right;
	delete motor_left;
}

void InwheelMotorController::init()
{
	motor_right->motor_stop();
	motor_left->motor_stop();
	motor_right->reset_pulse_count();
	motor_left->reset_pulse_count();
	while(motor_right->get_pulse_count(&count_right_i)!=0);
	while(motor_left->get_pulse_count(&count_left_i)!=0);
	count_left_i *= -1;
	count_right = 0;
	count_left = 0;
	angle_right = 0.0;
	angle_left = 0.0;
}

void InwheelMotorController::update(double vr, double vl)
{
	rpm_r = vr * mps_to_rpm;
	rpm_l = - vl * mps_to_rpm;

//	motor_right->move(rpm_r);
//	motor_left->move(rpm_l);

}

void InwheelMotorController::joint_states_callback(const ros::TimerEvent& e)
{
	// Publish joint_states
	if(motor_right->get_pulse_count(&count_right)==0 && motor_left->get_pulse_count(&count_left)==0)
	{
		angle_right = count_right * radian_per_ticks;
		angle_left = -count_left * radian_per_ticks;

		sensor_msgs::JointState js;
		js.header.stamp = ros::Time::now();
		js.name.resize(2);
		js.name[0] = "wheel_right_joint";
		js.name[1] = "wheel_left_joint";
		js.position.resize(2);
		js.position[0] = angle_right;
		js.position[1] = angle_left;
		pub_joint_states.publish(js);
	}

	motor_right->move(rpm_r);
	motor_left->move(rpm_l);
}


