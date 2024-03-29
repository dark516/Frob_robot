
#include "encoder_wiring_pi.hpp"
#include <chrono>
#include <std_msgs/Float64.h>

typedef boost::chrono::steady_clock time_source;

class EncodersPair {
public:
	EncodersPair(double update_rate);

private:
	ros::NodeHandle node;

	ros::Publisher left_wheel_angle_pub;
	ros::Publisher right_wheel_angle_pub;
	ros::Publisher left_wheel_velocity_pub;
	ros::Publisher right_wheel_velocity_pub;

	ros::Timer encoders_timer;

	std_msgs::Float64 left_wheel_angle_msg;
	std_msgs::Float64 right_wheel_angle_msg;
	std_msgs::Float64 left_wheel_velocity_msg;
	std_msgs::Float64 right_wheel_velocity_msg;

	EncoderWiringPi encoder_left;
	EncoderWiringPi encoder_right;

	double left_wheel_angle;
	double right_wheel_angle;
	double left_wheel_velocity;
	double right_wheel_velocity;
	double left_wheel_position;
	double right_wheel_position;

	time_source::time_point last_time;

	void encodersCallback(const ros::TimerEvent& event);
};

EncodersPair::EncodersPair(double update_rate)
	: encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoderPosition1)
	, encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoderPosition2) {
	left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/frob/left_wheel/angle", 1);
	right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/frob/right_wheel/angle", 1);
	left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/frob/left_wheel/current_velocity", 1);
	right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/frob/right_wheel/current_velocity", 1);

	encoders_timer = node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	left_wheel_angle = -1 * encoder_left.getAngle();
	right_wheel_angle = 1 * encoder_right.getAngle();

	left_wheel_angle_msg.data = left_wheel_angle;
	right_wheel_angle_msg.data = right_wheel_angle;

	left_wheel_angle_pub.publish(left_wheel_angle_msg);
	right_wheel_angle_pub.publish(right_wheel_angle_msg);

	double delta_left_wheel = left_wheel_angle - left_wheel_position;
	double delta_right_wheel = right_wheel_angle - right_wheel_position;

	left_wheel_position += delta_left_wheel;
	left_wheel_velocity = delta_left_wheel / elapsed.toSec();

	right_wheel_position += delta_right_wheel;
	right_wheel_velocity = delta_right_wheel / elapsed.toSec();

	left_wheel_velocity_msg.data = left_wheel_velocity;
	right_wheel_velocity_msg.data = right_wheel_velocity;

	left_wheel_velocity_pub.publish(left_wheel_velocity_msg);
	right_wheel_velocity_pub.publish(right_wheel_velocity_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoders");
	EncodersPair encoders_pair(0.01);
	ros::spin();
	return 0;
}