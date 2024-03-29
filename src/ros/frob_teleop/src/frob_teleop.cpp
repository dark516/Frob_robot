#include "frob_teleop.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "frob_teleop");
	ros::NodeHandle private_node("~");
	AbotTeleop frobTeleop(private_node);
	ros::spin();
}