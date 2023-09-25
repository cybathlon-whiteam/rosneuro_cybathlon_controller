#include <ros/ros.h>
#include "rosneuro_cybathlon_controller/NavigationController.h"
#include "rosneuro_cybathlon_controller/AllController.h"


int main(int argc, char** argv) {

	ros::init(argc, argv, "navigation_controller");

	rosneuro::AllController controller;

	if(controller.configure() == false) {
		ROS_ERROR("Cannot configure the navigation controller");
		return -1;
	}

	controller.run();

	ros::shutdown();


	return 0;
}
