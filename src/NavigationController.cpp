#include "rosneuro_cybathlon_controller/NavigationController.h"

namespace rosneuro {

NavigationController::NavigationController(void) : p_nh_("~") {

	this->subprob_ = this->nh_.subscribe("/integrator/neuroprediction", 1, 
						   &NavigationController::on_received_neuroprediction, this);
	this->subevt_  = this->nh_.subscribe("/events/bus", 1, 
							&NavigationController::on_received_neuroevent, this);
	this->pubctrl_ = this->nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


	this->has_new_ctrl_ = false;
}

NavigationController::~NavigationController(void) {

}

bool NavigationController::configure(void) {

	this->ctrl_max_  = 1.0f;	
	this->ctrl_min_  = -1.0f;	
	this->input_max_ = 1.0f;	
	this->input_min_ = 0.0f;	

	return true;
}

void NavigationController::run(void) {
	
	ros::Rate r(200);
	
	while(ros::ok()) {


		if(this->has_new_ctrl_ == true) {
			this->pubctrl_.publish(this->ctrl_);
			this->has_new_ctrl_ = false;
		}

		ros::spinOnce();
		r.sleep();
	}

}

void NavigationController::on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) {
	
	float ctrl, input;
	input = msg.softpredict.data.at(0);
	ctrl = this->input2control(input);
	this->ctrl_.linear.x  = 0.5f * this->gaussian(ctrl, 0.0, 0.5);
	this->ctrl_.angular.z = 0.5f * ctrl;
	this->has_new_ctrl_ = true;


}

void NavigationController::on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg) {

}

float NavigationController::input2control(float input) {
	float b, a, xmax, xmin, ctrl;

	b    = this->ctrl_max_;
	a    = this->ctrl_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	ctrl = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return ctrl;
}

float NavigationController::gaussian(float x, float mu, float sigma) {

	return exp( - std::pow(x - mu, 2) / (2 * pow(sigma, 2) ) );
}

}
