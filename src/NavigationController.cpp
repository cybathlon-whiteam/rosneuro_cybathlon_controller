#include "rosneuro_cybathlon_controller/NavigationController.h"

namespace rosneuro {

NavigationController::NavigationController(void) : p_nh_("~") {

	this->subprob_ = this->nh_.subscribe("/integrator/neuroprediction", 1, 
						   &NavigationController::on_received_neuroprediction, this);
	this->subevt_  = this->nh_.subscribe("/events/bus", 1, 
							&NavigationController::on_received_neuroevent, this);
	this->pubctrl_ = this->nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	// Bind dynamic reconfigure callback
	this->recfg_callback_type_ = boost::bind(&NavigationController::on_request_reconfigure, this, _1, _2);
	this->recfg_srv_.setCallback(this->recfg_callback_type_);


	this->has_new_ctrl_ = false;
    this->has_new_eog_ = false;
}

NavigationController::~NavigationController(void) {

}

bool NavigationController::configure(void) {

	this->ctrl_max_  = 1.0f;	
	this->ctrl_min_  = -1.0f;	
	this->input_max_ = 1.0f;	
	this->input_min_ = 0.0f;	

	// Getting classes
	if(this->p_nh_.getParam("classes", this->classes_) == false) {
		ROS_ERROR("Parameter 'classes' is mandatory");
		return false;
	}	
	
	//// Getting thresholds
	//if(this->p_nh_.getParam("thresholds", this->thresholds_) == false) {
	//	ROS_ERROR("Parameter 'thresholds' is mandatory");
	//	return false;
	//}
	//
	//this->thresholds_.at(1) = 1.0f - this->thresholds_.at(1);

	ros::param::param("~linear_strength",  this->linear_strength_, 1.0f);
	ros::param::param("~angular_strength", this->angular_strength_, 1.0f);
	ros::param::param("~is_discrete", this->is_discrete_, false);

	return true;
}

bool NavigationController::has_class(int refclass, const std::vector<int>& classes) {

	bool retcod = false;
	auto it = std::find(classes.begin(), classes.end(), refclass);

	if(it != classes.end())
		retcod = true;

	return retcod;

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

int NavigationController::get_class_index(int refclass, const std::vector<int>& classes) {

	int index = -1;
	auto it = std::find(classes.begin(), classes.end(), refclass);

	if(it != classes.end())
		index = it - classes.begin();

	return index;
}

void NavigationController::on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) {

	int refclass = this->classes_.at(0);
	int refclassid;
	bool is_class_found = true;
	float ctrl, input;
	std::vector<int> msgclasses = msg.decoder.classes;

	

	if(this->is_discrete_ == true)
		return;

	// First: check that the incoming classes are the ones provided
	for(auto it = msgclasses.begin(); it != msgclasses.end(); ++it)	
		is_class_found = is_class_found && this->has_class(*it, this->classes_);

	if(is_class_found == false) {
		this->has_new_ctrl_ = false;
		ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
		return;
	}
	
	refclassid = this->get_class_index(refclass, msgclasses);

	if(refclassid == -1) {
		this->has_new_ctrl_ = false;
		ROS_WARN_THROTTLE(5.0f, "The incoming neurooutput message does not have the provided classes");
		return;
	} else {
		input = msg.softpredict.data.at(refclassid);
		ctrl = this->input2control(input);
		this->ctrl_.linear.x  = this->linear_strength_ * this->gaussian(ctrl, 0.0, 0.5);
		this->ctrl_.angular.z = this->angular_strength_ * ctrl;
		this->has_new_ctrl_ = true;
	}

}

void NavigationController::on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg) {

	int event = msg.event;
	int classevt = event - this->cmdmask_;
	int refclassid;
	float input, ctrl;

	if(event == 1024){
        this->has_new_eog_ = true;
	}else if(event == 1024+0x8000){
		this->has_new_eog_ = false;
	}

	if(this->is_discrete_ == false)
		return;

	// Manage the reset 
	if(event == 781) {
		input = 0.5f;
		ctrl = this->input2control(input);
		this->ctrl_.linear.x  = this->linear_strength_ * this->gaussian(ctrl, 0.0, 0.5);
		this->ctrl_.angular.z = this->angular_strength_ * ctrl;
		this->has_new_ctrl_ = true;
		return;
	}

	// Manage class commands
	if(this->has_class(classevt, this->classes_) == false)
		return;

	refclassid = this->get_class_index(classevt, this->classes_);

	input = refclassid == 0 ? 1.0f : 0.0f;

	ctrl = this->input2control(input);
	ROS_INFO("input=%f, ctrl=%f\n", input, ctrl);
	this->ctrl_.linear.x  = this->linear_strength_ * this->gaussian(ctrl, 0.0, 0.5);
	this->ctrl_.angular.z = this->angular_strength_ * ctrl;
	this->has_new_ctrl_ = true;

	
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

bool NavigationController::update_if_different(const double& first, double& second, double epsilon){
	bool is_different = false;
	if(std::abs(first - second) >= epsilon){
		second = first;
		is_different = true;
	}
	return is_different;
}


void NavigationController::on_request_reconfigure(rosneuro_config_cybathlon_controller &config, uint32_t level) {

	if( std::fabs(config.linear_strength - this->linear_strength_) > 0.00001) {
		this->linear_strength_ = config.linear_strength;
		ROS_WARN("Changed linear strength to: %f", this->linear_strength_);
	}
	
	if( std::fabs(config.angular_strength - this->angular_strength_) > 0.00001) {
		this->angular_strength_ = config.angular_strength;
		ROS_WARN("Changed angular strength to: %f", this->angular_strength_);
	}

	if(config.is_discrete != this->is_discrete_) {
		this->is_discrete_ = config.is_discrete;
		ROS_WARN("Changed the control modality to %s", this->is_discrete_ ? "discrete" : "continuous");
	}
}



}
