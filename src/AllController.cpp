#include "rosneuro_cybathlon_controller/AllController.h"

namespace rosneuro {

AllController::AllController(void) : NavigationController(){
  this->pub_status_bars_  = this->nh_.advertise<std_msgs::Float32MultiArray>("/bar_status", 1);
  this->pub_discrete_cmd_ = this->nh_.advertise<std_msgs::UInt8MultiArray>("/button", 1);
  
  this->has_new_dbar_   = false;
  this->has_new_button_ = false;
}

AllController::~AllController(void){
}

bool AllController::configure(void) {
  if (NavigationController::configure()) {
    std::string tsts = "0.8, 0.8";
    std::string tsth = "0.9, 0.9";
    std::string tstf = "1.0, 1.0";
    std::string tsti = "0.6, 0.6";

    ros::param::param("~threshold_soft",  this->string_thresholds_soft_,  tsts);
    ros::param::param("~threshold_hard",  this->string_thresholds_hard_,  tsth);
    ros::param::param("~threshold_final", this->string_thresholds_final_, tstf);
    ros::param::param("~threshold_initial", this->string_thresholds_initial_, tsti);


    this->thresholds_initial_ = this->string2vector_converter(this->string_thresholds_initial_);
    this->thresholds_soft_  = this->string2vector_converter(this->string_thresholds_soft_);
    this->thresholds_hard_  = this->string2vector_converter(this->string_thresholds_hard_);
    this->thresholds_final_ = this->string2vector_converter(this->string_thresholds_final_);
  } else
    return false;
  return true;
}

void AllController::run(void) {
	
	ros::Rate r(200);
	
	while(ros::ok()) {

		if(this->has_new_ctrl_ == true) {
			this->pubctrl_.publish(this->ctrl_);
			this->has_new_ctrl_ = false;
		}

    if(this->has_new_dbar_ == true) {
      this->set_status_bar();
      this->pub_status_bars_.publish(this->status_bar_);
      this->has_new_dbar_ = false;
    }

    if(this->has_new_button_ == true) {
      this->set_discrete_cmd(this->digital_key_); 
      this->pub_discrete_cmd_.publish(this->discrete_cmd_);
      this->digital_key_ = -1;
      this->has_new_button_ = false;
    }

		ros::spinOnce();
		r.sleep();
	}

}

std::vector<float> AllController::string2vector_converter(std::string msg){
	// If possible, always prefer std::vector to naked array
  std::vector<float> v;

	msg.replace(msg.find(", "), 2, " ");

  // Build an istream that holds the input string
  std::istringstream iss(msg);

  // Iterate over the istream, using >> to grab floats
  // and push_back to store them in the vector
  std::copy(std::istream_iterator<float>(iss),
        std::istream_iterator<float>(),
        std::back_inserter(v));

  // Put the result on standard out
  std::copy(v.begin(), v.end(),
        std::ostream_iterator<float>(std::cout, ", "));
  std::cout << "\n";
  return v;
	
}

void AllController::on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) {

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

    if (input < this->thresholds_hard_[0] && input > (1 - this->thresholds_hard_[1])) {
		  ctrl = this->input2control(input);
      if (ctrl < 0.0f)
        ctrl = 0.0f;
		  this->ctrl_.linear.x  = this->linear_strength_ * ctrl;
		  this->ctrl_.angular.z = this->angular_strength_ * (1.0f - ctrl);
		  this->has_new_ctrl_ = true;
    } else if (input > this->thresholds_hard_[0]) {
      this->increase_bar(0);
    } else if (input < (1 - this->thresholds_hard_[1])) {
      this->increase_bar(1);
    }

    // TODO: check if the input is greater than trehsold_final and in case reset the integrator

    this->decrease_bars();
	
  }

}

float AllController::input2control(float input) {
  if (input < this->thresholds_initial_[0] && input > (1 - this->thresholds_initial_[1]))
    return 1.0f;
  else if (input > this->thresholds_initial_[0])
    return (this->thresholds_soft_[0] - input) / (this->thresholds_soft_[0] - this->thresholds_initial_[0]);
  else if(input < (1 - this->thresholds_initial_[1]))
    input = (1.0f - input);
    return (this->thresholds_soft_[1] - input) / (this->thresholds_soft_[1] - this->thresholds_initial_[1]);
  return 0.0f;
}

void AllController::set_status_bar() {
	std_msgs::Float32MultiArray msg;
	std::vector<float> tmp = {0,0}; 
	tmp[0] = (float) this->bar1_;
	tmp[1] = (float) this->bar2_;

	msg.data = tmp;
	std_msgs::MultiArrayDimension dim;
	dim.size   = 2;
	dim.stride = 0;

	dim.label  = "current bar status";
	msg.layout.dim.push_back(dim);	

	this->status_bar_ = msg;
}

void AllController::set_discrete_cmd(int button_id) {
	std_msgs::UInt8MultiArray msg;
	msg.data.assign(DEFAULT_NUM_BUTTONS, 0);
	std_msgs::MultiArrayDimension dim;
	dim.size   = 1;
	dim.stride = 0;

	for(auto i = 0; i<DEFAULT_NUM_BUTTONS; i++) {
		dim.label  = "button " + std::to_string(i);
		msg.layout.dim.push_back(dim);
	}

	if (button_id >= 0 & button_id < DEFAULT_NUM_BUTTONS)
		msg.data[button_id] = true;

	this->discrete_cmd_ = msg;
}

void AllController::decrease_bars(){
	this->bar1_ -= this->dbar_increment_ / 4.0;
  this->bar2_ -= this->dbar_increment_ / 4.0;

  if (	this->bar1_ < 0 )
			this->bar1_ = 0;
	if (	this->bar2_ < 0 )
		  this->bar2_ = 0;

  this->has_new_dbar_ = true;
}

void AllController::increase_bar(int index){
	if (index == 0) {
		this->bar1_ += this->dbar_increment_;
		
		if (	bar1_ >= 1.0 ) {
				this->bar1_ = 0.0;
				this->digital_key_ = index;
				this->has_new_button_ = true;
		}
	} else {
		this->bar2_ += this->dbar_increment_;
		
		if (	this->bar2_ >= 1.0 ) {
				this->bar2_ = 0.0;
        this->digital_key_ = index;
				this->has_new_button_ = true;
		}
	}
  this->has_new_dbar_ = true;
}


}