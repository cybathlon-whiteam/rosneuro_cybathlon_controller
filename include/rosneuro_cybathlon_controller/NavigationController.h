#ifndef ROSNEURO_CYBATHLON_CONTROLLER_NAVIGATION_
#define ROSNEURO_CYBATHLON_CONTROLLER_NAVIGATION_

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>
#include "rosneuro_cybathlon_controller/NavigationControllerConfig.h"

namespace rosneuro {

using rosneuro_config_cybathlon_controller = rosneuro_cybathlon_controller::NavigationControllerConfig;
using dyncfg_cybathlon_controller = dynamic_reconfigure::Server<rosneuro_config_cybathlon_controller>;


class NavigationController {

	public:
		NavigationController(void);
		virtual ~NavigationController(void);

		virtual bool configure(void);
		virtual void run(void);

	protected:
		virtual void on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg);
		void on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg);
		void on_request_reconfigure(rosneuro_config_cybathlon_controller &config, uint32_t level);
        bool update_if_different(const double& first, double& second, double epsilon = 0.00001);
    virtual float input2control(float input);

    bool has_new_ctrl_;
	bool has_new_eog_;
    float angular_strength_;
    float linear_strength_;
    bool is_discrete_;

    geometry_msgs::Twist ctrl_;

    ros::NodeHandle nh_;
    ros::NodeHandle p_nh_;
		
		float gaussian(float x, float mu, float sigma);
		bool has_class(int refclass, const std::vector<int>& classes);
		int get_class_index(int refclass, const std::vector<int>& classes);

    ros::Publisher	pubctrl_;
		
		std::vector<int> classes_;
 
	private:
		ros::Subscriber subprob_;
		ros::Subscriber subevt_;
		
        //std::vector<float> thresholds_;

		const int cmdmask_ = 6000;


		float ctrl_max_;
		float ctrl_min_;
		float input_max_;
		float input_min_;
		
	dyncfg_cybathlon_controller recfg_srv_;
  	dyncfg_cybathlon_controller::CallbackType recfg_callback_type_;

};

}

#endif
