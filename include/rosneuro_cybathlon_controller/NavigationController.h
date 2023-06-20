#ifndef ROSNEURO_CYBATHLON_CONTROLLER_NAVIGATION_
#define ROSNEURO_CYBATHLON_CONTROLLER_NAVIGATION_

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <rosneuro_msgs/NeuroOutput.h>
#include <rosneuro_msgs/NeuroEvent.h>

namespace rosneuro {

class NavigationController {

	public:
		NavigationController(void);
		virtual ~NavigationController(void);

		bool configure(void);
		void run(void);

	protected:
		void on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg);
		void on_received_neuroevent(const rosneuro_msgs::NeuroEvent& msg);

	private:
		float input2control(float input);
		float gaussian(float x, float mu, float sigma);
		
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;

		ros::Subscriber subprob_;
		ros::Subscriber subevt_;
		ros::Publisher	pubctrl_;

		geometry_msgs::Twist ctrl_;
		bool has_new_ctrl_;
		float ctrl_max_;
		float ctrl_min_;
		float input_max_;
		float input_min_;

};

}

#endif
