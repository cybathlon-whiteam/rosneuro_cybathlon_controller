#ifndef ROSNEURO_CYBATHLON_CONTROLLER_ALL_
#define ROSNEURO_CYBATHLON_CONTROLLER_ALL_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/Empty.h>
#include <cmath>

#include "rosneuro_cybathlon_controller/NavigationController.h"

#include "rosneuro_cybathlon_controller/AllControllerConfig.h"
#include "rosneuro_cybathlon_controller/BarsConfig.h"
#include "rosneuro_cybathlon_controller/RangeConfig.h"

namespace rosneuro {

constexpr int DEFAULT_NUM_BUTTONS = 4;

using cybathlon_feedback   = rosneuro_cybathlon_controller::AllControllerConfig;
using dyncfg_feedback_cy   = dynamic_reconfigure::Server<cybathlon_feedback>;

using cybathlon_feedback_bars = rosneuro_cybathlon_controller::BarsConfig;
using dyncfg_feedback_bars    = dynamic_reconfigure::Server<cybathlon_feedback_bars>;

using cybathlon_feedback_range = rosneuro_cybathlon_controller::RangeConfig;
using dyncfg_feedback_range    = dynamic_reconfigure::Server<cybathlon_feedback_range>;

class AllController : public NavigationController {
    public:
        AllController(void);
        virtual ~AllController(void);

        bool configure(void) override;
        void run(void) override;

    protected:
    	void on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) override;
        float input2control(float input) override;
        float input2angular(float input);

        void on_request_reconfigure_f(cybathlon_feedback &config, uint32_t level);  
        void on_request_reconfigure_b(cybathlon_feedback_bars &config, uint32_t level);
        void on_request_reconfigure_r(cybathlon_feedback_range &config, uint32_t level);

    private:
        std_msgs::Float32MultiArray status_bar_;
        void set_status_bar();

        std::vector<double> convert_tresholds_(double offset, double width);
       
        std_msgs::UInt8MultiArray discrete_cmd_;
        void set_discrete_cmd(int button_id);
      
        std::vector<double> string2vector_converter(std::string msg);

        void decrease_bars();
        void increase_bar(int bar_id);

        void request_reset_integration();
        
        volatile float bar1_ = 0;
        volatile float bar2_ = 0;
              
        ros::Publisher pub_discrete_cmd_;
        ros::Publisher pub_status_bars_;

        std::vector<double> thresholds_soft_, thresholds_hard_, thresholds_final_, thresholds_initial_;
        std::string string_thresholds_soft_, string_thresholds_hard_, string_thresholds_final_, string_thresholds_initial_;

        double offset_i_;
        double width_i_;

        float dbar_increment_ ;
        float dbar_decrement_ ;

        bool has_new_dbar_   = false;
        bool has_new_button_ = false;

        bool reset_on_hit    = true;

        int digital_key_ = -1;

		    dyncfg_feedback_cy::CallbackType recfg_callback_type_f_;
        dyncfg_feedback_bars::CallbackType recfg_callback_type_b_;
        dyncfg_feedback_range::CallbackType recfg_callback_type_r_;

        ros::ServiceClient reset_integrator_service;

};
}

#endif
