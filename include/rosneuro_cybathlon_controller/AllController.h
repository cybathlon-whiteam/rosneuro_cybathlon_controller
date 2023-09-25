#ifndef ROSNEURO_CYBATHLON_CONTROLLER_ALL_
#define ROSNEURO_CYBATHLON_CONTROLLER_ALL_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cmath>

#include "rosneuro_cybathlon_controller/NavigationController.h"

namespace rosneuro {

constexpr int DEFAULT_NUM_BUTTONS = 4;

class AllController : public NavigationController {
    public:
        AllController(void);
        virtual ~AllController(void);

        bool configure(void) override;
        void run(void) override;

    protected:
    		void on_received_neuroprediction(const rosneuro_msgs::NeuroOutput& msg) override;
        float input2control(float input) override;

    private:
        std_msgs::Float32MultiArray status_bar_;
        void set_status_bar();
       
        std_msgs::UInt8MultiArray discrete_cmd_;
        void set_discrete_cmd(int button_id);
      
        std::vector<float> string2vector_converter(std::string msg);

        void decrease_bars();
        void increase_bar(int bar_id);
        
        volatile float bar1_ = 0;
        volatile float bar2_ = 0;
              
        ros::Publisher pub_discrete_cmd_;
        ros::Publisher pub_status_bars_;

        std::vector<float> thresholds_soft_, thresholds_hard_, thresholds_final_, thresholds_initial_;
        std::string string_thresholds_soft_, string_thresholds_hard_, string_thresholds_final_, string_thresholds_initial_;

        float dbar_increment_ = 1.0 / 32.0;

        bool has_new_dbar_   = false;
        bool has_new_button_ = false;

        int digital_key_ = -1;

};

}

#endif