#ifndef LOCOMAN_CONTROL_THREAD_H_
#define LOCOMAN_CONTROL_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/sig/Vector.h>

#include <idynutils/yarp_single_chain_interface.h>
#include <GYM/yarp_command_interface.hpp>
#include <GYM/control_thread.hpp>

/**
 * @brief The locoman_control_thread class inherit from a control_thread
 */
class locoman_control_thread: public control_thread
{
private:   

    walkman::yarp_command_interface command_interface;

    // link the locoman optional params
    void link_locoman_params();
    
    //Example stuff (for sin wave generation)
    unsigned int left_arm_joints;
    double omega;
    double phi;
    int tick;
    
    
    //Example stuff for parameters setting from outside
    yarp::sig::Vector left_arm_configuration;
    double max_vel;
public:
    /**
     * @brief tutorial_control_thread constructor
     * @param module_prefix passed from the module
     * @param rf passed from the module
     * @param ph a param helper builted while generating the control thread
     */
    locoman_control_thread(    std::string module_prefix, 
                                yarp::os::ResourceFinder rf, 
                                std::shared_ptr<paramHelp::ParamHelperServer> ph );

    /**
     * @brief custom_init we use this method when a "start" is sent to the module
     * @return true on success, false otherwise
     */
    virtual bool custom_init();

    /**
     * @brief run we use this one to compute our super control!
     */
    virtual void run();
    
    /**
     * @brief custom_pause we use this method when a "pause" is sent to the module
     * @return true on success, false otherwise
     */
    virtual bool custom_pause();

    /**
     * @brief custom_resume we use this method when a "resume" is sent to the module
     * @return true on success, false otherwise
     */
    virtual bool custom_resume();
        
    yarp::sig::Vector senseMotorPosition( ) ;
    
};

#endif
