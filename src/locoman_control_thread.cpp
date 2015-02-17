#include <yarp/os/all.h>
#include <cstdlib>

#include "locoman_control_thread.h"
#include "locoman_constants.h"
#include <GYM/yarp_command_interface.hpp>

locoman_control_thread::locoman_control_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph ):
    control_thread( module_prefix, rf, ph ),command_interface(module_prefix)
{
    left_arm_joints = robot.left_arm.getNumberOfJoints();
    omega = 0.1;
    phi = 10;
    tick = 0;
    max_vel=20;
    left_arm_configuration.resize(left_arm_joints);
}

void locoman_control_thread::link_locoman_params()
{
    // get a shared pointer to param helper
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    // link the left_arm configuration (vector linking)
    ph->linkParam( PARAM_ID_LEFT_ARM, left_arm_configuration.data() );
    // link the max_vel parameter (single value linking
    ph->linkParam( PARAM_ID_MAX_VEL, &max_vel );
}


bool locoman_control_thread::custom_init()
{   
    link_locoman_params();
    robot.setPositionMode();
    robot.setReferenceSpeed(0.2);
    return true;
}

void locoman_control_thread::run()
{   
    std::string cmd = command_interface.getCommand();
   yarp::sig::Vector q_sensed = robot.left_arm.sensePosition();
    std::cout << "Sense : " << q_sensed.toString() << std::endl;
    
    yarp::sig::Vector q_ref(left_arm_joints, 0.0);
    for( int i = 0; i < left_arm_joints; i++) {
	q_ref[i] = q_sensed[i] + sin( (omega * tick / 5) + phi );
    }
    
    std::cout << "Rererence : " << q_ref.toString() << std::endl;
    
    robot.left_arm.move(q_ref);
    
    tick++;
}

bool locoman_control_thread::custom_pause()
{

}

bool locoman_control_thread::custom_resume()
{

}


