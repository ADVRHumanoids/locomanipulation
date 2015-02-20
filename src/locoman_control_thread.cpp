#include <yarp/os/all.h>
#include <cstdlib>
#include <yarp/math/Math.h>
//#include <yarp/sig/all.h>
#include "locoman_control_thread.h"
#include "locoman_constants.h"
#include <GYM/yarp_command_interface.hpp>


using namespace yarp::math;



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
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    link_locoman_params();
    
    robot.idynutils.setFloatingBaseLink("base_link");
    yarp::sig::Vector q_current = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
    robot.setPositionDirectMode();
    return true;
}

void locoman_control_thread::run()
{   
    
    yarp::sig::Vector q_current = robot.sensePosition() ;
    robot.idynutils.updateiDyn3Model( q_current, true ); //update model first
    
    yarp::sig::Vector tau_current = robot.senseTorque() ;

    double a =1.0/1000.0 ; 

    yarp::sig::Vector q_current_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_current_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_current_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_current_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_current_left_leg(robot.left_leg.getNumberOfJoints()) ;
    //
    robot.fromIdynToRobot(  q_current,
                            q_current_right_arm,
                            q_current_left_arm,
                            q_current_torso,
                            q_current_right_leg,
                            q_current_left_leg  ) ;
			    
    yarp::sig::Vector tau_current_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector tau_current_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector tau_current_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector tau_current_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector tau_current_left_leg(robot.left_leg.getNumberOfJoints()) ;
  
    robot.fromIdynToRobot(  tau_current,
                            tau_current_right_arm,
                            tau_current_left_arm,
                            tau_current_torso,
                            tau_current_right_leg,
                            tau_current_left_leg ) ;

    yarp::sig::Vector q_ref_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_ref_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_left_leg(robot.left_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref(robot.getNumberOfJoints()) ;
  //  q_ref_left_arm[0] += .01;
    
    q_ref_torso = q_current_torso ;  //  (1/1000)*tau_current_torso + q_current_torso 
    
    q_ref_right_arm  =  q_current_right_arm	;
    q_ref_left_arm   =  q_current_left_arm 	;
    q_ref_right_leg  =  q_current_right_leg 	;
    q_ref_left_leg   =  q_current_left_leg 	;
    
    
    yarp::sig::Matrix C_right_arm( robot.right_arm.getNumberOfJoints() ,  robot.right_arm.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_left_arm(  robot.left_arm.getNumberOfJoints()  ,  robot.left_arm.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_torso(     robot.torso.getNumberOfJoints()     ,  robot.torso.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_right_leg( robot.right_leg.getNumberOfJoints() ,  robot.right_leg.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_left_leg(  robot.left_leg.getNumberOfJoints()  ,  robot.left_leg.getNumberOfJoints() ) ;

    
    //---------------------------------------------------------------------------//
    // RIGHT ARM
    
    yarp::sig::Vector C_vec_right_arm(  robot.right_arm.getNumberOfJoints() ) ;
   
    C_vec_right_arm[0] = 1.0/1000.0 ;
    C_vec_right_arm[1] = 1.0/1000.0 ;
    C_vec_right_arm[2] = 1.0/600.0 ;
    C_vec_right_arm[3] = 1.0/1000.0 ;
    C_vec_right_arm[4] = 1.0/100.0 ;
    C_vec_right_arm[5] = 1.0/100.0 ;
    C_vec_right_arm[6] = 1.0/10.0 ;

    C_right_arm.diagonal( C_vec_right_arm );
    
    //---------------------------------------------------------------------------//
    // LEFT ARM
    
    yarp::sig::Vector C_vec_left_arm(  robot.left_arm.getNumberOfJoints() ) ;
   
    C_vec_left_arm[0] = 1.0/1000.0 ;
    C_vec_left_arm[1] = 1.0/1000.0 ;
    C_vec_left_arm[2] = 1.0/600.0 ;
    C_vec_left_arm[3] = 1.0/1000.0 ;
    C_vec_left_arm[4] = 1.0/100.0 ;
    C_vec_left_arm[5] = 1.0/100.0 ;
    C_vec_left_arm[6] = 1.0/10.0 ;

    C_left_arm.diagonal( C_vec_left_arm );
     
    //--------------------------------------------------------------------------//
    //TORSO
    
    yarp::sig::Vector C_vec_torso(  robot.torso.getNumberOfJoints() ) ;
   
    C_vec_torso[0] = 1.0/1000.0 ;
    C_vec_torso[1] = 1.0/1000.0 ;
    C_vec_torso[2] = 1.0/1000.0 ;


    C_torso.diagonal( C_vec_torso );


    //---------------------------------------------------------------------------//
    // RIGHT LEG
    
    yarp::sig::Vector C_vec_right_leg(  robot.right_leg.getNumberOfJoints() ) ;
   
    C_vec_right_leg[0] = 1.0/3000.0 ;
    C_vec_right_leg[1] = 1.0/5000.0 ;
    C_vec_right_leg[2] = 1.0/3000.0 ;
    C_vec_right_leg[3] = 1.0/3000.0 ;
    C_vec_right_leg[4] = 1.0/4000.0 ;
    C_vec_right_leg[5] = 1.0/3000.0 ;

    C_right_leg.diagonal( C_vec_right_leg );
    
    //---------------------------------------------------------------------------//
    // LEFT LEG
    
    yarp::sig::Vector C_vec_left_leg(  robot.left_leg.getNumberOfJoints() ) ;
   
    C_vec_left_leg[0] = 1.0/3000.0 ;
    C_vec_left_leg[1] = 1.0/5000.0 ;
    C_vec_left_leg[2] = 1.0/3000.0 ;
    C_vec_left_leg[3] = 1.0/3000.0 ;
    C_vec_left_leg[4] = 1.0/4000.0 ;
    C_vec_left_leg[5] = 1.0/3000.0 ;

    C_left_leg.diagonal( C_vec_left_leg );    
    
    //---------------------------------------------------------------------------//

    q_ref_right_arm =  C_right_arm*tau_current_right_arm +  q_current_right_arm ;
    q_ref_left_arm  =  C_left_arm*tau_current_left_arm   +  q_current_left_arm ;
    q_ref_torso     =  C_torso*tau_current_torso         +  q_current_torso ; //   q_current_torso ; 
    q_ref_right_leg =  C_right_leg*tau_current_right_leg +  q_current_right_leg ;
    q_ref_left_leg  =  C_left_leg*tau_current_left_leg   +  q_current_left_leg ;   
    
    
    
    robot.fromRobotToIdyn( q_ref_right_arm ,
                           q_ref_left_arm  ,
                           q_ref_torso  ,
                           q_ref_right_leg ,
                           q_ref_left_leg  ,
                           q_ref ); 
    //---------------------------------------------------------------------------//
    // Test Printing
    
     std::cout << "C_vec_left_leg[0] = " << C_vec_left_leg[0] << std::endl ;  
     std::cout << "C_vec_left_leg[1] = " << C_vec_left_leg[1] << std::endl ;
     std::cout << "C_vec_left_leg[2] = " << C_vec_left_leg[2] << std::endl ;  
     std::cout << "C_left_leg[0][0] = " << C_left_leg[0][0] << std::endl ; 
     std::cout << "C_left_leg[1][1] = " << C_left_leg[1][1] << std::endl ;  
     std::cout << "C_left_leg[2][2] = " << C_left_leg[2][2] << std::endl ;  

    
    std::cout << "q_ref_torso[0] = " << q_ref_torso[0] << std::endl ;
    std::cout << "q_current_torso[0] = " << q_current_torso[0] << std::endl ;        
    
    double temp_0 =  1000*(q_ref_torso[0] -q_current_torso[0] ) ;
    std::cout << "temp_0 = " << temp_0 << std::endl ;
    std::cout << "tau_current_torso[0] = " << tau_current_torso[0] << std::endl ;                           
                           
    std::cout << "q_ref_torso[1] = " << q_ref_torso[01] << std::endl ;
    std::cout << "q_current_torso[1] = " << q_current_torso[1] << std::endl ;            
    double temp_1 =  1000*(q_ref_torso[1] -q_current_torso[1] ) ;
    std::cout << "temp_1 = " << temp_1 << std::endl ;
    std::cout << "tau_current_torso[1] = " << tau_current_torso[1] << std::endl ;   
    
    std::cout << "q_ref_torso[2] = " << q_ref_torso[2] << std::endl ;
    std::cout << "q_current_torso[2] = " << q_current_torso[2] << std::endl ;      
    double temp_2 =  1000*(q_ref_torso[2] -q_current_torso[2] ) ;
    std::cout << "temp_2 = " << temp_2 << std::endl ;
    std::cout << "tau_current_torso[2] = " << tau_current_torso[2] << std::endl ;  

    //---------------------------------------------------------------------------//

    
    
    
    
    
    
//     robot.left_arm.move(q_ref);
    
    
                           
  //Try to stay in the current configuration
  
     //  yarp::sig::Vector q_ref =  a*tau_current + q_current ;
  
        // Move something
//     yarp::sig::Vector q_ref = q_current ;
//     q_ref[0] += .1 ;

    robot.move(q_ref);
    
    //     std::string cmd = command_interface.getCommand() ;  // USEFUL TO SEND INPUT TO THE MOUDLE

    /*  Code from the generic module
    yarp::sig::Vector q_sensed = robot.left_arm.sensePosition();
    std::cout << "Sense : " << q_sensed.toString() << std::endl;
    
    yarp::sig::Vector q_ref(left_arm_joints, 0.0);
    for( int i = 0; i < left_arm_joints; i++) {
	q_ref[i] = q_sensed[i] + sin( (omega * tick / 5) + phi );
    }
    
    std::cout << "Reference : " << q_ref.toString() << std::endl;
    
    
    tick++;
    */
}

bool locoman_control_thread::custom_pause()
{

}

bool locoman_control_thread::custom_resume()
{

}


