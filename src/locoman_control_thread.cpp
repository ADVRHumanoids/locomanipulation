// #include <yarp/os/all.h>
#include <cstdlib>
#include <yarp/math/Math.h>
//#include <yarp/sig/all.h>
#include "locoman_control_thread.h"
#include "locoman_constants.h"
#include <GYM/yarp_command_interface.hpp>
#include <iCub/iDynTree/yarp_kdl.h>
#include <fstream>

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

yarp::sig::Vector locoman_control_thread::senseMotorPosition()
{
    yarp::sig::Vector q_link = robot.sensePosition() ;
    yarp::sig::Vector tau  = robot.senseTorque() ;
    //     
    yarp::sig::Vector Cq_vec_right_arm(  robot.right_arm.getNumberOfJoints() ) ;
    yarp::sig::Vector Cq_vec_left_arm(   robot.left_arm.getNumberOfJoints() ) ;  
    yarp::sig::Vector Cq_vec_torso(      robot.torso.getNumberOfJoints() ) ;
    yarp::sig::Vector Cq_vec_right_leg(  robot.right_leg.getNumberOfJoints() ) ;
    yarp::sig::Vector Cq_vec_left_leg(   robot.left_leg.getNumberOfJoints() ) ;
    // RIGHT ARM    
    Cq_vec_right_arm[0] = 1.0/1000.0 ;
    Cq_vec_right_arm[1] = 1.0/1000.0 ;
    Cq_vec_right_arm[2] = 1.0/600.0 ;
    Cq_vec_right_arm[3] = 1.0/1000.0 ;
    Cq_vec_right_arm[4] = 1.0/100.0 ;
    Cq_vec_right_arm[5] = 1.0/100.0 ;
    Cq_vec_right_arm[6] = 1.0/10.0 ;
    // LEFT ARM   
    Cq_vec_left_arm[0] = 1.0/1000.0 ;
    Cq_vec_left_arm[1] = 1.0/1000.0 ;
    Cq_vec_left_arm[2] = 1.0/600.0 ;
    Cq_vec_left_arm[3] = 1.0/1000.0 ;
    Cq_vec_left_arm[4] = 1.0/100.0 ;
    Cq_vec_left_arm[5] = 1.0/100.0 ;
    Cq_vec_left_arm[6] = 1.0/10.0 ;
    //TORSO
    Cq_vec_torso[0] = 1.0/1000.0 ;
    Cq_vec_torso[1] = 1.0/1000.0 ;
    Cq_vec_torso[2] = 1.0/1000.0 ;
    // RIGHT LEG
    Cq_vec_right_leg[0] = 1.0/3000.0 ;
    Cq_vec_right_leg[1] = 1.0/5000.0 ;
    Cq_vec_right_leg[2] = 1.0/3000.0 ;
    Cq_vec_right_leg[3] = 1.0/3000.0 ;
    Cq_vec_right_leg[4] = 1.0/4000.0 ;
    Cq_vec_right_leg[5] = 1.0/3000.0 ;
    // LEFT LEG
    Cq_vec_left_leg[0] = 1.0/3000.0 ;
    Cq_vec_left_leg[1] = 1.0/5000.0 ;
    Cq_vec_left_leg[2] = 1.0/3000.0 ;
    Cq_vec_left_leg[3] = 1.0/3000.0 ;
    Cq_vec_left_leg[4] = 1.0/4000.0 ;
    Cq_vec_left_leg[5] = 1.0/3000.0 ;
    //
    yarp::sig::Vector Cq_vec( robot.getNumberOfJoints()  )  ;     
    robot.fromRobotToIdyn( Cq_vec_right_arm ,
                           Cq_vec_left_arm  ,
                           Cq_vec_torso  ,
                           Cq_vec_right_leg ,
                           Cq_vec_left_leg  ,
                           Cq_vec ); 
    yarp::sig::Matrix Cq_matrix(  robot.getNumberOfJoints(), robot.getNumberOfJoints() )  ; 
    Cq_matrix.diagonal(  Cq_vec ) ;  
    yarp::sig::Vector q_motor( robot.getNumberOfJoints()  )  ; 
    q_motor = Cq_matrix*tau  + q_link ;
    return q_motor ;
}

yarp::sig::Matrix locoman_control_thread::getRot(const yarp::sig::Matrix T_ab)
{
    yarp::sig::Matrix R_ab( 3 , 3 ) ;  
    R_ab = T_ab.submatrix(0,2,0,2) ;
    return R_ab ;
}

yarp::sig::Vector locoman_control_thread::getTrasl(const yarp::sig::Matrix T_ab)
{
    yarp::sig::Matrix d_ab_mat( 3, 1 ) ;  
    d_ab_mat = T_ab.submatrix(0,2,3, 3) ;   
    yarp::sig::Vector d_ab( 3  ) ;  
    d_ab[0] = d_ab_mat[0][0] ;
    d_ab[1] = d_ab_mat[1][0] ;
    d_ab[2] = d_ab_mat[2][0] ;
    return d_ab ;
}

yarp::sig::Matrix locoman_control_thread::Homogeneous(const yarp::sig::Matrix R_ab, const yarp::sig::Vector d_ab)
{
    yarp::sig::Matrix T_ab( 4 , 4 ) ;
    yarp::sig::Matrix d_ab_mat( 3 , 1 ) ;

    T_ab.setSubmatrix(R_ab, 0, 0 ) ;
    
    d_ab_mat[0][0] = d_ab[0] ;
    d_ab_mat[1][0] = d_ab[1] ;
    d_ab_mat[2][0] = d_ab[2] ;
    
    T_ab.setSubmatrix(d_ab_mat, 0, 3 ) ;
    return T_ab ; 
}


yarp::sig::Matrix locoman_control_thread::iHomogeneous(const yarp::sig::Matrix T_ab)
{
    yarp::sig::Matrix T_ba( 4 , 4 ) ;  
    yarp::sig::Matrix R_ab( 3 , 3 ) ; 
    yarp::sig::Vector d_ab( 3  ) ;  
    yarp::sig::Matrix R_ba( 3 , 3 ) ; 
    yarp::sig::Vector d_ba( 3  ) ;    
    
    R_ab = getRot(T_ab) ;
    d_ab = getTrasl(T_ab)  ;
    
    R_ba = R_ab.transposed() ; 
    d_ba = -1.0*R_ba*d_ab ; // 
    
    T_ba = Homogeneous(R_ba, d_ba) ;
    //
    return T_ba ; 
}

yarp::sig::Matrix locoman_control_thread::Adjoint(const yarp::sig::Matrix T_ab)
{
 
    yarp::sig::Matrix Adj( 6 , 6 ) ;
    yarp::sig::Matrix R_ab( 3 , 3 ) ; 
    yarp::sig::Vector d_ab( 3  ) ;  
    yarp::sig::Matrix d_ab_skew( 3 , 3 )   ;
    
    R_ab = getRot(T_ab) ;
    d_ab = getTrasl(T_ab)  ;
    d_ab_skew = crossProductMatrix( d_ab ) ; 
    
    Adj.setSubmatrix(R_ab, 0 , 0 ) ;
    Adj.setSubmatrix(R_ab, 3 , 3 ) ;
    Adj.setSubmatrix(d_ab_skew*R_ab, 0 , 3 ) ;
    
    return Adj ;
  
}

yarp::sig::Matrix locoman_control_thread::Adjoint_MT(const yarp::sig::Matrix T_ab)
{
    yarp::sig::Matrix Adj_MT( 6 , 6 ) ;
    yarp::sig::Matrix R_ab( 3 , 3 ) ; 
    yarp::sig::Vector d_ab( 3  ) ;  
    yarp::sig::Matrix d_ab_skew( 3 , 3 )   ;
    
    R_ab = getRot(T_ab) ;
    d_ab = getTrasl(T_ab)  ;
    d_ab_skew = crossProductMatrix( d_ab ) ; 
    
    Adj_MT.setSubmatrix(R_ab, 0 , 0 ) ;
    Adj_MT.setSubmatrix(R_ab, 3 , 3 ) ;
    Adj_MT.setSubmatrix(d_ab_skew*R_ab, 3 , 0 ) ;
    
    return Adj_MT ;
}

yarp::sig::Matrix locoman_control_thread::fConToSens(const int sens_index, const int c1_index, const int c2_index, const int c3_index, const int c4_index)
{
    yarp::sig::Matrix map_fConToSens( 6 , 12 ) ;
    yarp::sig::Matrix B_select( 6 , 3 ) ;
    yarp::sig::Matrix T_w_sensor( 4 , 4 ) ;
    yarp::sig::Matrix T_w_c1( 4 , 4 ) ;
    yarp::sig::Matrix T_w_c2( 4 , 4 ) ;
    yarp::sig::Matrix T_w_c3( 4 , 4 ) ;
    yarp::sig::Matrix T_w_c4( 4 , 4 ) ;
    yarp::sig::Matrix Eye_3( 3 , 3 ) ;
    
    Eye_3.eye() ;

    B_select.setSubmatrix( Eye_3 , 0 , 0 ) ;
    T_w_sensor = model.iDyn3_model.getPosition(sens_index) ;
    T_w_c1  = model.iDyn3_model.getPosition(c1_index)  ;
    T_w_c2  = model.iDyn3_model.getPosition(c2_index) ;    
    T_w_c3  = model.iDyn3_model.getPosition(c3_index)  ;    
    T_w_c4 = model.iDyn3_model.getPosition(c4_index) ;    
    
    yarp::sig::Matrix Ad_1 = Adjoint_MT( iHomogeneous(T_w_sensor)*T_w_c1  ) *B_select  ;
    yarp::sig::Matrix Ad_2 = Adjoint_MT( iHomogeneous(T_w_sensor)*T_w_c2  ) *B_select  ;
    yarp::sig::Matrix Ad_3 = Adjoint_MT( iHomogeneous(T_w_sensor)*T_w_c3  ) *B_select  ;
    yarp::sig::Matrix Ad_4 = Adjoint_MT( iHomogeneous(T_w_sensor)*T_w_c4  ) *B_select  ;

    map_fConToSens.setSubmatrix( Ad_1 , 0 , 0 ) ;
    map_fConToSens.setSubmatrix( Ad_2 , 0 , 3 ) ;
    map_fConToSens.setSubmatrix( Ad_3 , 0 , 6 ) ;
    map_fConToSens.setSubmatrix( Ad_4 , 0 , 9 ) ;

    return map_fConToSens ;
}



void locoman_control_thread::run()
{   
    
    yarp::sig::Vector q_current = robot.sensePosition() ;
    robot.idynutils.updateiDyn3Model( q_current, true ); //update model first
    
    yarp::sig::Vector tau_current = robot.senseTorque() ;

    // Splitting q_
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
    // Splitting tau_
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

    
			    
			    
			    
/*     yarp::sig::Vector q_ref_current_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_ref_current_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_current_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_current_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_current_left_leg(robot.left_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_current( robot.getNumberOfJoints() ) ;
   
    
    
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
     
    //--------------------------------------------------------------------------//
    //TORSO
    
    yarp::sig::Vector C_vec_torso(  robot.torso.getNumberOfJoints() ) ;
   
    C_vec_torso[0] = 1.0/1000.0 ;
    C_vec_torso[1] = 1.0/1000.0 ;
    C_vec_torso[2] = 1.0/1000.0 ;

    //---------------------------------------------------------------------------//
    // RIGHT LEG
    
    yarp::sig::Vector C_vec_right_leg(  robot.right_leg.getNumberOfJoints() ) ;
   
    C_vec_right_leg[0] = 1.0/3000.0 ;
    C_vec_right_leg[1] = 1.0/5000.0 ;
    C_vec_right_leg[2] = 1.0/3000.0 ;
    C_vec_right_leg[3] = 1.0/3000.0 ;
    C_vec_right_leg[4] = 1.0/4000.0 ;
    C_vec_right_leg[5] = 1.0/3000.0 ;
    
    //---------------------------------------------------------------------------//
    // LEFT LEG
    
    yarp::sig::Vector C_vec_left_leg(  robot.left_leg.getNumberOfJoints() ) ;
   
    C_vec_left_leg[0] = 1.0/3000.0 ;
    C_vec_left_leg[1] = 1.0/5000.0 ;
    C_vec_left_leg[2] = 1.0/3000.0 ;
    C_vec_left_leg[3] = 1.0/3000.0 ;
    C_vec_left_leg[4] = 1.0/4000.0 ;
    C_vec_left_leg[5] = 1.0/3000.0 ;  
    
    //---------------------------------------------------------------------------//

    yarp::sig::Vector q_ref_ToMove_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_ref_ToMove_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_left_leg(robot.left_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove(robot.getNumberOfJoints()) ;     
    
    //---------------------------------------------------------------------------//
    // q_ref computation: Method 2
    // Chain-by-chain computation of the _ref variables
    
    yarp::sig::Matrix C_right_arm( robot.right_arm.getNumberOfJoints() ,  robot.right_arm.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_left_arm(  robot.left_arm.getNumberOfJoints()  ,  robot.left_arm.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_torso(     robot.torso.getNumberOfJoints()     ,  robot.torso.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_right_leg( robot.right_leg.getNumberOfJoints() ,  robot.right_leg.getNumberOfJoints() ) ;
    yarp::sig::Matrix C_left_leg(  robot.left_leg.getNumberOfJoints()  ,  robot.left_leg.getNumberOfJoints() ) ;
    
  
    C_right_arm.diagonal( C_vec_right_arm );
    C_left_arm.diagonal( C_vec_left_arm );
    C_torso.diagonal( C_vec_torso );
    C_right_leg.diagonal( C_vec_right_leg );
    C_left_leg.diagonal( C_vec_left_leg ); 
    
    q_ref_current_right_arm =  C_right_arm*tau_current_right_arm +  q_current_right_arm ;
    q_ref_current_left_arm  =  C_left_arm*tau_current_left_arm   +  q_current_left_arm ;
    q_ref_current_torso     =  C_torso*tau_current_torso         +  q_current_torso ; //   q_current_torso ; 
    q_ref_current_right_leg =  C_right_leg*tau_current_right_leg +  q_current_right_leg ;
    q_ref_current_left_leg  =  C_left_leg*tau_current_left_leg   +  q_current_left_leg ;   
    
    robot.fromRobotToIdyn( q_ref_current_right_arm ,
                           q_ref_current_left_arm  ,
                           q_ref_current_torso  ,
                           q_ref_current_right_leg ,
                           q_ref_current_left_leg  ,
                           q_ref_current );   
    //
    q_ref_ToMove = q_ref_current ;
        
    robot.fromIdynToRobot(  q_ref_ToMove,
                            q_ref_ToMove_right_arm,
                            q_ref_ToMove_left_arm,
                            q_ref_ToMove_torso,
                            q_ref_ToMove_right_leg,
                            q_ref_ToMove_left_leg  ) ;  */

    // STOP   
    //--------------------------------------------------------------------------//  
    //---------------------------------------------------------------------------//
    // q_ref computation: Method 2
    // Whole computation of the _ref variables
    //
    
    /* yarp::sig::Vector C_vec( robot.getNumberOfJoints()  )  ; 
    
    robot.fromRobotToIdyn( C_vec_right_arm ,
                           C_vec_left_arm  ,
                           C_vec_torso  ,
                           C_vec_right_leg ,
                           C_vec_left_leg  ,
                           C_vec ); 
    yarp::sig::Matrix C_q(  robot.getNumberOfJoints(), robot.getNumberOfJoints() )  ;
    
    C_q.diagonal(  C_vec ) ;  
   
    q_ref_current = C_q*tau_current + q_current ; 
    
    robot.fromIdynToRobot(  q_ref_current,
                            q_ref_current_right_arm,
                            q_ref_current_left_arm,
                            q_ref_current_torso,
                            q_ref_current_right_leg,
                            q_ref_current_left_leg  ) ;
  
    q_ref_ToMove = q_ref_current ;
        
    robot.fromIdynToRobot(  q_ref_ToMove,
                            q_ref_ToMove_right_arm,
                            q_ref_ToMove_left_arm,
                            q_ref_ToMove_torso,
                            q_ref_ToMove_right_leg,
                            q_ref_ToMove_left_leg  ) ;  */
			    
    // STOP   
    //
    //---------------------------------------------------------------------------//
    // Sensing motor via external function
    
    yarp::sig::Vector q_motor_side(robot.getNumberOfJoints() ) ;		    
    q_motor_side = senseMotorPosition() ;
   
    
    yarp::sig::Matrix T;
    KDL::Frame T_KDL;
    YarptoKDL(T,T_KDL);
			    
    // u defnition
    // virtual kinematic chain (VKC) parameters

    yarp::sig::Vector u_current( 6 )  ;
    
    u_current[0] = 0 ;
    u_current[1] = 0 ;
    u_current[2] = 0 ;
    u_current[3] = 0 ;
    u_current[4] = 0 ;
    u_current[5] = 0 ;

    yarp::sig::Vector u_ref( 6 )  ;  // no spring at the joints of the VKC
    u_ref = u_current ;   
    
    
    //--------------------------------------------------------------------------//
    // Getting Contact Forces
    RobotUtils::ftPtrMap fts = robot.getftSensors();
    
   //Cheking the existence of the sensors
    //assert(fts.size() > 0 && "no ft found!");
    
    //Printing Sensor Names
    RobotUtils::ftPtrMap::iterator i = fts.begin() ;    
    //std::cout << "qui" << std::endl;
    //std::cout << i->second->getReferenceFrame() << std::endl;

  /*  for(RobotUtils::ftPtrMap::iterator i = fts.begin() ;  i != fts.end(); i++)
    {
      std::cout << i->first << std::endl;
// Alternative formulations for the iterators      
// std::cout << i->second->getReferenceFrame() << std::endl;     
//      (*i).second->getReferenceFrame();
//      fts[i->first]->getReferenceFrame(); 
    }*/
  
  //--------------------------------------------------------------------//
    //Getting Sensor Measures

    yarp::sig::Vector ft_r_ankle(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    
    yarp::sig::Vector ft_l_ankle(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    
   // std::cout << "start r_ankle" << std::endl;
    for(int i=0; i< ft_r_ankle.size() ; i++)
    {
    //  std::cout<< ft_l_ankle.size()<< std::endl ;
     std::cout <<  ft_r_ankle[i] << std::endl ;
    }
   // std::cout << "end r_ankle" << std::endl ;
    
  //  std::cout << "start l_ankle" << std::endl;
    for(int i=0; i< ft_l_ankle.size() ; i++)
    {
    //  std::cout<< ft_l_ankle.size()<< std::endl ;
     std::cout <<  ft_l_ankle[i] << std::endl ;
    }
  //  std::cout << "end l_ankle" << std::endl ;
    
    //
  
    // writing data on a file
    //std::ofstream r_ankle ;
    //r_ankle.open ("r_ankle.txt");
    std::ofstream r_ankle_cl ( "r_ankle.m", std::ios::app );
    if( r_ankle_cl.is_open() )
    r_ankle_cl <<  ft_r_ankle[2] << std::endl;
    //r_ankle.close();
    
    std::ofstream l_ankle_cl ( "l_ankle.m", std::ios::app );
    if( l_ankle_cl.is_open() )
    l_ankle_cl <<  ft_l_ankle[2] << std::endl;
    
    //--------------------------------------------------------------------//  
    
    //
    //q_ref_ToMove_left_arm = left_arm_configuration; // left_arm_configuration [rad]
   
    //-------------------------------------------------------------------------//
    // Getting Jacobians of the left foot
        
    int l_ankle_index = model.iDyn3_model.getLinkIndex("l_ankle") ; // sensors are placed in _ankle in the model
    int l_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link");
    int l_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");
    int l_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link");
    int l_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
    
    yarp::sig::Matrix Jac_l_ankle_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_right_link_body( 6  , ( robot.getNumberOfJoints() + 6)  ) ; 
    yarp::sig::Matrix Jac_l_foot_upper_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_upper_right_link_body( 6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    
    model.iDyn3_model.getJacobian( l_ankle_index, Jac_l_ankle_body, true ) ; //false= mixed version jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_left_link_index  , Jac_l_foot_lower_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_right_link_index , Jac_l_foot_lower_right_link_body , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_upper_left_link_index  , Jac_l_foot_upper_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_upper_right_link_index , Jac_l_foot_upper_right_link_body , true  ) ; //true= body jacobian

    // Getting Jacobians of the right foot
    
    int r_ankle_index = model.iDyn3_model.getLinkIndex("r_ankle") ;
    int r_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
    int r_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");
    int r_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
    int r_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
    
    yarp::sig::Matrix Jac_r_ankle_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_right_link_body( 6  , ( robot.getNumberOfJoints() + 6)  ) ; 
    yarp::sig::Matrix Jac_r_foot_upper_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_upper_right_link_body( 6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    
    model.iDyn3_model.getJacobian( r_ankle_index, Jac_l_ankle_body, true ) ; //false= mixed version jacobian
    model.iDyn3_model.getJacobian( r_foot_lower_left_link_index  , Jac_r_foot_lower_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_lower_right_link_index , Jac_r_foot_lower_right_link_body , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_upper_left_link_index  , Jac_r_foot_upper_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_upper_right_link_index , Jac_r_foot_upper_right_link_body , true  ) ; //true= body jacobian

//----------------------------------------------------------------------------------------//
// Computing contact forces at each point - Left Foot
    
    yarp::sig::Matrix T_w_l_ankle(  4 ,   4 ) ;
    yarp::sig::Matrix T_w_l_foot_lower_left_link(   4 ,   4 ) ;    
    yarp::sig::Matrix T_w_l_foot_lower_right_link(  4 ,   4 ) ;
    yarp::sig::Matrix T_w_l_foot_upper_left_link(   4 ,   4 ) ;	
    yarp::sig::Matrix T_w_l_foot_upper_right_link(  4 ,   4 ) ;	
    
    yarp::sig::Matrix T_l_l1(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l2(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l3(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l4(  4 ,   4 ) ;

    T_w_l_ankle = model.iDyn3_model.getPosition(l_ankle_index) ;
    T_w_l_foot_lower_left_link  = model.iDyn3_model.getPosition(l_foot_lower_left_link_index)  ;
    T_w_l_foot_lower_right_link = model.iDyn3_model.getPosition(l_foot_lower_right_link_index) ;    
    T_w_l_foot_upper_left_link  = model.iDyn3_model.getPosition(l_foot_upper_left_link_index)  ;    
    T_w_l_foot_upper_right_link = model.iDyn3_model.getPosition(l_foot_upper_right_link_index) ;    

    T_l_l1 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_upper_left_link  ;
    T_l_l2 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_upper_right_link ; 
    T_l_l3 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_lower_left_link  ;
    T_l_l4 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_lower_right_link ;
    
    std::cout << " T_l_l1 = " << T_l_l1.toString() << std::endl ;
    std::cout << " T_l_l2 = " << T_l_l2.toString() << std::endl ;
    std::cout << " T_l_l3 = " << T_l_l3.toString() << std::endl ;
    std::cout << " T_l_l4 = " << T_l_l4.toString() << std::endl ;
    
    
//----------------------------------------------------------------------------------------//
// Computing contact forces at each point - Right Foot    
    
    yarp::sig::Matrix T_w_r_ankle(  4 ,   4 ) ;	
    yarp::sig::Matrix T_w_r_foot_lower_left_link(   4 ,   4 ) ;    
    yarp::sig::Matrix T_w_r_foot_lower_right_link(  4 ,   4 ) ;
    yarp::sig::Matrix T_w_r_foot_upper_left_link(   4 ,   4 ) ;	
    yarp::sig::Matrix T_w_r_foot_upper_right_link(  4 ,   4 ) ;	
	
    yarp::sig::Matrix T_r_r1(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r2(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r3(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r4(  4 ,   4 ) ;

    T_w_r_ankle = model.iDyn3_model.getPosition(r_ankle_index) ;
    T_w_r_foot_lower_left_link  = model.iDyn3_model.getPosition(r_foot_lower_left_link_index)  ;
    T_w_r_foot_lower_right_link = model.iDyn3_model.getPosition(r_foot_lower_right_link_index) ;    
    T_w_r_foot_upper_left_link  = model.iDyn3_model.getPosition(r_foot_upper_left_link_index)  ;    
    T_w_r_foot_upper_right_link  = model.iDyn3_model.getPosition(r_foot_upper_right_link_index) ;    

    T_r_r1 = yarp::math::luinv(T_w_r_ankle)*T_w_r_foot_upper_left_link  ;
    T_r_r2 = yarp::math::luinv(T_w_r_ankle)*T_w_r_foot_upper_right_link ; 
    T_r_r3 = yarp::math::luinv(T_w_r_ankle)*T_w_r_foot_lower_left_link  ;
    T_r_r4 = yarp::math::luinv(T_w_r_ankle)*T_w_r_foot_lower_right_link ;
    
    std::cout << " T_r_r1 = " << T_r_r1.toString() << std::endl ;
    std::cout << " T_r_r2 = " << T_r_r2.toString() << std::endl ;
    std::cout << " T_r_r3 = " << T_r_r3.toString() << std::endl ;
    std::cout << " T_r_r4 = " << T_r_r4.toString() << std::endl ;
    std::cout << "  ------------ "  << std::endl ;     
    
    yarp::sig::Matrix Ad_g_rr1(  6 ,   6 ) ;
    yarp::sig::Matrix Ad_g_MT_rr1(  6 ,   6 ) ;
    
    Ad_g_rr1 = Adjoint(T_r_r1) ;
    Ad_g_MT_rr1 = Adjoint_MT(T_r_r1) ;
    std::cout << " Ad_g_rr1 = "    << Ad_g_rr1.toString() << std::endl ;
    std::cout << " Ad_g_MT_rr1 = " << Ad_g_MT_rr1.toString() << std::endl ;
    
    
     yarp::sig::Matrix map_l_fcToSens =   fConToSens( l_ankle_index, 
						      l_foot_lower_left_link_index, 
						      l_foot_lower_right_link_index, 
						      l_foot_upper_left_link_index, 
					              l_foot_upper_right_link_index) ;
     yarp::sig::Matrix map_r_fcToSens =   fConToSens( r_ankle_index, 
				                      r_foot_lower_left_link_index, 
						      r_foot_lower_right_link_index, 
					              r_foot_upper_left_link_index, 
						      r_foot_upper_right_link_index) ;	

     std::cout << "map_l_fcToSens = " << map_l_fcToSens.toString() << std::endl ; 
    
     // Jacobian tests
     yarp::sig::Matrix Jac_l_ankle_spa = Adjoint( iHomogeneous( T_w_l_ankle )   )*Jac_l_ankle_body  ;
     
    yarp::sig::Matrix Jac_l_ankle_mix_1(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    
    model.iDyn3_model.getJacobian( l_ankle_index, Jac_l_ankle_mix_1, false ) ;      
     
    
    yarp::sig::Matrix Jac_l_ankle_mix_2(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_ankle_mix_3(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 

    yarp::sig::Matrix Eye_3( 3 , 3 ) ;
    Eye_3.eye() ;
    
    
    Jac_l_ankle_mix_2 = Adjoint(Homogeneous(Eye_3, getTrasl(T_w_l_ankle)))*Jac_l_ankle_spa; //Transformed from the spatial form
    Jac_l_ankle_mix_3 = Adjoint( iHomogeneous(Homogeneous(Eye_3, getTrasl(T_w_l_ankle)) ) )*Jac_l_ankle_body ; //Transformed from the body form;
    
    //Verify equalityies.........................................
    
    
    
   /* yarp::sig::Matrix R_r_r1(  3 ,   3 ) ;
    yarp::sig::Vector d_r_r1(  3  ) ;

    R_r_r1 = getRot(T_r_r1) ;
    d_r_r1 = getTrasl(T_r_r1) ;  */
  
    /* std::cout << " R_r_r1 = " << R_r_r1.toString() << std::endl ;
    std::cout << " d_r_r1 = " << d_r_r1.toString() << std::endl ;
    std::cout << "  ------------ "  << std::endl ;     */

    /*yarp::sig::Matrix T_r1_r_temp_1(  4 ,   4 ) ;
    yarp::sig::Matrix T_r1_r_temp_2(  4 ,   4 ) ;  */

    /*T_r1_r_temp_1 = iHomogeneous(T_r_r1) ;
    T_r1_r_temp_2 = Homogeneous(R_r_r1, d_r_r1 )     ;  */
    
   /* std::cout << " T_r1_r_temp_1 = " << T_r1_r_temp_1.toString() << std::endl ;
    std::cout << " T_r1_r_temp_2 = " << T_r1_r_temp_2.toString() << std::endl ;
    std::cout << "  ------------ "  << std::endl ;    */
   
    
   
   
   
   
    
 // model.iDyn3_model.getPosition()  ; //Getting Configuration in world Frame
   
  
  
//----------------------------------------------------------------------------------------//
// Computing contact forces at each point - Right Foot
  
    
    
    
    
    //---------------------------------------------------------------------------//
    //---------------------------------------------------------------------------//
			    
  
  
    yarp::sig::Vector q_ref_ToMove_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_ref_ToMove_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove_left_leg(robot.left_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_ref_ToMove(robot.getNumberOfJoints()) ; 
  
    q_ref_ToMove = q_motor_side ;
    
    robot.fromIdynToRobot(  q_ref_ToMove,
                            q_ref_ToMove_right_arm,
                            q_ref_ToMove_left_arm,
                            q_ref_ToMove_torso,
                            q_ref_ToMove_right_leg,
                            q_ref_ToMove_left_leg  ) ; 
  
    
			    
    // Move something
    q_ref_ToMove_right_arm[0] += -.00 ;  
    
    robot.fromRobotToIdyn( q_ref_ToMove_right_arm ,
                           q_ref_ToMove_left_arm  ,
                           q_ref_ToMove_torso  ,
                           q_ref_ToMove_right_leg ,
                           q_ref_ToMove_left_leg  ,
                           q_ref_ToMove );    

     robot.move(q_ref_ToMove);  // q_ref_ToMove
   // robot.left_arm.move(q_ref_ToMove_left_arm);
    
    
    
    
    
    //---------------------------------------------------------------------------------------------------//
    // Test Printing
      
    /*std::cout << " C_q[0][0]  = " <<     C_q[0][0]  << std::endl ; 
    std::cout << " C_q[0][1]  = " <<     C_q[0][1]  << std::endl ;
    std::cout << " C_q[1][1]  = " <<     C_q[1][1]  << std::endl ; 
    std::cout << " C_vec[0]  = " <<     C_vec[0]  << std::endl ; 
    std::cout << " C_vec[1]  = " <<     C_vec[1]  << std::endl ; */

    
    std::cout << "q_ref : " << q_motor_side.toString() << std::endl;
    
  /*   std::cout << "q_ref_ToMove[0] = " << q_ref_ToMove[0] << std::endl ;  
     std::cout << "q_ref_ToMove[1] = " << q_ref_ToMove[1] << std::endl ;  
     std::cout << "q_ref_ToMove[2] = " << q_ref_ToMove[2] << std::endl ;  
     std::cout << "q_ref_ToMove[3] = " << q_ref_ToMove[3] << std::endl ;  
     std::cout << "q_ref_ToMove[4] = " << q_ref_ToMove[4] << std::endl ;  
     std::cout << "q_ref_ToMove[5] = " << q_ref_ToMove[5] << std::endl ;  
     std::cout << "q_ref_ToMove[6] = " << q_ref_ToMove[6] << std::endl ;  
     std::cout << "q_ref_ToMove[7] = " << q_ref_ToMove[7] << std::endl ;  
     std::cout << "q_ref_ToMove[8] = " << q_ref_ToMove[8] << std::endl ;  
     std::cout << "q_ref_ToMove[9] = " << q_ref_ToMove[9] << std::endl ;  
     std::cout << "q_ref_ToMove[10] = " << q_ref_ToMove[10] << std::endl ;  
     std::cout << "q_ref_ToMove[11] = " << q_ref_ToMove[11] << std::endl ;  
     std::cout << "q_ref_ToMove[12] = " << q_ref_ToMove[12] << std::endl ;   */
    /* std::cout << "q_ref_ToMove[13] = " << q_ref_ToMove[13] << std::endl ; 
     std::cout << "q_ref_ToMove[14] = " << q_ref_ToMove[14] << std::endl ;  
     std::cout << "q_ref_ToMove[15] = " << q_ref_ToMove[15] << std::endl ;  
     std::cout << "q_ref_ToMove[16] = " << q_ref_ToMove[16] << std::endl ;  
     std::cout << "q_ref_ToMove[17] = " << q_ref_ToMove[17] << std::endl ; 
     std::cout << "q_ref_ToMove[18] = " << q_ref_ToMove[18] << std::endl ;  
     std::cout << "q_ref_ToMove[19] = " << q_ref_ToMove[19] << std::endl ;  
     std::cout << "q_ref_ToMove[20] = " << q_ref_ToMove[20] << std::endl ;  
     std::cout << "q_ref_ToMove[21] = " << q_ref_ToMove[21] << std::endl ; 
     std::cout << "q_ref_ToMove[22] = " << q_ref_ToMove[22] << std::endl ; 
     std::cout << "q_ref_ToMove[23] = " << q_ref_ToMove[23] << std::endl ;  
     std::cout << "q_ref_ToMove[24] = " << q_ref_ToMove[24] << std::endl ;  
     std::cout << "q_ref_ToMove[25] = " << q_ref_ToMove[25] << std::endl ;  
     std::cout << "q_ref_ToMove[26] = " << q_ref_ToMove[26] << std::endl ; 
     std::cout << "q_ref_ToMove[27] = " << q_ref_ToMove[27] << std::endl ;  
     std::cout << "q_ref_ToMove[28] = " << q_ref_ToMove[28] << std::endl ;  
     //
     std::cout << "q_ref_ToMove_torso[0] = " << q_ref_ToMove_torso[0] << std::endl ;  
     std::cout << "q_ref_ToMove_torso[1] = " << q_ref_ToMove_torso[1] << std::endl ;  
     std::cout << "q_ref_ToMove_torso[2] = " << q_ref_ToMove_torso[2] << std::endl ;  
     //
     std::cout << "q_ref_ToMove_right_arm[0] = " << q_ref_ToMove_right_arm[0] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[1] = " << q_ref_ToMove_right_arm[1] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[2] = " << q_ref_ToMove_right_arm[2] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[3] = " << q_ref_ToMove_right_arm[3] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[4] = " << q_ref_ToMove_right_arm[4] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[5] = " << q_ref_ToMove_right_arm[5] << std::endl ;  
     std::cout << "q_ref_ToMove_right_arm[6] = " << q_ref_ToMove_right_arm[6] << std::endl ;  
     //
     std::cout << "q_ref_ToMove_left_arm[0] = " << q_ref_ToMove_left_arm[0] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[1] = " << q_ref_ToMove_left_arm[1] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[2] = " << q_ref_ToMove_left_arm[2] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[3] = " << q_ref_ToMove_left_arm[3] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[4] = " << q_ref_ToMove_left_arm[4] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[5] = " << q_ref_ToMove_left_arm[5] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[6] = " << q_ref_ToMove_left_arm[6] << std::endl ;  
     std::cout << "q_ref_ToMove_left_arm[7] = " << q_ref_ToMove_left_arm[7] << std::endl ;  */
     //
  /*   std::cout << "q_ref_ToMove_right_leg[0] = " << q_ref_ToMove_right_leg[0] << std::endl ;  
     std::cout << "q_ref_ToMove_right_leg[1] = " << q_ref_ToMove_right_leg[1] << std::endl ;  
     std::cout << "q_ref_ToMove_right_leg[2] = " << q_ref_ToMove_right_leg[2] << std::endl ;  
     std::cout << "q_ref_ToMove_right_leg[3] = " << q_ref_ToMove_right_leg[3] << std::endl ;  
     std::cout << "q_ref_ToMove_right_leg[4] = " << q_ref_ToMove_right_leg[4] << std::endl ;  
     std::cout << "q_ref_ToMove_right_leg[5] = " << q_ref_ToMove_right_leg[5] << std::endl ;  
    // std::cout << "q_ref_ToMove_right_leg[6] = " << q_ref_ToMove_right_leg[6] << std::endl ;  
     //
     std::cout << "q_ref_ToMove_left_leg[0] = " << q_ref_ToMove_left_leg[0] << std::endl ;  
     std::cout << "q_ref_ToMove_left_leg[1] = " << q_ref_ToMove_left_leg[1] << std::endl ;  
     std::cout << "q_ref_ToMove_left_leg[2] = " << q_ref_ToMove_left_leg[2] << std::endl ;  
     std::cout << "q_ref_ToMove_left_leg[3] = " << q_ref_ToMove_left_leg[3] << std::endl ;  
     std::cout << "q_ref_ToMove_left_leg[4] = " << q_ref_ToMove_left_leg[4] << std::endl ;  
     std::cout << "q_ref_ToMove_left_leg[5] = " << q_ref_ToMove_left_leg[5] << std::endl ;  
    // std::cout << "q_ref_ToMove_left_leg[6] = " << q_ref_ToMove_left_leg[6] << std::endl ;       */
    
    
  
  
  
  
  
  
/*     std::cout << "C_vec_left_leg[0] = " << C_vec_left_leg[0] << std::endl ;  
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
    
    std::cout << "q_ref[0] = " << q_ref[0] << std::endl ;
*/
    //---------------------------------------------------------------------------//
    
    //---------------------------------------------------------------------------// 
    // Default Stuff form the Generic Module
    //
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


