// #include <yarp/os/all.h>
#include <cstdlib>
#include <yarp/math/Math.h>
//#include <yarp/sig/all.h>
#include "locoman_control_thread.h"
#include "locoman_constants.h"
#include <GYM/yarp_command_interface.hpp>
#include <iCub/iDynTree/yarp_kdl.h>
#include <yarp/math/SVD.h>
#include <fstream>
#include <unistd.h>
#include <limits>

#include <locoman/utils/screws.hpp>
#include <locoman/utils/kinematics.hpp>
#include <locoman/utils/kinetostatics.hpp>
#include <locoman/utils/locoman_utils.hpp>
#include <locoman/utils/algebra.hpp>


using namespace yarp::math;

locoman_control_thread::locoman_control_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph ):
    control_thread( module_prefix, rf, ph ),  
    command_interface(module_prefix), 
    loop_counter(0) ,
    CoM_w_cmd(3, 0.0) ,
    CoM_w_up(3, 0.0) ,
    CoM_w_dw(3, 0.0) ,
    FC_size(24)  ,
    FC_HANDS_size(24) ,
    WINDOW_size(15) , //30 //50  // 15
    FC_DES(FC_size, 0.0) , 
    FC_DES_LEFT_sensor(6, 0.0) ,
    FC_DES_RIGHT_sensor(6,0.0),
    FC_FILTERED(FC_size),
    FC_WINDOW(FC_size, WINDOW_size ) ,
    //
    FC_HANDS_DES(FC_HANDS_size, 0.0) ,
    FC_DES_LEFT_HAND_sensor(6, 0.0) ,
    FC_DES_RIGHT_HAND_sensor(6, 0.0) ,
    FC_HANDS_FILTERED(FC_HANDS_size) ,
    FC_HANDS_WINDOW(FC_HANDS_size, WINDOW_size ) 
{
    //------------ 
    CoM_waist_cmd = locoman::utils::getRot( locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist"))))*
                    model.iDyn3_model.getCOM() ;
    T_waist_l1_foot_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link"))) ; 
    T_waist_r1_foot_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ; 
    T_waist_l_hand_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))*
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("LSoftHand")))  ;    
    T_waist_r_hand_cmd = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist"))) *
                    model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("RSoftHand")))  ;      
    R_waist_aw_cmd = locoman::utils::getRot( locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( model.iDyn3_model.getLinkIndex("Waist")))) ; //just initialization
    //------------ 
    CoM_w_cmd  = model.iDyn3_model.getCOM()  ;
    CoM_w_up = CoM_w_cmd ;
    CoM_w_dw = CoM_w_cmd ;
    T_l1_r1_up = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link")) ) ;  
    T_l1_r1_fw = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;       
    T_l1_r1_dw = locoman::utils::iHomogeneous(model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;       
 
    T_w_l1_cmd = model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link"))) ;     
    T_w_r1_cmd = model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ;    
     
    T_r1_l1_up = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;
    T_r1_l1_fw = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;
    T_r1_l1_dw = locoman::utils::iHomogeneous( model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("r_foot_upper_left_link"))) ) *
                     model.iDyn3_model.getPosition( (model.iDyn3_model.getLinkIndex("l_foot_upper_left_link")))   ;     
    omega = 0.1;
    phi = 10;
    tick = 0;
    max_vel= 0.1; //  0.35;  // 
    if(robot.idynutils.getRobotName() == "bigman") {
            mg = 1200;
    }
    if(robot.idynutils.getRobotName() == "coman") {
            mg = 290;
    }
    left_arm_joints = LEFT_ARM_JOINT_NUM;
    left_arm_configuration.resize(left_arm_joints);
    //
    right_arm_joints = RIGHT_ARM_JOINT_NUM;
    right_arm_configuration.resize(right_arm_joints);
    torso_joints = TORSO_JOINT_NUM;
    torso_configuration.resize(torso_joints);
    left_leg_joints = LEFT_LEG_JOINT_NUM;
    left_leg_configuration.resize(left_leg_joints);
    right_leg_joints = RIGHT_LEG_JOINT_NUM;
    right_leg_configuration.resize(right_leg_joints);
    //
    //  Simulator-To-Robot Switch
    flag_robot = 1 ;
    flag_simulator = 1-flag_robot ;
    //
    //-----------------------------------------------
    //
    double part = -0.0/10.0 ;  // - => moving on the right; + => moving on the left
    // On the left foot
    FC_DES[2] = - (mg/8.0 + part*(mg/8.0) )   ;
    FC_DES[5] = - (mg/8.0 + part*(mg/8.0) )   ;
    FC_DES[8] = - (mg/8.0 + part*(mg/8.0) )   ;
    FC_DES[11] = - (mg/8.0 + part*(mg/8.0) )  ;
    // On the right foot
    FC_DES[14] = - (mg/8.0 - part*(mg/8.0) )   ;
    FC_DES[17] = - (mg/8.0 - part*(mg/8.0) )   ;
    FC_DES[20] = - (mg/8.0 - part*(mg/8.0) )   ;
    FC_DES[23] = - (mg/8.0 - part*(mg/8.0) )   ; 
    //     
    FC_FILTERED = FC_DES ;
    //                       FC_WINDOW(FC_size, WINDOW_size ) ;
    for(int t=0; t<WINDOW_size ; t++ )
    {
      FC_WINDOW.setCol(t, FC_DES )   ;
    }
    FC_SUM = WINDOW_size * FC_DES ;
    std::cout << " FC_WINDOW  =  "  << std::endl << FC_WINDOW.toString() << std::endl  ; 
    std::cout << " FC_DES  =  "  << std::endl << FC_DES.toString() << std::endl  ;     
   // std::cout << " FC_DES_LEFT_sensor  =  "  << std::endl << FC_DES_LEFT_sensor.toString() << std::endl  ; 
   // std::cout << " FC_DES_RIGHT_sensor  =  "  << std::endl << FC_DES_RIGHT_sensor.toString() << std::endl  ;     
    std::cout << " FC_SUM  =  "  << std::endl << FC_SUM.toString() << std::endl  ; 
    //
    // Hands
    FC_HANDS_FILTERED = FC_HANDS_DES ;
    for(int t=0; t<WINDOW_size ; t++ )
    {
      FC_HANDS_WINDOW.setCol(t, FC_HANDS_DES )   ;
    }
    FC_HANDS_SUM = WINDOW_size * FC_HANDS_DES ;
    std::cout << " FC_HANDS_WINDOW  =  "  << std::endl << FC_HANDS_WINDOW.toString() << std::endl  ; 
    std::cout << " FC_HANDS_DES  =  "  << std::endl << FC_HANDS_DES.toString() << std::endl  ;     
   // std::cout << " FC_DES_LEFT_HAND_sensor  =  "  << std::endl << FC_DES_LEFT_HAND_sensor.toString() << std::endl  ; 
   // std::cout << " FC_DES_RIGHT_HAND_sensor  =  "  << std::endl << FC_DES_RIGHT_HAND_sensor.toString() << std::endl  ;     
    std::cout << " FC_HANDS_SUM  =  "  << std::endl << FC_HANDS_SUM.toString() << std::endl  ; 
    if(robot.idynutils.getRobotName() == "bigman") {
    mg = 1200;
    }
}




void locoman_control_thread::link_locoman_params()
{
    // get a shared pointer to param helper
    std::shared_ptr<paramHelp::ParamHelperServer> ph = get_param_helper();
    
    // link the left_leg configuration (vector linking)
    ph->linkParam( PARAM_ID_LEFT_LEG, left_leg_configuration.data() );
    
    // link the right_leg configuration (vector linking)
    ph->linkParam( PARAM_ID_RIGHT_LEG, right_leg_configuration.data() );
    
    // link the torso configuration (vector linking)
    ph->linkParam( PARAM_ID_TORSO, torso_configuration.data() );
    
    // link the left_arm configuration (vector linking)
    ph->linkParam( PARAM_ID_LEFT_ARM, left_arm_configuration.data() );
    
    // link the right_arm configuration (vector linking)
    ph->linkParam( PARAM_ID_RIGHT_ARM, right_arm_configuration.data() );
    
    // link the max_vel parameter (single value linking
    ph->linkParam( PARAM_ID_MAX_VEL, &max_vel );
}


bool locoman_control_thread::custom_init()
{   
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    link_locoman_params();
    
    model.setFloatingBaseLink("Waist");
    yarp::sig::Vector q_current(robot.getNumberOfKinematicJoints(),0.0) ; // = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
   robot.setPositionDirectMode();    
    
   // TODO : adding integral part in the low level control
   
   
   //------------------------------------------------------------
   int dim_offeset = 1000    ;
   
   yarp::sig::Matrix offset_window(robot.getNumberOfKinematicJoints(), dim_offeset);
   for(int k=0; k<dim_offeset ; k++ ) 
   {
       q_current += locoman::utils::sense_position_no_hands(robot); //if sense returns motorPosition       
       usleep(1*1000) ;  
    }
   q_current = q_current/dim_offeset ;
 
   yarp::sig::Vector q_motor_init(robot.getNumberOfKinematicJoints() ) ;                
   robot.fromRobotToIdyn29(  right_arm_configuration ,
                           left_arm_configuration  ,
                           torso_configuration     ,
                           right_leg_configuration ,
                           left_leg_configuration  ,
                           q_motor_init            )  ;
   q_offset = q_motor_init - q_current ;
   q_current_open_loop = q_current + q_offset;

//    yarp::sig::Vector q_offset_r_arm(right_arm_configuration.length()) ;
//    yarp::sig::Vector q_offset_l_arm(right_arm_configuration.length()) ;
//    yarp::sig::Vector q_offset_torso(torso_configuration.length()) ;
//    yarp::sig::Vector q_offset_r_leg(right_leg_configuration.length()) ;
//    yarp::sig::Vector q_offset_l_leg(left_leg_configuration.length()) ;
//    
//    robot.fromIdynToRobot(  q_offset,
//                            q_offset_r_arm,
//                            q_offset_l_arm,
//                            q_offset_torso,
//                            q_offset_r_leg,
//                            q_offset_l_leg  ) ; 
//    
   std::cout << " q_offset = "  <<  std::endl<<  q_offset.toString() << std::endl; 
   std::cout << " q_motor_init = "  <<  std::endl<<  q_motor_init.toString() << std::endl; 
//    std::cout << " q_offset_r_arm = "  <<  std::endl << q_offset_r_arm.toString() << std::endl;
//    std::cout << " q_offset_l_arm = "  <<  std::endl << q_offset_l_arm.toString() << std::endl;
//    std::cout << " q_offset_torso = "  <<  std::endl << q_offset_torso.toString() << std::endl;
//    std::cout << " q_offset_r_leg = "  <<  std::endl << q_offset_r_leg.toString() << std::endl;
//    std::cout << " q_offset_l_leg = "  <<  std::endl << q_offset_l_leg.toString() << std::endl;   
   //------------------------------------------------------------
   
  //  robot.setPositionMode() ;
//     robot.setReferenceSpeed(0.3) ;
    //-----------------------------------------------------------

    
    //---------------------------------------------------------------------------//
    /*    Old _Init (now replaced with the homing)
    yarp::sig::Vector q_motor_0(robot.getNumberOfJoints() ) ;		    
    q_motor_0 = locoman::utils::senseMotorPosition(robot, flag_robot) ; // this uses manually imposed joint stiffness values

    yarp::sig::Vector q_des(robot.getNumberOfJoints() ) ;		    
    yarp::sig::Vector q_right_arm_des(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_left_arm_des(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_torso_des(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_right_leg_des(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_left_leg_des(robot.left_leg.getNumberOfJoints()) ;    
    
   // q_des = q_motor_0; 
    
    robot.fromIdynToRobot(  q_des,
                            q_right_arm_des,
                            q_left_arm_des,
                            q_torso_des,
                            q_right_leg_des,
                            q_left_leg_des  ) ; 
    
    q_right_arm_des = right_arm_configuration  ; //Insertion of  predefined values (from .ini)			    
    q_left_arm_des  = left_arm_configuration   ;
    q_torso_des     = torso_configuration      ;
    q_left_leg_des  = left_leg_configuration   ;
    q_right_leg_des = right_leg_configuration  ;
    
    robot.fromRobotToIdyn( q_right_arm_des ,
                           q_left_arm_des  ,
                           q_torso_des     ,
                           q_right_leg_des ,
                           q_left_leg_des  ,
                           q_des            );     
     
   yarp::sig::Vector q_motor_act(robot.getNumberOfJoints() ) ;
   yarp::sig::Vector q_motor_act_1(robot.getNumberOfJoints() ) ;
   yarp::sig::Vector d_q_des(robot.getNumberOfJoints()) ;     
    
    d_q_des = (q_des - q_motor_0) ; // *10  ;  // ?????? *10 ??????? /10  /100
    
    //robot.move(q_des) ; 
    
    q_motor_act = locoman::utils::senseMotorPosition(robot, flag_robot) ;
    usleep(100*1000) ;  
    q_motor_act = locoman::utils::senseMotorPosition(robot, flag_robot) ;
    double err_0 = norm(q_motor_act- q_des) +0.1 ;
    double err_1 = norm(q_motor_act- q_des)  ;
    int cont = 1 ;
    int frac = 1000 ; //1000 ;
    while ( norm(q_motor_act- q_des)>0.01 && fabs(err_0-err_1)>0.000001 ) //( ( (norm(q_motor_act- q_des)>0.01)  )) // &&  ( abs(err_0-err_1)>0.000001 )) )
    {
    err_0 = err_1 ;
    q_motor_act = locoman::utils::senseMotorPosition(robot,flag_robot) ;
    
    robot.move((q_motor_0 + cont*d_q_des/frac)) ;  //q_motor_0
    usleep(10*1000) ;  //  usleep(10*1000) ;
    q_motor_act_1 = locoman::utils::senseMotorPosition(robot,flag_robot) ;
    int cont_2 = cont ;
    std::cout << " cont_2  =  " <<  cont_2 << std::endl; 
    while(norm(q_motor_act_1-q_motor_act)<0.1){
    std::cout << " cont_2  =  " <<  cont_2 << std::endl; 
    robot.move((q_motor_0 + cont_2*d_q_des/frac)) ;  //q_motor_0
    q_motor_act_1 = locoman::utils::senseMotorPosition(robot,flag_robot) ;
    cont_2++;  
    }

    cont++ ;
    std::cout << " cont  =  " <<  cont << std::endl; 

    if(cont>frac){cont =frac;}
    std::cout << " err_1  =  " <<  err_1 << std::endl; 
     continue ; 
         q_motor_act = locoman::utils::senseMotorPosition(robot,flag_robot) ;
    err_1 = norm(q_motor_act- q_des)  ;
    }
    std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl; 
    std::cout << " final fabs(err_0-err_1) =  " <<  fabs(err_0-err_1) << std::endl; */
    
    usleep(2000*1000) ; // usleep(milliseconds*1000)
    // robot.left_arm.move(q_ref_ToMove_left_arm);
    return true;
    //
    //----------------------------------------------
    //

}


void locoman_control_thread::run()
{     
  //------------------------------------------------------------------------------------------------------------------
  // Defining Various Parameters
  yarp::sig::Vector zero_3(3, 0.0) ;
  yarp::sig::Matrix Zeros_6_6(6,6) ;
  Zeros_6_6.zero();
  yarp::sig::Matrix Eye_6(6,6) ;
  Eye_6.eye() ;
  yarp::sig::Matrix Eye_3(3,3) ;
  Eye_3.eye() ;
  yarp::sig::Matrix B( 6 , 3 ) ;
  B.setSubmatrix( Eye_3 , 0 , 0 ) ;
   
  int size_q = robot.getNumberOfKinematicJoints() ;
  int size_u = 6 ;
  int size_fc = 24 ;
   
  
  double kc = 1E6 ;
  yarp::sig::Matrix Kq = locoman::utils::getKq(robot) ;
  yarp::sig::Matrix Kc(size_fc, size_fc) ;
  Kc.eye() ;
  Kc = kc*Kc ;    //   ;// 1E6*Kc ;    1E8*Kc ;  
    
/*  yarp::sig::Matrix Kc_left(size_fc/2, size_fc/2) ;
  Kc_left.eye() ;
  Kc_left = 1E4*Kc_left ;    //   ;// 1E6*Kc ;    1E8*Kc ;  
  
  yarp::sig::Matrix Kc_r(size_fc/2, size_fc/2) ;
  Kc_r.eye() ;
  Kc_r = kc*Kc_r ;    //   ;// 1E6*Kc ;    1E8*Kc ; */ 
  
  yarp::sig::Matrix Kc_f_rh(size_fc+size_fc/2, size_fc+size_fc/2) ;
  Kc_f_rh.eye() ;
  Kc_f_rh = (kc/10)*Kc_f_rh ;
  
  //------------------------------------------------------------------------------------------------------------------
//     robot.setReferenceSpeed(max_vel) ;

    std::cout << robot.getNumberOfKinematicJoints() << " getNumberOfKinematicJoints " << std::endl;
    yarp::sig::Vector q_current = locoman::utils::sense_position_no_hands(robot);
    
    std::cout << " sense size : " << q_current.size() << std::endl;
    robot.idynutils.updateiDyn3Model( q_current, true ); //update model first
    
    yarp::sig::Vector tau_current = locoman::utils::sense_torque_no_hands(robot) ;
    yarp::sig::Vector q_motor_side(robot.getNumberOfKinematicJoints() ) ;
    // using the q_ in open loop
    robot.idynutils.updateiDyn3Model( q_current_open_loop, true );   // q_current_open_loop    //  q_current
     
    
    
//     yarp::sig::Vector q_motor_side_1 = locoman::utils::senseMotorPosition( robot, 1 ) ;
//     yarp::sig::Vector q_motor_side_0 = locoman::utils::senseMotorPosition( robot, 0 ) ;
//         yarp::sig::Matrix Kq_walk = locoman::utils::getKq_Walkman(robot) ;
 //  yarp::sig::Vector q_computed_motor_wrong_tau = q_current -1.0* yarp::math::luinv(Kq_walk)*tau_current ;
    
//         std::cout << " flag_robot " << std::endl;   
// 
//    std::cout << " q_current_sensPos = "  <<  std::endl<<  q_current.toString() << std::endl; 
//    std::cout << " q_motor_side_0 = "  <<  std::endl << q_motor_side_0.toString() << std::endl;
//    std::cout << " q_motor_side_1 = "  <<  std::endl << q_motor_side_1.toString() << std::endl;
// 
//       std::cout << " q_computed_motor_wrong_tau = "  <<  std::endl << q_computed_motor_wrong_tau.toString() << std::endl;
// 
//       std::cout << " q_offset = "  <<  std::endl << q_offset.toString() << std::endl;

   
   
  //--------------------------------------------------------------------//
    //Getting Sensor Measures

    yarp::sig::Vector ft_l_ankle(6,0.0);
    if(!robot.senseftSensor("l_leg_ft", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    yarp::sig::Vector ft_r_ankle(6,0.0);
    if(!robot.senseftSensor("r_leg_ft", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;     

    yarp::sig::Vector ft_l_wrist(6,0.0);
    if(!robot.senseftSensor("l_arm_ft", ft_l_wrist)) std::cout << "ERROR READING SENSOR l_wrist" << std::endl; 
    yarp::sig::Vector ft_r_wrist(6,0.0);
    if(!robot.senseftSensor("r_arm_ft", ft_r_wrist)) std::cout << "ERROR READING SENSOR r_wrist" << std::endl;     

  std::cout << " r_wrist  =  "  << std::endl << ft_r_wrist.toString() << std::endl  ;     
  std::cout << " l_wrist  =  "  << std::endl << ft_l_wrist.toString() << std::endl  ;     

   
//---------------------------------------------------------------------------------------------------------//  
    // 
    int waist_index   = model.iDyn3_model.getLinkIndex("Waist");
    
    int l_ankle_index = model.iDyn3_model.getLinkIndex("l_leg_ft") ; // sensors are placed in *_ankle in the model
//     int l_ankle_index = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
    int l_c1_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link");
    int l_c2_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
    int l_c3_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link");
    int l_c4_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

    int r_ankle_index = model.iDyn3_model.getLinkIndex("r_leg_ft") ;
//     int r_ankle_index = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link") ;
    int r_c1_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
    int r_c2_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
    int r_c3_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
    int r_c4_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

    int l_hand_index  = model.iDyn3_model.getLinkIndex("LSoftHand");
    int r_hand_index  = model.iDyn3_model.getLinkIndex("RSoftHand");    

//     int l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
//     int l_hand_c1_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link");  // r_foot_upper_left_link
//     int l_hand_c2_index = model.iDyn3_model.getLinkIndex("l_hand_lower_right_link");  // r_foot_upper_right_link
//     int l_hand_c3_index = model.iDyn3_model.getLinkIndex("l_hand_upper_left_link");   // r_foot_lower_left_link
//     int l_hand_c4_index = model.iDyn3_model.getLinkIndex("l_hand_lower_left_link");  // r_foot_lower_right_link
//     
//     int r_wrist_index  = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
//     int r_hand_c1_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link");  // r_foot_upper_left_link
//     int r_hand_c2_index = model.iDyn3_model.getLinkIndex("r_hand_lower_right_link");  // r_foot_upper_right_link
//     int r_hand_c3_index = model.iDyn3_model.getLinkIndex("r_hand_upper_left_link");   // r_foot_lower_left_link
//     int r_hand_c4_index = model.iDyn3_model.getLinkIndex("r_hand_lower_left_link");  // r_foot_lower_right_link
        
    int l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
    int l_hand_c1_index = model.iDyn3_model.getLinkIndex("LSoftHand");  // r_foot_upper_left_link
    int l_hand_c2_index = model.iDyn3_model.getLinkIndex("LSoftHand");  // r_foot_upper_right_link
    int l_hand_c3_index = model.iDyn3_model.getLinkIndex("LSoftHand");   // r_foot_lower_left_link
    int l_hand_c4_index = model.iDyn3_model.getLinkIndex("LSoftHand");  // r_foot_lower_right_link
    
    int r_wrist_index  = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
    int r_hand_c1_index = model.iDyn3_model.getLinkIndex("RSoftHand");  // r_foot_upper_left_link
    int r_hand_c2_index = model.iDyn3_model.getLinkIndex("RSoftHand");  // r_foot_upper_right_link
    int r_hand_c3_index = model.iDyn3_model.getLinkIndex("RSoftHand");   // r_foot_lower_left_link
    int r_hand_c4_index = model.iDyn3_model.getLinkIndex("RSoftHand");  // r_foot_lower_right_link
    
    
    yarp::sig::Matrix map_l_fcToSens =   locoman::utils::fConToSens( l_ankle_index, 
						      l_c1_index  , 
					              l_c2_index  ,  						      
						      l_c3_index  , 
						      l_c4_index,
                                                      model
                                                                   ) ;
						      
    yarp::sig::Matrix map_r_fcToSens =   locoman::utils::fConToSens( r_ankle_index, 
					              r_c1_index, 
						      r_c2_index,
						      r_c3_index, 
						      r_c4_index,
                                                      model
                                                                   ) ;
						     
    
    yarp::sig::Matrix map_r_hand_fcToSens = locoman::utils::fConToSens(r_wrist_index, 
					               r_hand_c1_index, 
						       r_hand_c2_index,
						       r_hand_c3_index, 
						       r_hand_c4_index ,
                                                       model
                                                                      ) ;
// 	 
// 
    yarp::sig::Matrix map_l_hand_fcToSens = locoman::utils::fConToSens(l_wrist_index, 
					               l_hand_c1_index, 
						       l_hand_c2_index,
						       l_hand_c3_index, 
						       l_hand_c4_index,
                                                       model
                                                                      ) ;
						    
  std::cout << " map_l_fcToSens  =  "  << std::endl << map_l_fcToSens.toString() << std::endl  ;     
  std::cout << " ft_l_ankle  =  "  << std::endl << ft_l_ankle.toString() << std::endl  ;      

						       
						   
    yarp::sig::Vector fc_l_c_to_robot = locoman::utils::Pinv_trunc_SVD( map_l_fcToSens, 1E-10) *  ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_c_to_robot = locoman::utils::Pinv_trunc_SVD( map_r_fcToSens, 1E-10) *  ft_r_ankle  ;  // yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
    yarp::sig::Vector fc_l_c_to_world =  - 1.0 * fc_l_c_to_robot     ;
    yarp::sig::Vector fc_r_c_to_world =  - 1.0 * fc_r_c_to_robot     ;
    
    if(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman"){  // Changing the sign again in case of walkman real robot
          fc_l_c_to_world  = -1.0*fc_l_c_to_world ;
          fc_r_c_to_world  = -1.0*fc_r_c_to_world ;
    }
    
    yarp::sig::Vector fc_l_c_hand_to_robot = locoman::utils::Pinv_trunc_SVD( map_l_hand_fcToSens, 1E-10) *  ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_c_hand_to_robot = locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10) *  ft_r_wrist  ;  // yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
    yarp::sig::Vector fc_l_c_hand_to_world =  - 1.0 * fc_l_c_hand_to_robot     ;
    yarp::sig::Vector fc_r_c_hand_to_world =  - 1.0 * fc_r_c_hand_to_robot     ;    
    
    yarp::sig::Vector fc_to_world_0(size_fc) ;
    fc_to_world_0.setSubvector(0, fc_l_c_to_world ) ;
    fc_to_world_0.setSubvector(fc_l_c_to_world.length(), fc_r_c_to_world ) ;    
    
    yarp::sig::Vector fc_hand_to_world_0(size_fc) ;
    fc_hand_to_world_0.setSubvector(0, fc_l_c_hand_to_world ) ;
    fc_hand_to_world_0.setSubvector(fc_l_c_hand_to_world.length(), fc_r_c_hand_to_world ) ;  
    if(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman"){  // Changing the sign again in case of walkman real robot
          fc_hand_to_world_0  = -1.0*fc_hand_to_world_0 ;
          fc_hand_to_world_0  = -1.0*fc_hand_to_world_0 ;
    }
    
//-------------------------------------------------------------------------------------------------------------// 
    // Fc filtering   
    int counter_window = loop_counter% WINDOW_size ;
    //  
    FC_WINDOW.setCol( counter_window , fc_to_world_0 ) ;
    for(int k=0; k<fc_to_world_0.length() ; k++ )
    {
    FC_SUM[k] = FC_SUM[k]+ fc_to_world_0[k]- FC_WINDOW[k][(counter_window+ 1)%WINDOW_size]  ;
    }
    FC_FILTERED = FC_SUM / WINDOW_size ;  // to_world 
    //
    yarp::sig:: Vector FC_FILTERED_LEFT_sensor = map_l_fcToSens * FC_FILTERED.subVector(0,11) ;
    yarp::sig:: Vector FC_FILTERED_RIGHT_sensor = map_r_fcToSens* FC_FILTERED.subVector(12,23) ;

    //------

    FC_HANDS_WINDOW.setCol( counter_window , fc_hand_to_world_0 ) ;
    for(int k=0; k<fc_hand_to_world_0.length() ; k++ )
    {
    FC_HANDS_SUM[k] = FC_HANDS_SUM[k]+ fc_hand_to_world_0[k]- FC_HANDS_WINDOW[k][(counter_window+ 1)%WINDOW_size]  ;
    }  
    FC_HANDS_FILTERED = FC_HANDS_SUM / WINDOW_size ;  // to_world 
    //
    yarp::sig:: Vector FC_FILTERED_LEFT_HAND_sensor = map_l_hand_fcToSens * FC_HANDS_FILTERED.subVector(0,11) ;
    yarp::sig:: Vector FC_FILTERED_RIGHT_HAND_sensor = map_r_hand_fcToSens* FC_HANDS_FILTERED.subVector(12,23) ; 
    
   //---------- 
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " loop_counter = " <<  loop_counter << std::endl; 
   std::cout << " counter_window = " <<  counter_window << std::endl; 
   std::cout << " FC_WINDOW = " <<  std::endl << (FC_WINDOW.submatrix(0,FC_WINDOW.rows()-1,(FC_WINDOW.cols()-5),(FC_WINDOW.cols()-1) )).toString() << std::endl;
   std::cout << " FC_FILTERED_LEFT_sensor  =  "  << std::endl << FC_FILTERED_LEFT_sensor.toString() << std::endl  ; 
   std::cout << " FC_FILTERED_RIGHT_sensor  =  "  << std::endl << FC_FILTERED_RIGHT_sensor.toString() << std::endl  ;     
   std::cout << " FC_FILTERED = " <<  std::endl << FC_FILTERED.toString() << std::endl; 
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " FC_FILTERED_LEFT_HAND_sensor  =  "  << std::endl << FC_FILTERED_LEFT_HAND_sensor.toString() << std::endl  ; 
   std::cout << " FC_FILTERED_RIGHT_HAND_sensor  =  "  << std::endl << FC_FILTERED_RIGHT_HAND_sensor.toString() << std::endl  ;
   std::cout << " FC_FILTERED_RIGHT_HAND  =  "  << std::endl << FC_HANDS_FILTERED.subVector(12,23).toString() << std::endl  ;

   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   
   yarp::sig::Vector q_motor_init(robot.getNumberOfKinematicJoints() ) ;                

   robot.fromRobotToIdyn29(  right_arm_configuration ,
                           left_arm_configuration  ,
                           torso_configuration     ,
                           right_leg_configuration ,
                           left_leg_configuration  ,
                           q_motor_init     )  ;
   yarp::sig::Vector q_sense_position = locoman::utils::sense_position_no_hands(robot) ;                        
   //yarp::sig::Vector q_sense_motor_position = locoman::utils::senseMotorPosition(robot, flag_robot) ;    
   
   yarp::sig::Vector q_recostructed = q_sense_position +q_offset ; 
   
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " q_motor_init = "  <<  std::endl<<  q_motor_init.toString() << std::endl; 
  //std::cout << " q_sense_position = " <<  std::endl <<  q_sense_position.toString() << std::endl; 
  // std::cout << " q_sense_motor_position = " <<  std::endl << q_sense_motor_position.toString() << std::endl;
   std::cout << " q_recostructed = " <<  std::endl << q_recostructed.toString() << std::endl;

   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 

   
   yarp::sig::Vector q_reconstr_r_arm(right_arm_configuration.length()) ;
   yarp::sig::Vector q_reconstr_l_arm(left_arm_configuration.length()) ; ;
   yarp::sig::Vector q_reconstr_torso(torso_configuration.length()) ; ;
   yarp::sig::Vector q_reconstr_r_leg(right_leg_configuration.length()) ; ;
   yarp::sig::Vector q_reconstr_l_leg(left_leg_configuration.length()) ;
   
    robot.fromIdynToRobot29(  q_recostructed  ,
                            q_reconstr_r_arm,
                            q_reconstr_l_arm,
                            q_reconstr_torso,
                            q_reconstr_r_leg,
                            q_reconstr_l_leg  ) ;  // extracting values for the chains
 
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
//    std::cout << " right_arm_configuration = "  <<  std::endl <<  right_arm_configuration.toString() << std::endl; 
//    std::cout << " q_reconstr_r_arm = "         <<  std::endl <<  q_reconstr_r_arm.toString() << std::endl;
// 
//    std::cout << " left_arm_configuration = "  <<  std::endl <<  left_arm_configuration.toString() << std::endl; 
//    std::cout << " q_reconstr_l_arm = "        <<  std::endl <<  q_reconstr_l_arm.toString() << std::endl;
//    
//    std::cout << " torso_configuration = "  <<  std::endl <<  torso_configuration.toString() << std::endl; 
//    std::cout << " q_reconstr_torso = "     <<  std::endl <<  q_reconstr_torso.toString() << std::endl;
//    
//    std::cout << " right_leg_configuration = "  <<  std::endl<<  right_leg_configuration.toString() << std::endl; 
//    std::cout << " q_reconstr_r_leg = "         <<  std::endl << q_reconstr_r_leg.toString() << std::endl;
//    
//    std::cout << " left_leg_configuration = "  <<  std::endl<<  left_leg_configuration.toString() << std::endl; 
//    std::cout << " q_reconstr_l_leg = "        <<  std::endl << q_reconstr_l_leg.toString() << std::endl;
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
 
//     q_right_arm_des = right_arm_configuration  ; //Insertion of  predefined values (from .ini)                      
//     q_left_arm_des  = left_arm_configuration   ;
//     q_torso_des     = torso_configuration      ;
//     q_left_leg_des  = left_leg_configuration   ;
//     q_right_leg_des = right_leg_configuration  ;
//     
//     robot.fromRobotToIdyn( q_right_arm_des ,
//                            q_left_arm_des  ,
//                            q_torso_des     ,
//                            q_right_leg_des ,
//                            q_left_leg_des  ,
//                            q_des            );     // building q_des
   
   
   
   loop_counter++ ;   
  // End of Contact Forces Filtering
  //
  //-----------------------------------------------------------------------------------------
  // Contact Forces
  yarp::sig::Vector fc_l_c1_filt = FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l_c2_filt = FC_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l_c3_filt = FC_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l_c4_filt = FC_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r_c1_filt = FC_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r_c2_filt = FC_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r_c3_filt = FC_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r_c4_filt = FC_FILTERED.subVector(21,23)  ; 
  
  yarp::sig::Vector fc_l1_hand_filt = FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l2_hand_filt = FC_HANDS_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l3_hand_filt = FC_HANDS_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l4_hand_filt = FC_HANDS_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r1_hand_filt = FC_HANDS_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r2_hand_filt = FC_HANDS_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r3_hand_filt = FC_HANDS_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r4_hand_filt = FC_HANDS_FILTERED.subVector(21,23)  ;   

  //-----------------------------------------------------------------------------------------------------------------
  // Defining the "Auxiliary World" Frame => {AW}

  yarp::sig::Matrix T_w_aw_0 = locoman::utils::AW_world_posture(model, robot) ;
  yarp::sig::Matrix T_aw_w_0 = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  yarp::sig::Matrix T_w_waist_0   = model.iDyn3_model.getPosition(waist_index) ;  
  yarp::sig::Matrix T_w_l_ankle_0 = model.iDyn3_model.getPosition(l_ankle_index) ;
  yarp::sig::Matrix T_w_l_c1_0    = model.iDyn3_model.getPosition(l_c1_index)    ;    
  yarp::sig::Matrix T_w_l_c2_0    = model.iDyn3_model.getPosition(l_c2_index)    ;  
  yarp::sig::Matrix T_w_l_c3_0    = model.iDyn3_model.getPosition(l_c3_index)    ;
  yarp::sig::Matrix T_w_l_c4_0    = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_ankle_0 = model.iDyn3_model.getPosition(r_ankle_index) ;
  yarp::sig::Matrix T_w_r_c1_0    = model.iDyn3_model.getPosition(r_c1_index)    ;    
  yarp::sig::Matrix T_w_r_c2_0    = model.iDyn3_model.getPosition(r_c2_index)    ;  
  yarp::sig::Matrix T_w_r_c3_0    = model.iDyn3_model.getPosition(r_c3_index)    ;
  yarp::sig::Matrix T_w_r_c4_0    = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  yarp::sig::Matrix T_w_l_hand_0  = model.iDyn3_model.getPosition( l_hand_index ) ;
  yarp::sig::Matrix T_w_r_hand_0  = model.iDyn3_model.getPosition( r_hand_index ) ;   

  yarp::sig::Matrix T_w_l_wrist_0 = model.iDyn3_model.getPosition(l_wrist_index) ;
  yarp::sig::Matrix T_w_l1_hand_0 = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_l2_hand_0 = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_l3_hand_0 = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  yarp::sig::Matrix T_w_l4_hand_0 = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_wrist_0 = model.iDyn3_model.getPosition(r_wrist_index) ;
  yarp::sig::Matrix T_w_r1_hand_0 = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_r2_hand_0 = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_r3_hand_0 = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  yarp::sig::Matrix T_w_r4_hand_0 = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  // -----------------------------------------------------------------------
  yarp::sig::Matrix T_waist_w_0   = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  yarp::sig::Matrix T_l_ankle_w_0 = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  yarp::sig::Matrix T_l_c1_w_0    = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  yarp::sig::Matrix T_l_c2_w_0    = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  yarp::sig::Matrix T_l_c3_w_0    = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  yarp::sig::Matrix T_l_c4_w_0    = locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  yarp::sig::Matrix T_r_ankle_w_0 = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  yarp::sig::Matrix T_r_c1_w_0    = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  yarp::sig::Matrix T_r_c2_w_0    = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  yarp::sig::Matrix T_r_c3_w_0    = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  yarp::sig::Matrix T_r_c4_w_0    = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  yarp::sig::Matrix T_l_wrist_w_0 = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  yarp::sig::Matrix T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  yarp::sig::Matrix T_l2_hand_w_0 = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  yarp::sig::Matrix T_l3_hand_w_0 = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  yarp::sig::Matrix T_l4_hand_w_0 = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  yarp::sig::Matrix T_r_wrist_w_0 = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  yarp::sig::Matrix T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  yarp::sig::Matrix T_r2_hand_w_0 = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  yarp::sig::Matrix T_r3_hand_w_0 = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  yarp::sig::Matrix T_r4_hand_w_0 = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  //---------------------------------------------------------------------
  
  yarp::sig::Matrix T_l_hand_w_0 = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  yarp::sig::Matrix T_r_hand_w_0 = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  yarp::sig::Matrix T_aw_l_c1_0 = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l_c2_0 = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l_c3_0 = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l_c4_0 = T_aw_w_0 * T_w_l_c4_0 ;

  yarp::sig::Matrix T_aw_r_c1_0 = T_aw_w_0 * T_w_r_c1_0 ;
  yarp::sig::Matrix T_aw_r_c2_0 = T_aw_w_0 * T_w_r_c2_0 ;
  yarp::sig::Matrix T_aw_r_c3_0 = T_aw_w_0 * T_w_r_c3_0 ;
  yarp::sig::Matrix T_aw_r_c4_0 = T_aw_w_0 * T_w_r_c4_0 ; 

  yarp::sig::Matrix T_aw_l1_hand_0 = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l2_hand_0 = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l3_hand_0 = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l4_hand_0 = T_aw_w_0 * T_w_l4_hand_0 ;

  yarp::sig::Matrix T_aw_r1_hand_0 = T_aw_w_0 * T_w_r1_hand_0 ;
  yarp::sig::Matrix T_aw_r2_hand_0 = T_aw_w_0 * T_w_r2_hand_0 ;
  yarp::sig::Matrix T_aw_r3_hand_0 = T_aw_w_0 * T_w_r3_hand_0 ;
  yarp::sig::Matrix T_aw_r4_hand_0 = T_aw_w_0 * T_w_r4_hand_0 ; 
  
  //---------------------------------------------------------------------------------------
  // Jacobian Matrices 
  yarp::sig::Matrix J_l_c1_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c2_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c3_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c4_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r_c1_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c2_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c3_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c4_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l1_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l2_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l3_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l4_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r1_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r2_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r3_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r4_hand_mix_0( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //-----------------------------------------------------
  model.iDyn3_model.getJacobian( l_c1_index, J_l_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c2_index, J_l_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c3_index, J_l_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_c4_index, J_l_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  model.iDyn3_model.getJacobian( r_c1_index, J_r_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c2_index, J_r_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c3_index, J_r_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_c4_index, J_r_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
 
  model.iDyn3_model.getJacobian( l_hand_c1_index, J_l1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c2_index, J_l2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c3_index, J_l3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( l_hand_c4_index, J_l4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  model.iDyn3_model.getJacobian( r_hand_c1_index, J_r1_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c2_index, J_r2_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c3_index, J_r3_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_c4_index, J_r4_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    
  model.iDyn3_model.getJacobian( l_hand_index, J_l_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
  model.iDyn3_model.getJacobian( r_hand_index, J_r_hand_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

  yarp::sig::Matrix J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  yarp::sig::Matrix J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  yarp::sig::Matrix J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  yarp::sig::Matrix J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  yarp::sig::Matrix J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  yarp::sig::Matrix J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  yarp::sig::Matrix J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  yarp::sig::Matrix J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  yarp::sig::Matrix J_l_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  yarp::sig::Matrix J_r_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  yarp::sig::Matrix J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  yarp::sig::Matrix J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  yarp::sig::Matrix J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  yarp::sig::Matrix J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  yarp::sig::Matrix J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  yarp::sig::Matrix J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  yarp::sig::Matrix J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  yarp::sig::Matrix J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Introducing Spatial Jacobian terms: Fixed base in {AW}
  
  yarp::sig::Matrix J_aw_l_c1_spa_0 = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l_c2_spa_0 = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l_c3_spa_0 = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l_c4_spa_0 = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r_c2_spa_0 = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r_c3_spa_0 = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r_c4_spa_0 = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  yarp::sig::Matrix J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l2_hand_spa_0 = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l3_hand_spa_0 = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l4_hand_spa_0 = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r2_hand_spa_0 = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r3_hand_spa_0 = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r4_hand_spa_0 = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  J_aw_l_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_r_c1_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c2_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c3_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r_c4_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_l1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_l4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
 
  J_aw_r1_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r2_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r3_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  J_aw_r4_hand_spa_0.setSubmatrix( Eye_6, 0 ,  0 )  ;
  
  // Recomputing body Jacobian

  J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c1_0) ) * J_aw_l_c1_spa_0 ;
  J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c2_0) ) * J_aw_l_c2_spa_0 ;
  J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c3_0) ) * J_aw_l_c3_spa_0 ;
  J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l_c4_0) ) * J_aw_l_c4_spa_0 ;

  J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c1_0) ) * J_aw_r_c1_spa_0 ;
  J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c2_0) ) * J_aw_r_c2_spa_0 ;
  J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c3_0) ) * J_aw_r_c3_spa_0 ;
  J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r_c4_0) ) * J_aw_r_c4_spa_0 ;

  J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l1_hand_0) ) * J_aw_l1_hand_spa_0 ;
  J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l2_hand_0) ) * J_aw_l2_hand_spa_0 ;
  J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l3_hand_0) ) * J_aw_l3_hand_spa_0 ;
  J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_l4_hand_0) ) * J_aw_l4_hand_spa_0 ;

  J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r1_hand_0) ) * J_aw_r1_hand_spa_0 ;
  J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r2_hand_0) ) * J_aw_r2_hand_spa_0 ;
  J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r3_hand_0) ) * J_aw_r3_hand_spa_0 ;
  J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::iHomogeneous(T_aw_r4_hand_0) ) * J_aw_r4_hand_spa_0 ;
  //------------------------------------------------------------------------------------------------------------
  //  
  // Stance and Jacobian Matrices
    
  yarp::sig::Matrix Complete_Jac( 8*B.cols() , size_q + 6) ;
  Complete_Jac.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;

  Complete_Jac.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
  Complete_Jac.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;

  yarp::sig::Matrix J_c   = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 6, Complete_Jac.cols()-1 ) ;
  yarp::sig::Matrix S_c_T = Complete_Jac.submatrix( 0,  Complete_Jac.rows()-1 , 0, 5 ) ;
  yarp::sig::Matrix S_c   = S_c_T.transposed() ;
  
//   yarp::sig::Matrix J_c_l   = J_c.submatrix( 0, (J_c.rows()/2)-1, 0, J_c.cols() -1) ;
//   yarp::sig::Matrix S_c_l_T = S_c_T.submatrix( 0, (S_c_T.rows()/2)-1, 0, S_c_T.cols()-1 ) ;
//   yarp::sig::Matrix S_c_l   = S_c_l_T.transposed() ;
//   
//   yarp::sig::Matrix J_c_r   = J_c.submatrix( (J_c.rows()/2), J_c.rows()-1,  0 , J_c.cols()-1 ) ;
//   yarp::sig::Matrix S_c_r_T = S_c_T.submatrix((S_c_T.rows()/2), S_c_T.rows()-1, 0, S_c_T.cols()-1 ) ;
//   yarp::sig::Matrix S_c_r   = S_c_r_T.transposed() ; 
    
  yarp::sig::Matrix Complete_Jac_f_rh( 12*B.cols() , size_q + 6) ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c1_body_0 , 0 ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c2_body_0 , B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c3_body_0 , 2*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_l_c4_body_0 , 3*B.cols() ,0 )  ;

  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c1_body_0 , 4*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c2_body_0 , 5*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c3_body_0 , 6*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r_c4_body_0 , 7*B.cols() ,0 )  ;

  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r1_hand_body_0 , 8*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r2_hand_body_0 , 9*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r3_hand_body_0 , 10*B.cols() ,0 )  ;
  Complete_Jac_f_rh.setSubmatrix( B.transposed()*J_r4_hand_body_0 , 11*B.cols() ,0 )  ; 
  
  yarp::sig::Matrix J_c_f_rh   = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 6, Complete_Jac_f_rh.cols()-1 ) ;
  yarp::sig::Matrix S_c_f_rh_T = Complete_Jac_f_rh.submatrix( 0,  Complete_Jac_f_rh.rows()-1 , 0, 5 ) ;
  yarp::sig::Matrix S_c_f_rh   = S_c_f_rh_T.transposed() ;
  //----------------------------------------------------------
  // -------------------------------------------------------------------------------------------------------------
  // Defining derivative Terms

  yarp::sig::Matrix Q_aw_l_c1(size_q+ 6, size_q + 6)   ; //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  yarp::sig::Matrix Q_aw_l_c2(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_l_c3(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_l_c4(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  yarp::sig::Matrix Q_aw_r_c1(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c2(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c3(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r_c4(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  yarp::sig::Matrix Q_aw_r1_hand(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r2_hand(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r3_hand(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r4_hand(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  yarp::sig::Matrix Q_aw_l_tot(size_q+ 6, size_q + 6)   ; // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  yarp::sig::Matrix Q_aw_r_tot(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  yarp::sig::Matrix Q_aw_r_hand_tot(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;

  yarp::sig::Matrix Q_aw_c(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  
  yarp::sig::Matrix U_aw_s_cont( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_cont( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  
  yarp::sig::Matrix Q_aw_c_f_rh(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_c_f_rh( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_c_f_rh( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
        
//----------------------------------------------------------------------------------------------------//
 
  // Computing Derivative Terms
  Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  Q_aw_r_c1 = locoman::utils::Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2 = locoman::utils::Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3 = locoman::utils::Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4 = locoman::utils::Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_r1_hand = locoman::utils::Q_ci(J_aw_r1_hand_spa_0, T_aw_r1_hand_0, fc_r1_hand_filt ) ; //(size_q+ 6, size_q + 6) ;  
  Q_aw_r2_hand = locoman::utils::Q_ci(J_aw_r2_hand_spa_0, T_aw_r2_hand_0, fc_r2_hand_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r3_hand = locoman::utils::Q_ci(J_aw_r3_hand_spa_0, T_aw_r3_hand_0, fc_r3_hand_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r4_hand = locoman::utils::Q_ci(J_aw_r4_hand_spa_0, T_aw_r4_hand_0, fc_r4_hand_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_l_tot = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  Q_aw_r_tot = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_r_hand_tot = Q_aw_r1_hand + Q_aw_r2_hand + Q_aw_r3_hand + Q_aw_r4_hand;

  Q_aw_c =  Q_aw_l_tot + Q_aw_r_tot ;  

  U_aw_s_cont = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_cont = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;

//   yarp::sig::Matrix U_aw_l_s_cont = Q_aw_l_tot.submatrix( 0 ,  5 , 0, 5) ;     
//   yarp::sig::Matrix Q_aw_l_s_cont = Q_aw_l_tot.submatrix( 0  , 5,  6,  (Q_aw_l_tot.cols()-1)  ) ;
//   yarp::sig::Matrix U_aw_r_s_cont = Q_aw_r_tot.submatrix( 0 ,  5 , 0, 5) ;     
//   yarp::sig::Matrix Q_aw_r_s_cont = Q_aw_r_tot.submatrix( 0  , 5,  6,  (Q_aw_r_tot.cols()-1)  ) ;
//  
  Q_aw_c_f_rh = Q_aw_l_tot + Q_aw_r_tot   + Q_aw_r_hand_tot ;  
  U_aw_s_c_f_rh = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_c_f_rh = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c_f_rh.cols()-1)  ) ;

  // 
  //-------------------------------------------------------------------------------------------
  // Useful Definitions
  yarp::sig::Vector d_fc_des_to_world(size_fc)  ;
  yarp::sig::Vector d_EE_r_des(6,0.0) ;
  yarp::sig::Vector d_EE_l_des(6,0.0) ;
  yarp::sig::Matrix T_l_c1_r_c1_loop(4,4) ;
  T_l_c1_r_c1_loop.zero() ;
  yarp::sig::Matrix T_r_c1_l_c1_loop(4,4) ;
  T_r_c1_l_c1_loop.zero() ;  
  yarp::sig::Matrix J_com_w( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_com_w_redu( 3,  ( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_aw( 3,  ( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_waist( 3,  ( robot.getNumberOfKinematicJoints() + 6 ))   ;

  yarp::sig::Matrix J_r_c1_aw( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_l_c1_aw( 6, ( robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Vector q_ref_ToMove( robot.getNumberOfKinematicJoints() + 6 , 0.0) ;
  //
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw     = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_com_waist  = locoman::utils::getRot(T_waist_w_0)*J_com_w_redu;
  //-----------------------------------------
  
  yarp::sig::Matrix T_waist_l_hand_0  = T_waist_w_0 * T_w_l_hand_0  ;    
  yarp::sig::Matrix T_waist_r_hand_0  = T_waist_w_0 * T_w_r_hand_0  ;    
  yarp::sig::Matrix T_waist_l1_foot_0 = T_waist_w_0 * T_w_l_c1_0 ; 
  yarp::sig::Matrix T_waist_r1_foot_0 = T_waist_w_0 * T_w_r_c1_0 ; 
  yarp::sig::Vector CoM_waist_0       = locoman::utils::getRot( T_waist_w_0 ) * CoM_w_cmd ;
  yarp::sig::Matrix R_waist_aw_0      = locoman::utils::getRot(T_waist_w_0 *T_w_aw_0) ; 
  
  yarp::sig::Matrix Eye_4(4,4) ;
  Eye_4.eye() ;
  //--------------------------------------------------------------------------------------------------------------------------------------------  
  // Input-Guided State Machine => Building the proper FLLM and control in each state
    
  std::string command  ; //  
  if(bool ifCommand = command_interface.getCommand(command) ){
       //  if(command!=last_command){
  std::cout << " ifCommand  =  "<< std::endl << ifCommand << std::endl  ; 
  CoM_w_cmd = model.iDyn3_model.getCOM()  ;  //   ...cmd is for "@ command time"
  //-------------------------------------------------------------------------------
  T_waist_l_hand_cmd  = T_waist_w_0 * T_w_l_hand_0  ;    
  T_waist_r_hand_cmd  = T_waist_w_0 * T_w_r_hand_0  ;    
  T_waist_l1_foot_cmd = T_waist_w_0 * T_w_l_c1_0 ; 
  T_waist_r1_foot_cmd = T_waist_w_0 * T_w_r_c1_0 ; 
  CoM_waist_cmd       = locoman::utils::getRot( T_waist_w_0 ) * CoM_w_cmd ;
  R_waist_aw_cmd      = locoman::utils::getRot(T_waist_w_0 *T_w_aw_0) ; 
  //------------------------------------------------------------------------------_
  // std::cout << " CoM_w_cmd  =  "<< std::endl << CoM_w_cmd.toString() << std::endl  ; 
  
  yarp::sig::Vector CoM_aw_cmd = locoman::utils::getRot(T_aw_w_0)*CoM_w_cmd ;
  yarp::sig::Vector CoM_aw_up = CoM_aw_cmd ; 
  yarp::sig::Vector CoM_aw_dw = CoM_aw_cmd ; 
  double com_z_var = 0.02 ;
  CoM_aw_up[2] += com_z_var ;
  CoM_aw_dw[2] -= com_z_var ;  
  
  CoM_w_up = locoman::utils::getRot(T_w_aw_0)*CoM_aw_up ;
  CoM_w_dw = locoman::utils::getRot(T_w_aw_0)*CoM_aw_dw ;
    
  T_w_r1_cmd = T_w_r_c1_0 ;
  T_w_l1_cmd = T_w_l_c1_0 ;
  
  //-------------------------------------------------------------------------------------
  yarp::sig::Matrix Rot_des_r = locoman::utils::Rot_z(-0.2) ;
  yarp::sig::Matrix Rot_des_l = locoman::utils::Rot_z( 0.2) ;
  
  //----------------------------------------------------------------------------------- 
  //    T_l1_r1_up  
  T_l1_r1_up = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
  double z_r1_up = 0.14 ;
  yarp::sig::Matrix T_aw_r1_up = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_l_c1_0),zero_3)* T_l1_r1_up ;
  T_aw_r1_up[2][3] += z_r1_up;
  T_l1_r1_up = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_l_c1_0)),zero_3)*T_aw_r1_up  ;
  T_l1_r1_up.setSubmatrix( Rot_des_r,0,0 );
  //std::cout << " T_l1_r1_up  =  "<< std::endl << T_l1_r1_up.toString() << std::endl  ; 
  //std::cout << " T_l1_r1_up  =  "<< std::endl << T_l1_r1_up.toString() << std::endl  ; 

  //    T_r1_l1_up  
  T_r1_l1_up = locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0;
  double z_l1_up = 0.04 ;
  yarp::sig::Matrix T_aw_l1_up = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_r_c1_0),zero_3)* T_r1_l1_up ;
  T_aw_l1_up[2][3] += z_l1_up;
  T_r1_l1_up = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_r_c1_0)),zero_3)*T_aw_l1_up  ;
  T_r1_l1_up.setSubmatrix( Rot_des_l,0,0 );
  //std::cout << " T_r1_l1_up  =  "<< std::endl << T_r1_l1_up.toString() << std::endl  ; 
  //std::cout << " T_r1_l1_up  =  "<< std::endl << T_r1_l1_up.toString() << std::endl  ;   
  
  //-----------------------------------------------------------------------------------
  //    T_l1_r1_fw  
  T_l1_r1_fw = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
  yarp::sig::Vector x_l1_r1(3,0.0) ;
  x_l1_r1[0]=  T_l1_r1_fw[0][0];
  x_l1_r1[1]=  T_l1_r1_fw[1][0];
  x_l1_r1[2]=  T_l1_r1_fw[2][0];
  //--------------------------
  yarp::sig::Vector x_l1(3,0.0) ;
  x_l1[0]=  1.0 ;
  x_l1[1]=  0.3 ;
  x_l1[2]=  0.0 ;
  x_l1 = x_l1/(norm(x_l1)) ;
  
  yarp::sig::Vector x_fw(3,0.0) ;
  
  double alpha_left = 0.5 ;
  double alpha_right = 1.0 - alpha_left ;
  x_fw[0] = alpha_left*x_l1[0] + alpha_right*x_l1_r1[0] ;
  x_fw[1] = alpha_left*x_l1[1] + alpha_right*x_l1_r1[1];
  x_fw[2] = alpha_left*x_l1[2] + alpha_right*x_l1_r1[2];
  x_fw = x_fw/(norm(x_fw)) ;
  double x_r1_fw = 0.04 ;
  T_l1_r1_fw[0][3] += x_r1_fw*x_fw[0] ;
  T_l1_r1_fw[1][3] += x_r1_fw*x_fw[1] ;
  T_l1_r1_fw[2][3] += x_r1_fw*x_fw[2] ;
  T_l1_r1_fw.setSubmatrix( Rot_des_r,0,0 );
  std::cout << " T_l1_r1_fw  =  "<< std::endl << T_l1_r1_fw.toString() << std::endl  ; 

   //    T_r1_l1_fw  
  T_r1_l1_fw = locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0 ;
  yarp::sig::Vector x_r1_l1(3,0.0) ;
  x_r1_l1[0]=  T_r1_l1_fw[0][0];
  x_r1_l1[1]=  T_r1_l1_fw[1][0];
  x_r1_l1[2]=  T_r1_l1_fw[2][0];
  //--------------------------
  yarp::sig::Vector x_r1(3,0.0) ;
  x_r1[0]=  1.0 ;
  x_r1[1]= -0.3 ;
  x_r1[2]=  0.0 ;
  x_r1 = x_r1/(norm(x_r1)) ;
  
  yarp::sig::Vector x_fw_l(3,0.0) ;
  double alpha_left_l = 0.5 ;
  double alpha_right_l = 1.0 - alpha_left ;
  x_fw_l[0] = alpha_left_l*x_r1[0] + alpha_right_l*x_r1_l1[0] ;
  x_fw_l[1] = alpha_left_l*x_r1[1] + alpha_right_l*x_r1_l1[1];
  x_fw_l[2] = alpha_left_l*x_r1[2] + alpha_right_l*x_r1_l1[2];
  x_fw_l = x_fw_l/(norm(x_fw_l)) ;
  double x_l1_fw = 0.04 ;
  T_r1_l1_fw[0][3] += x_l1_fw*x_fw_l[0] ;
  T_r1_l1_fw[1][3] += x_l1_fw*x_fw_l[1] ;
  T_r1_l1_fw[2][3] += x_l1_fw*x_fw_l[2] ;
  T_r1_l1_fw.setSubmatrix( Rot_des_l,0,0 );
  std::cout << " T_r1_l1_fw  =  "<< std::endl << T_r1_l1_fw.toString() << std::endl  ; 
  
  //-----------------------------------------------------------------------------------
 //    T_l1_r1_dw  
  T_l1_r1_dw = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
  double z_r1_dw = -10.0*z_r1_up ;
  yarp::sig::Matrix T_aw_r1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_l_c1_0),zero_3)* T_l1_r1_dw ;
  T_aw_r1_dw[2][3] += z_r1_dw ;
  T_l1_r1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_l_c1_0)),zero_3)*T_aw_r1_dw  ;
  T_l1_r1_dw.setSubmatrix( Rot_des_r,0,0 );  
  std::cout << " T_l1_r1_dw  =  "<< std::endl << T_l1_r1_dw.toString() << std::endl  ; 
//  
//    T_r1_l1_dw  
  T_r1_l1_dw = locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0 ;
  double z_l1_dw = -10.0*z_l1_up ;
  yarp::sig::Matrix T_aw_l1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_r_c1_0),zero_3)* T_r1_l1_dw ;
  T_aw_l1_dw[2][3] += z_l1_dw ;
  T_r1_l1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_r_c1_0)),zero_3)*T_aw_l1_dw  ;
  T_r1_l1_dw.setSubmatrix( Rot_des_l,0,0 );  
  std::cout << " T_r1_l1_dw  =  "<< std::endl << T_r1_l1_dw.toString() << std::endl  ; 
//  
//   
 // }
  }
  
  if (command != "")    {
   last_command = command ; 
  }
       std::cout << " last_command  =  "<< std::endl << last_command << std::endl  ; 

  
  if (last_command=="pause")
  {  }
  else if (last_command =="start" || last_command =="resume" ||
           last_command =="to_rg" || last_command =="to_lf" || last_command =="to_cr" || last_command =="center"  )
  {  // Double Stance Phase  

    if (last_command =="to_rg")
     {
        locoman::utils::FC_DES_right(FC_DES, mg) ;  // all the weight on the right foot
     }
     else if (last_command =="to_lf")
     { 
        locoman::utils::FC_DES_left(FC_DES, mg) ; // all the weight on the left foot
     //   std::cout << " CoM_w_0  =  "<< std::endl << CoM_w_0.toString() << std::endl  ; 

     }
    else if (last_command =="to_cr" || last_command =="center")
     { 
        locoman::utils::FC_DES_center(FC_DES, mg) ;  // half weight on the right, half on the left foot
     }
  
 // desired contact force definition  
  FC_DES_LEFT_sensor  = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
  FC_DES_RIGHT_sensor = map_l_fcToSens * FC_DES.subVector(12,23)  ;
 
  d_fc_des_to_world  = FC_DES - FC_FILTERED ; // -fc_to_world_0 ;

  std::cout << " d_fc_des_to_world = " << d_fc_des_to_world.toString() <<  std::endl ;

  //-----------------------------------------------------------------------------------------------------------------     
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " FC_DES = " <<  std::endl << FC_DES.toString() << std::endl; 
   std::cout << " FC_DES_LEFT_sensor = "  <<  std::endl << FC_DES_LEFT_sensor.toString() << std::endl; 
   std::cout << " FC_DES_RIGHT_sensor = " <<  std::endl << FC_DES_RIGHT_sensor.toString() << std::endl;  
   std::cout << " norm(FC_DES_sens - FC_sens)  =  "<< std::endl << norm(FC_DES_LEFT_sensor- FC_FILTERED_LEFT_sensor) + norm(FC_DES_RIGHT_sensor- FC_FILTERED_RIGHT_sensor) << std::endl  ; 
   std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl;    
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 

  //-----------------------------------------------------------------------------------------------------
  // FLMM parts to be used
  //     yarp::sig::Matrix Rf_aw = Rf_ext( J_c , S_c , Q_aw_j_cont, Q_aw_s_cont,  U_aw_j_cont,  U_aw_s_cont,  Kc , Kq )  ;
/*
  yarp::sig::Matrix Rf_redu_aw = Rf_redu( J_c,  S_c,  Q_aw_s_cont,  U_aw_s_cont ,  Kc) ;
  yarp::sig::Matrix Rf_aw_filt = filter_SVD( Rf_redu_aw,  1E-4); 

  yarp::sig::Vector d_q_aw_2_5 = -1.0* Pinv_Regularized( Rf_aw_filt, 1E5)* d_fc_des_to_world ;
  yarp::sig::Vector d_q_aw_2_6 = -1.0* Pinv_Regularized( Rf_aw_filt, 1E6)* d_fc_des_to_world ;

  std::cout << " d_q_aw_2_5 = " <<  std::endl << d_q_aw_2_5.toString() << std::endl;
  std::cout << " d_q_aw_2_6 = " <<  std::endl << d_q_aw_2_6.toString() << std::endl;   */
  //-----------------------------------------------------------------------------------------------------

  yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
   
  yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
  yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 

  
//    yarp::sig::Vector d_q_dsp_1 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E1)* d_fc_des_to_world ;
//    yarp::sig::Vector d_q_dsp_2 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E2)* d_fc_des_to_world ;
//    yarp::sig::Vector d_q_dsp_3 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E3)* d_fc_des_to_world ;
//    yarp::sig::Vector d_q_dsp_4 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E4)* d_fc_des_to_world ;
//    yarp::sig::Vector d_q_dsp_5 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E5)* d_fc_des_to_world ;
  yarp::sig::Vector d_q_dsp_6 = -1.0* locoman::utils::Pinv_Regularized( Rf_temp_2_filt, 1E6)* d_fc_des_to_world ;
 // yarp::sig::Vector d_q_dsp_5_m = -1.0* Pinv_Marq( Rf_temp_2_filt, 1E-5)* d_fc_des_to_world ;

  
  
//  yarp::sig::Vector d_q_dsp_7 = -1.0* Pinv_Regularized( Rf_temp_2_filt, 1E7)* d_fc_des_to_world ;
/*   std::cout << " d_q_dsp_1 = "<<  std::endl << d_q_dsp_1.toString()  << std::endl;  
   std::cout << " d_q_dsp_2 = " <<  std::endl << d_q_dsp_2.toString()  << std::endl;   
   std::cout << " d_q_dsp_3 = " <<  std::endl << d_q_dsp_3.toString()  << std::endl;  
   std::cout << " d_q_dsp_4 = " <<  std::endl << d_q_dsp_4.toString()  << std::endl;  
   std::cout << " d_q_dsp_5 = " <<  std::endl << d_q_dsp_5.toString()  << std::endl; */ 
  std::cout << " d_q_dsp_6 = " <<  std::endl << d_q_dsp_6.toString()  << std::endl;  
 //  std::cout << " d_q_dsp_5_m = " <<  std::endl << d_q_dsp_5_m.toString()  << std::endl;  

//  std::cout << " d_q_dsp_7 = " <<  std::endl << d_q_dsp_7.toString()  << std::endl;  

//  std::cout << " norm(d_q_dsp_5) = " <<  std::endl << norm(d_q_dsp_5)  << std::endl;  
  
  std::cout << " norm(d_q_dsp_6) = " <<  std::endl << norm(d_q_dsp_6)  << std::endl;  
//  std::cout << " norm(d_q_dsp_7) = " <<  std::endl << norm(d_q_dsp_7)  << std::endl;   
  

  yarp::sig::Vector d_q_move = d_q_dsp_6 ; // d_q_dsp_5_m   ; // d_q_dsp_6 ; //

  
  
  if(norm(d_q_move)>0.0015){d_q_move =  0.0015 *d_q_move/ norm(d_q_move) ; //d_q_dsp_7 ; //0.012 *d_q_move/ norm(d_q_move) ;
  }
  if(norm(d_q_move)<0.0002){d_q_move =  0.0002 *d_q_move/ norm(d_q_move) ;
  }
  std::cout << " d_q_move = " <<  std::endl << d_q_move.toString()  << std::endl;   
   
  //   
  double err = norm( d_fc_des_to_world )  ;  // d_fc_des_to_world
  double err_min = 25.0 ; //10.0 ;
  double err_max = 700.0 ;  //40.0 ; 
    
  char file_name[] = "err.m";   // writit
  std::ofstream err_cl ( file_name, std::ios::app );
  if( err_cl.is_open() )
  err_cl <<  err << std::endl;  
    
    /*char file_name1[] = "err1.m";
    char temp[] = "temp" ;
    std::ofstream temp ( file_name1, std::ios::app );
    if( &temp.is_open() )
    temp <<  err << std::endl;  */
    
    //---------------------------------------------------- -------------------------------------------

  double alpha = locoman::utils::alpha_filter(err, err_min, err_max) ;
  std::cout << " err = "  << err << std::endl; 
  std::cout << " alpha = "  << alpha << std::endl; 
  
 /* double beta = 1.0 - alpha ;
  double gamma = 6.0 + beta ;
  double delta = pow(10.0, gamma ); 
  std::cout << " delta = "  << delta << std::endl; 
  yarp::sig::Vector d_q_temp_var = -1.0* Pinv_Regularized( Rf_temp_2_filt, delta )* d_fc_des_to_world ; */
 
  //
  q_ref_ToMove = q_motor_side + q_offset +  (0.0/1.0)* alpha*d_q_move ;  //d_q_temp_var ; // alpha*d_q_aw_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        
  q_ref_ToMove = q_current    + q_offset +  (0.0/1.0)* alpha*d_q_move  ;  // on the real robot
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ; 

  //       q_ref_ToMove = moving_right_arm(-0.01);  
  //q_ref_ToMove = q_computed_motor_wrong_tau + q_offset ;
 //------------------------------------------------------------------------------------------------------
    
//   yarp::sig::Vector q_right_arm_ToMove(right_arm_configuration.length()) ;
//   yarp::sig::Vector q_left_arm_ToMove(left_arm_configuration.length()) ;
//   yarp::sig::Vector q_torso_ToMove(torso_configuration.length()) ;
//   yarp::sig::Vector q_right_leg_ToMove(right_leg_configuration.length()) ;
//   yarp::sig::Vector q_left_leg_ToMove(left_leg_configuration.length()) ;
//   
//     robot.fromIdynToRobot(q_ref_ToMove,
//                             q_right_arm_ToMove ,
//                             q_left_arm_ToMove  ,
//                             q_torso_ToMove     ,
//                             q_right_leg_ToMove ,
//                             q_left_leg_ToMove   );
//    q_right_arm_ToMove[0] += 0.005 ;
//   
// //     q_right_arm_des = right_arm_configuration  ; //Insertion of  predefined values (from .ini)                      
// //     q_left_arm_des  = left_arm_configuration   ;
// //     q_torso_des     = torso_configuration      ;
// //     q_left_leg_des  = left_leg_configuration   ;
// //     q_right_leg_des = right_leg_configuration  ;
//     
//     robot.fromRobotToIdyn(  q_right_arm_ToMove ,
//                             q_left_arm_ToMove  ,
//                             q_torso_ToMove     ,
//                             q_right_leg_ToMove ,
//                             q_left_leg_ToMove   ,
//                              q_ref_ToMove            );     // building q_des
//  q_current_open_loop = q_ref_ToMove ;
 
 //------------------------------------------------------------------------------------------------------  
  
  
  
  std::cout << " q_ref_ToMove = "  << q_ref_ToMove.toString() << std::endl; 

  std::cout << "Double Stance Phase "<< std::endl ; 
  robot.move29( q_ref_ToMove);                 //  q_motor_side);//
  


  } // closing of the Double Stance Phase
//----------------------------------------------------------------------------------
else if (last_command =="feet_r_hand" )//|| last_command =="resume" ||            
  {  // Feet and R-Hand Stance Phase  

  locoman::utils::FC_DES_right(FC_DES, mg) ; //  locoman::utils::FC_DES_left(FC_DES, mg) ; // all the weight on the left foot    
   // desired contact force definition  
  FC_DES_LEFT_sensor  = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
  FC_DES_RIGHT_sensor = map_r_fcToSens * FC_DES.subVector(12,23)  ;
  //    
//   FC_DES_RIGHT_HAND_sensor.zero(); ;
//   FC_DES_RIGHT_HAND_sensor[1] = mg ;
  FC_HANDS_DES.zero();
  FC_HANDS_DES[13] = -1.0* mg/4.0 ;// .setSubvector(0 , locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10) * FC_DES_RIGHT_HAND_sensor) ;
  FC_HANDS_DES[16] = -1.0* mg/4.0 ;
  FC_HANDS_DES[19] = -1.0* mg/4.0 ;
  FC_HANDS_DES[22] = -1.0* mg/4.0 ;

  // Partitioning the forces between r-hand and left foot
  
 // desired contact force definition  
  double part_r_hand= 1.0/10.0 ;
  FC_HANDS_DES =  part_r_hand * FC_HANDS_DES ;  
  FC_DES       = (1-part_r_hand)* FC_DES ;
  // desired contact force variation
  d_fc_des_to_world  = -1.0*(FC_DES - FC_FILTERED) ; // -fc_to_world_0 ;
  
  yarp::sig::Vector d_fc_r_hand  = FC_HANDS_DES.subVector(12,23) -  FC_HANDS_FILTERED.subVector(12,23) ;
  yarp::sig::Vector d_fc_des_f_r_hand(size_fc+size_fc/2,0.0) ;
  d_fc_des_f_r_hand.setSubvector(0,d_fc_des_to_world) ;
  d_fc_des_f_r_hand.setSubvector(size_fc ,   d_fc_r_hand) ;
  //  std::cout << " size_fc = " << size_fc <<  std::endl ;

  std::cout << " d_fc_des_to_world = " << d_fc_des_to_world.toString() <<  std::endl ;
  
  //FC_HANDS_FILTERED
  //
  //-----------------------------------------------------------------------------------------------------------------     
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " FC_DES_feet = " << std::endl << FC_DES.toString() << std::endl; 
   std::cout << " FC_FILTERED_feet = " << std::endl << FC_FILTERED.toString() << std::endl; 
   std::cout << " d_fc_des_to_world = " <<  std::endl << d_fc_des_to_world.toString() << std::endl;  
   std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl;    
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " FC_R_HAND_DES = " <<  std::endl << FC_HANDS_DES.subVector(12,23).toString() << std::endl;    
   std::cout << " FC_R_HAND_FILTERED = " <<  std::endl << FC_HANDS_FILTERED.subVector(12,23) .toString() << std::endl;    
   std::cout << " d_fc_r_hand = " <<  std::endl << d_fc_r_hand.toString() << std::endl;  
   std::cout << " norm( d_fc_r_hand ) = " <<  std::endl << norm( d_fc_r_hand ) << std::endl;    
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " d_fc_des_f_r_hand = " <<  std::endl << d_fc_des_f_r_hand.toString() << std::endl;  
   std::cout << " norm( d_fc_des_f_r_hand ) = " <<  std::endl << norm( d_fc_des_f_r_hand ) << std::endl;       
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
  //-----------------------------------------------------------------------------------------------------
   
  yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c_f_rh, S_c_f_rh, Q_aw_s_c_f_rh, U_aw_s_c_f_rh, Kc_f_rh ) ;
  yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
     
  yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc+size_fc/2-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
  yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 

  yarp::sig::Vector weights(size_fc+size_fc/2,1.0) ;
  double weight_hand = 1.0 ;
  weights.setSubvector(size_fc, weight_hand*weights.subVector(size_fc, size_fc+size_fc/2-1 )   ) ;
  std::cout << " weights = " <<  std::endl << weights.toString()  << std::endl;  

  yarp::sig::Matrix W_f_r_hand(size_fc+size_fc/2, size_fc+size_fc/2);
  W_f_r_hand.zero();
  W_f_r_hand.diagonal(weights) ;
  W_f_r_hand = W_f_r_hand/weight_hand ; 

  yarp::sig::Vector d_q_dsp_6  = -1.0*  locoman::utils::Pinv_Regularized( Rf_temp_2_filt, 1E2)*W_f_r_hand* d_fc_des_f_r_hand ; //TODO?????
  std::cout << " d_q_dsp_6 = " <<  std::endl << d_q_dsp_6.toString()  << std::endl;  
 
  std::cout << " norm(d_q_dsp_6) = " <<  std::endl << norm(d_q_dsp_6)  << std::endl;  

  yarp::sig::Vector d_q_move = d_q_dsp_6 ; // d_q_dsp_5_m   ; // d_q_dsp_6 ; //
  
  if(norm(d_q_move)>0.0015){d_q_move = 0.0015 *d_q_move/ norm(d_q_move) ; //d_q_dsp_7 ; //0.012 *d_q_move/ norm(d_q_move) ;
  }
  if(norm(d_q_move)<0.0002){d_q_move = 0.0002 *d_q_move/ norm(d_q_move)  ;
  }
  std::cout << " d_q_move = " <<  std::endl << d_q_move.toString()  << std::endl;   
   
  //   
  double err = norm( d_fc_des_f_r_hand )  ;  // d_fc_des_to_world
  double err_min = 25.0 ; //10.0 ;
  double err_max = 700.0 ;  //40.0 ; 
    
  char file_name[] = "err.m";   // writit
  std::ofstream err_cl ( file_name, std::ios::app );
  if( err_cl.is_open() )
  err_cl <<  err << std::endl;  
    
    /*char file_name1[] = "err1.m";
    char temp[] = "temp" ;
    std::ofstream temp ( file_name1, std::ios::app );
    if( &temp.is_open() )
    temp <<  err << std::endl;  */
    
    //---------------------------------------------------- -------------------------------------------

  double alpha = locoman::utils::alpha_filter(err, err_min, err_max) ;
  std::cout << " err = "  << err << std::endl; 
  std::cout << " alpha = "  << alpha << std::endl; 
  
  q_ref_ToMove = q_motor_side + q_offset +  (0.0/1.0)* alpha*d_q_move ;  //d_q_temp_var ; // alpha*d_q_aw_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        
  q_ref_ToMove = q_current    + q_offset +  (0.0/1.0)* alpha*d_q_move  ;  // on the real robot
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ; 
 
  std::cout << " q_ref_ToMove = "  << q_ref_ToMove.toString() << std::endl; 

  std::cout << "Feet and R-hand Stance Phase "<< std::endl ; 
  robot.move29( q_ref_ToMove);                 //  q_motor_side);//
  } // End of Feet + R-Hand Phase

  
  
//------------------------------------------------------------------------------------
else if (last_command == "sw_rg_up" || last_command == "sw_rg_fw" || last_command == "sw_rg_dw" || 
         last_command == "sw_lf_up" || last_command == "sw_lf_fw" || last_command == "sw_lf_dw"  )
/*	 last_command == "com_up"   || last_command == "com_dw"   ||
	 last_command =="r_hand_z_world_fw" || last_command =="r_hand_z_world_bk" || 
	 last_command =="r_hand_x_world_fw" || last_command =="r_hand_x_world_bk" ||
	 last_command =="r_hand_z_world_rot_ccw" ||
	 last_command =="r_hand_z_world_rot_cw"  ||   
	 last_command =="r_hand_x_world_rot_ccw" ||
	 last_command =="r_hand_x_world_rot_cw" */  
   { // Single Stance Phase  
  
  if (last_command =="sw_rg_up")
     {
  std::cout << "swinging the right foot up"<< std::endl ; //FC_DES_right() ;  // here we should place the desired final configuration of the right foot

  T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  
  yarp::sig::Matrix D_T_r_up = T_l1_r1_up - T_l_c1_r_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_l1_r1_up(3, 0.0);
  d_l1_r1_up[0] = D_T_r_up[0][3] ;
  d_l1_r1_up[1] = D_T_r_up[1][3];
  d_l1_r1_up[2] = D_T_r_up[2][3]; // toward the desired displacement
  if(norm( d_l1_r1_up)>0.001){ d_l1_r1_up = 0.001*d_l1_r1_up/(norm( d_l1_r1_up)) ;  }
  if(norm( d_l1_r1_up)<0.0008){ d_l1_r1_up = 0.0*d_l1_r1_up  ;  }  // desired displacement
  
  yarp::sig::Vector e_o = locoman::utils::Orient_Error( locoman::utils::getRot(T_l1_r1_up ) , locoman::utils::getRot(T_l_c1_r_c1_loop ) ) ; 
  if(norm( e_o)>0.01){ e_o = 0.01*e_o/(norm( e_o)) ; }
  if(norm( e_o)<0.001){ e_o = 0.0*e_o  ; }     // desired orientation variation
  
  d_EE_r_des.setSubvector(0, (1.0/1.0)*d_l1_r1_up ) ;  
  d_EE_r_des.setSubvector(3, (1.0/1.0)*e_o ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_r_des  = " <<  std::endl << d_EE_r_des.toString()   << std::endl;   

  //-----------------------------------------------------------------------------------------------------------------
  // Here I consider an absolute frame with origin in {l1} and orientation parallel to {AW}
  // {AW_l1} => fixed and with z vertical
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_r_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_l_c1_r_c1_loop), zero_3) )* J_r_c1_body_0 ;
  
  yarp::sig::Matrix J_sw_rg_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_rg_up.setSubmatrix(J_com_aw,0,0) ;
  J_sw_rg_up.setSubmatrix(J_l_c1_body_0,3,0) ;
  J_sw_rg_up.setSubmatrix(J_r_c1_aw,9,0) ;
   
  yarp::sig::Vector Task_sw_rg_up(15,0.0) ;   
  Task_sw_rg_up.setSubvector( 9 , (1.0/1.0)* d_EE_r_des  ) ;
 // std::cout << " Task_sw_rg_up = " <<  std::endl << Task_sw_rg_up.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_rg_up = locoman::utils::Pinv_trunc_SVD(J_sw_rg_up)* Task_sw_rg_up ;     
  yarp::sig::Vector d_q_sw_rg_up = d_u_q_sw_rg_up.subVector( 6, d_u_q_sw_rg_up.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_rg_up  ;//  +   (1.0/1.0) *  nullspaceProjection(J_com_l1)*d_q_task  ; // + (100.0/1.0)*d_q_ssp_l_6;// + d_q_ssp_6 ; //d_q_dsp_5_left ;// +  (1.0/1.0)*d_q_temp ; // d_q_sw_rg_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        

  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_rg_up  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_rg_up  ; 
  robot.move29( q_ref_ToMove );           
     }
     
  else if (last_command =="sw_rg_fw")
     {
  std::cout << "swinging the right foot forward"<< std::endl ;
      
  T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  
  yarp::sig::Matrix D_T_r_fw = T_l1_r1_fw - T_l_c1_r_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_l1_r1_fw(3, 0.0);
  d_l1_r1_fw[0] = D_T_r_fw[0][3] ;
  d_l1_r1_fw[1] = D_T_r_fw[1][3];
  d_l1_r1_fw[2] = D_T_r_fw[2][3]; // toward the desired displacement
  if(norm( d_l1_r1_fw)>0.001){ d_l1_r1_fw = 0.001*d_l1_r1_fw/(norm( d_l1_r1_fw)) ;  }
  if(norm( d_l1_r1_fw)<0.0008){ d_l1_r1_fw = 0.0*d_l1_r1_fw  ;  }  // desired displacement
  
  yarp::sig::Vector e_o = locoman::utils::Orient_Error( locoman::utils::getRot(T_l1_r1_fw ) , locoman::utils::getRot(T_l_c1_r_c1_loop ) ) ; 
  if(norm( e_o)>0.01){ e_o = 0.01*e_o/(norm( e_o)) ; }
  if(norm( e_o)<0.001){ e_o = 0.0*e_o  ; }     // desired orientation variation
  
  d_EE_r_des.setSubvector(0, (1.0/1.0)*d_l1_r1_fw ) ;  
  d_EE_r_des.setSubvector(3, (1.0/1.0)*e_o ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_r_des  = " <<  std::endl << d_EE_r_des.toString()   << std::endl;   

  //-----------------------------------------------------------------------------------------------------------------
  // Here I consider an absolute frame with origin in {l1} and orientation parallel to {AW}
  // {AW_l1} => fixed and with z vertical
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_r_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_l_c1_r_c1_loop), zero_3) )* J_r_c1_body_0 ;
  
  yarp::sig::Matrix J_sw_rg_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_rg_up.setSubmatrix(J_com_aw,0,0) ;
  J_sw_rg_up.setSubmatrix(J_l_c1_body_0,3,0) ;
  J_sw_rg_up.setSubmatrix(J_r_c1_aw,9,0) ;
   
  yarp::sig::Vector Task_sw_rg_fw(15,0.0) ;   
  Task_sw_rg_fw.setSubvector( 9 , (1.0/1.0)* d_EE_r_des  ) ;
 // std::cout << " Task_sw_rg_fw = " <<  std::endl << Task_sw_rg_fw.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_rg_fw = locoman::utils::Pinv_trunc_SVD(J_sw_rg_up)* Task_sw_rg_fw ;     
  yarp::sig::Vector d_q_sw_rg_fw = d_u_q_sw_rg_fw.subVector( 6, d_u_q_sw_rg_fw.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_rg_fw  ;   //  +   (1.0/1.0) *  nullspaceProjection(J_com_l1)*d_q_task  ; // + (100.0/1.0)*d_q_ssp_l_6;// + d_q_ssp_6 ; //d_q_dsp_5_left ;// +  (1.0/1.0)*d_q_temp ; // d_q_sw_rg_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        

  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_rg_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_rg_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }
     
  else if (last_command =="sw_rg_dw")
     {
  std::cout << "swinging the right foot down"<< std::endl ;
  
  T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  
  yarp::sig::Matrix D_T_r_dw = T_l1_r1_dw - T_l_c1_r_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_l1_r1_dw(3, 0.0);
  d_l1_r1_dw[0] = D_T_r_dw[0][3] ;
  d_l1_r1_dw[1] = D_T_r_dw[1][3] ;
  d_l1_r1_dw[2] = D_T_r_dw[2][3] ; // toward the desired displacement
  if(norm( d_l1_r1_dw)>0.001){ d_l1_r1_dw = 0.001*d_l1_r1_dw/(norm( d_l1_r1_dw)) ;  }
  if(norm( d_l1_r1_dw)<0.0008){ d_l1_r1_dw = 0.0*d_l1_r1_dw  ;  }  // desired displacement
  if(FC_FILTERED_RIGHT_sensor[2]<-5.0){ d_l1_r1_dw = 0.0*d_l1_r1_dw  ;  }
  
  
  yarp::sig::Vector e_o = locoman::utils::Orient_Error( locoman::utils::getRot(T_l1_r1_dw ) , locoman::utils::getRot(T_l_c1_r_c1_loop ) ) ; 
  if(norm( e_o)>0.01){ e_o = 0.01*e_o/(norm( e_o)) ; }
  if(norm( e_o)<0.001){ e_o = 0.0*e_o  ; }     // desired orientation variation
  
  d_EE_r_des.setSubvector(0, (1.0/1.0)*d_l1_r1_dw ) ;  
  d_EE_r_des.setSubvector(3, (1.0/1.0)*e_o ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_r_des  = " <<  std::endl << d_EE_r_des.toString()   << std::endl;   

  //-----------------------------------------------------------------------------------------------------------------
  // Here I consider an absolute frame with origin in {l1} and orientation parallel to {AW}
  // {AW_l1} => fixed and with z vertical
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw  = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_r_c1_aw = locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_l_c1_r_c1_loop), zero_3) )* J_r_c1_body_0 ;
  
  yarp::sig::Matrix J_sw_rg_dw( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_rg_dw.setSubmatrix(J_com_aw,0,0) ;
  J_sw_rg_dw.setSubmatrix(J_l_c1_body_0,3,0) ;
  J_sw_rg_dw.setSubmatrix(J_r_c1_aw,9,0) ;
   
  yarp::sig::Vector Task_sw_rg_dw(15,0.0) ;   
  Task_sw_rg_dw.setSubvector( 9 , (1.0/1.0)* d_EE_r_des  ) ;
 // std::cout << " Task_sw_rg_up = " <<  std::endl << Task_sw_rg_up.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_rg_dw = locoman::utils::Pinv_trunc_SVD(J_sw_rg_dw)* Task_sw_rg_dw ;     
  yarp::sig::Vector d_q_sw_rg_dw = d_u_q_sw_rg_dw.subVector( 6, d_u_q_sw_rg_dw.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_rg_dw  ;//  +   (1.0/1.0) *  nullspaceProjection(J_com_l1)*d_q_task  ; // + (100.0/1.0)*d_q_ssp_l_6;// + d_q_ssp_6 ; //d_q_dsp_5_left ;// +  (1.0/1.0)*d_q_temp ; // d_q_sw_rg_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        

  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_rg_dw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_rg_dw  ; 
  robot.move29( q_ref_ToMove );   
  
     }
     
     
  else if (last_command =="sw_lf_up")
     {
  std::cout << "swinging the left foot up"<< std::endl ; 

  T_r_c1_l_c1_loop = (locoman::utils::iHomogeneous(T_aw_r_c1_0 )*   T_aw_l_c1_0) ; // where l1 is with respect to r1
  yarp::sig::Matrix D_T_l_up = T_r1_l1_up - T_r_c1_l_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_r1_l1_up(3, 0.0);
  d_r1_l1_up[0] = D_T_l_up[0][3] ;
  d_r1_l1_up[1] = D_T_l_up[1][3];
  d_r1_l1_up[2] = D_T_l_up[2][3]; // toward the desired displacement
  if(norm( d_r1_l1_up)>0.001){ d_r1_l1_up = 0.001*d_r1_l1_up/(norm( d_r1_l1_up)) ;  }
  if(norm( d_r1_l1_up)<0.0008){ d_r1_l1_up = 0.0*d_r1_l1_up  ;  }  // desired displacement
  yarp::sig::Vector e_o_l_up = locoman::utils::Orient_Error( locoman::utils::getRot(T_r1_l1_up ) , locoman::utils::getRot(T_r_c1_l_c1_loop ) ) ; 
  if(norm( e_o_l_up)>0.01){ e_o_l_up = 0.01*e_o_l_up/(norm( e_o_l_up)) ; }
  if(norm( e_o_l_up)<0.001){ e_o_l_up = 0.0*e_o_l_up  ; }     // desired orientation variation
  d_EE_l_des.setSubvector(0, (1.0/1.0)*d_r1_l1_up ) ;  
  d_EE_l_des.setSubvector(3, (1.0/1.0)*e_o_l_up ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw  = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_l_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_r_c1_l_c1_loop), zero_3) )* J_l_c1_body_0 ;
  yarp::sig::Matrix J_sw_lf_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_lf_up.setSubmatrix( J_com_aw      , 0,0) ;
  J_sw_lf_up.setSubmatrix( J_r_c1_body_0 , 3,0) ;
  J_sw_lf_up.setSubmatrix( J_l_c1_aw     , 9,0) ;   
  yarp::sig::Vector Task_sw_lf_up(15,0.0) ;   
  Task_sw_lf_up.setSubvector( 9 , (1.0/1.0)* d_EE_l_des  ) ;
 // std::cout << " Task_sw_lf_up = " <<  std::endl << Task_sw_lf_up.toString()  << std::endl;   
  yarp::sig::Vector d_u_q_sw_lf_up = locoman::utils::Pinv_trunc_SVD(J_sw_lf_up)* Task_sw_lf_up ;     
  yarp::sig::Vector d_q_sw_lf_up = d_u_q_sw_lf_up.subVector( 6, d_u_q_sw_lf_up.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_lf_up  ;  

  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_lf_up  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_lf_up  ; 
  robot.move29( q_ref_ToMove );           
     }
     
else if (last_command =="sw_lf_fw")
     {
  std::cout << "swinging the left foot forward"<< std::endl ;
      
  T_r_c1_l_c1_loop = (locoman::utils::iHomogeneous(T_aw_r_c1_0 )*  T_aw_l_c1_0) ; // where r1 is with respect to l1
  yarp::sig::Matrix D_T_l_fw = T_r1_l1_fw - T_r_c1_l_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_r1_l1_fw(3, 0.0);
  d_r1_l1_fw[0] = D_T_l_fw[0][3] ;
  d_r1_l1_fw[1] = D_T_l_fw[1][3];
  d_r1_l1_fw[2] = D_T_l_fw[2][3]; // toward the desired displacement
  if(norm( d_r1_l1_fw)>0.001 ){ d_r1_l1_fw = 0.001*d_r1_l1_fw/(norm( d_r1_l1_fw)) ;  }
  if(norm( d_r1_l1_fw)<0.0008){ d_r1_l1_fw = 0.0*d_r1_l1_fw  ;  }  // desired displacement
  yarp::sig::Vector e_o_l_fw = locoman::utils::Orient_Error( locoman::utils::getRot(T_r1_l1_fw ) , locoman::utils::getRot( T_r_c1_l_c1_loop ) ) ; 
  if(norm( e_o_l_fw)>0.01){ e_o_l_fw = 0.01*e_o_l_fw/(norm( e_o_l_fw )) ; }
  if(norm( e_o_l_fw)<0.001){ e_o_l_fw = 0.0*e_o_l_fw  ; }     // desired orientation variation
  d_EE_l_des.setSubvector(0, (1.0/1.0)*d_r1_l1_fw ) ;  
  d_EE_l_des.setSubvector(3, (1.0/1.0)*e_o_l_fw ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw  = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_l_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_r_c1_l_c1_loop), zero_3) )* J_l_c1_body_0 ;
  yarp::sig::Matrix J_sw_lf_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_lf_up.setSubmatrix(J_com_aw,0,0) ;
  J_sw_lf_up.setSubmatrix(J_r_c1_body_0,3,0) ;
  J_sw_lf_up.setSubmatrix(J_l_c1_aw,9,0) ;
  yarp::sig::Vector Task_sw_lf_fw(15,0.0) ;   
  Task_sw_lf_fw.setSubvector( 9 , (1.0/1.0)* d_EE_l_des  ) ;
 // std::cout << " Task_sw_lf_fw = " <<  std::endl << Task_sw_lf_fw.toString()  << std::endl;   
  yarp::sig::Vector d_u_q_sw_lf_fw = locoman::utils::Pinv_trunc_SVD(J_sw_lf_up)* Task_sw_lf_fw ;     
  yarp::sig::Vector d_q_sw_lf_fw = d_u_q_sw_lf_fw.subVector( 6, d_u_q_sw_lf_fw.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_lf_fw  ;  

  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_lf_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_lf_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     

   else if (last_command =="sw_lf_dw")
     {
  std::cout << "swinging the left foot down"<< std::endl ;
  
  T_r_c1_l_c1_loop = (locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0) ; // where r1 is with respect to l1
  
  yarp::sig::Matrix D_T_l_dw = T_r1_l1_dw - T_r_c1_l_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_r1_l1_dw(3, 0.0);
  d_r1_l1_dw[0] = D_T_l_dw[0][3] ;
  d_r1_l1_dw[1] = D_T_l_dw[1][3] ;
  d_r1_l1_dw[2] = D_T_l_dw[2][3] ; // toward the desired displacement
  if(norm( d_r1_l1_dw)>0.001){ d_r1_l1_dw = 0.001*d_r1_l1_dw/(norm( d_r1_l1_dw)) ;  }
  if(norm( d_r1_l1_dw)<0.0008){ d_r1_l1_dw = 0.0*d_r1_l1_dw  ;  }  // desired displacement
  if(FC_FILTERED_LEFT_sensor[2]<-5.0){ d_r1_l1_dw = 0.0*d_r1_l1_dw  ;  }
  yarp::sig::Vector e_o_l_dw = locoman::utils::Orient_Error( locoman::utils::getRot(T_r1_l1_dw ) , locoman::utils::getRot(T_r_c1_l_c1_loop ) ) ; 
  if(norm( e_o_l_dw)>0.01){ e_o_l_dw = 0.01*e_o_l_dw/(norm( e_o_l_dw )) ; }
  if(norm( e_o_l_dw)<0.001){ e_o_l_dw = 0.0*e_o_l_dw  ; }     // desired orientation variation
  
  d_EE_l_des.setSubvector(0, (1.0/1.0)*d_r1_l1_dw ) ;  
  d_EE_l_des.setSubvector(3, (1.0/1.0)*e_o_l_dw ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   

  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw  = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu; 
  J_l_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_r_c1_l_c1_loop), zero_3) )* J_l_c1_body_0 ;
  
  yarp::sig::Matrix J_sw_lf_dw( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_lf_dw.setSubmatrix(J_com_aw,0,0) ;
  J_sw_lf_dw.setSubmatrix(J_r_c1_body_0,3,0) ;
  J_sw_lf_dw.setSubmatrix(J_l_c1_aw,9,0) ;
   
  yarp::sig::Vector Task_sw_lf_dw(15,0.0) ;   
  Task_sw_lf_dw.setSubvector( 9 , (1.0/1.0)* d_EE_l_des  ) ;
 // std::cout << " Task_sw_lf_dw = " <<  std::endl << Task_sw_lf_dw.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_lf_dw = locoman::utils::Pinv_trunc_SVD(J_sw_lf_dw)* Task_sw_lf_dw ;     
  yarp::sig::Vector d_q_sw_lf_dw = d_u_q_sw_lf_dw.subVector( 6, d_u_q_sw_lf_dw.length()-1) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_lf_dw  ; 

  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_sw_lf_dw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_lf_dw  ; 
  robot.move29( q_ref_ToMove );   
  
     }  
     
   } // closing single stance phase 
  else   {  //  General Command part
  if (last_command == "com_up")
  { 
  std::cout << "Moving CoM up"<< std::endl ;
  
  // CoM error
  yarp::sig::Vector CoM_w_loop = model.iDyn3_model.getCOM()  ; 
  yarp::sig::Vector d_CoM_up = CoM_w_up - CoM_w_loop ; 
  if(norm( d_CoM_up)>0.001 ) { d_CoM_up = 0.001*d_CoM_up/(norm( d_CoM_up)) ;  }
  if(norm( d_CoM_up)<0.0008) { d_CoM_up = 0.0*d_CoM_up  ;  }            // desired displacement
  // L1 error (posistion and orientation)
  yarp::sig::Vector D_l1_com_up =locoman::utils::getTrasl( T_w_l1_cmd - T_w_l_c1_0) ;
  if(norm( D_l1_com_up)>0.002 ) { D_l1_com_up = 0.002*D_l1_com_up/(norm( D_l1_com_up)) ;  }
  if(norm( D_l1_com_up)<0.0008) { D_l1_com_up = 0.0*D_l1_com_up  ;  }  // desired displacement
  yarp::sig::Vector e_o_l1_com_up = locoman::utils::Orient_Error( locoman::utils::getRot(T_w_l1_cmd ) , locoman::utils::getRot( T_w_l_c1_0 ) ) ; 
  if(norm( e_o_l1_com_up)>0.01 ) { e_o_l1_com_up = 0.01*e_o_l1_com_up/(norm( e_o_l1_com_up )) ; }
  if(norm( e_o_l1_com_up)<0.001) { e_o_l1_com_up = 0.0*e_o_l1_com_up  ; }     // desired orientation variation
  // R1 error (posistion and orientation)
  yarp::sig::Vector D_r1_com_up =locoman::utils::getTrasl( T_w_r1_cmd - T_w_r_c1_0) ;
  if(norm( D_r1_com_up)>0.002 ) { D_r1_com_up = 0.002*D_r1_com_up/(norm( D_r1_com_up)) ;  }
  if(norm( D_r1_com_up)<0.0008) { D_r1_com_up = 0.0*D_r1_com_up  ;  }  // desired displacement
  yarp::sig::Vector e_o_r1_com_up = locoman::utils::Orient_Error( locoman::utils::getRot(T_w_r1_cmd ) , locoman::utils::getRot( T_w_r_c1_0 ) ) ; 
  if(norm( e_o_r1_com_up)>0.01 ) { e_o_r1_com_up = 0.01*e_o_r1_com_up/(norm( e_o_r1_com_up )) ; }
  if(norm( e_o_r1_com_up)<0.001) { e_o_r1_com_up = 0.0*e_o_r1_com_up  ; }     // desired orientation variation
  //---------------------------------------------------------------------------------------------------
  // Task Error Vector
  yarp::sig::Vector Task_com_up(15,0.0) ;   
  Task_com_up.setSubvector( 0 , (1.0/1.0)* d_CoM_up       ) ;
  Task_com_up.setSubvector( 3 , (1.0/1.0)* D_l1_com_up    ) ;
  Task_com_up.setSubvector( 6 , (1.0/1.0)* e_o_l1_com_up  ) ;
  Task_com_up.setSubvector( 9 , (1.0/1.0)* D_r1_com_up    ) ;
  Task_com_up.setSubvector( 12, (1.0/1.0)* e_o_r1_com_up  ) ;  
  std::cout << " Task_com_up = " <<  std::endl << Task_com_up.toString()  << std::endl;   
  //-----------------------------------------------------------------------------------------------------------------
  // Jacobian
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  // J_l_c1_mix_0
  // J_r_c1_mix_0
  yarp::sig::Matrix J_com_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ) );
  J_com_up.setSubmatrix( J_com_w_redu , 0 , 0 ) ;
  J_com_up.setSubmatrix( J_l_c1_mix_0 , 3 , 0 ) ;
  J_com_up.setSubmatrix( J_r_c1_mix_0 , 9 , 0 ) ;
//--------------------------------------------------------------------------------------------    
  yarp::sig::Vector d_u_q_com_up = locoman::utils::Pinv_trunc_SVD( J_com_up )* Task_com_up ;     
  yarp::sig::Vector d_q_com_up   = d_u_q_com_up.subVector( 6, d_u_q_com_up.length()-1 ) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_com_up  ;// 
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_com_up  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_com_up  ; 
  robot.move29( q_ref_ToMove );   
  }
  else if (last_command == "com_dw")
  { 
  std::cout << "Moving CoM dw"<< std::endl ; //
  
  // CoM error
  yarp::sig::Vector CoM_w_loop = model.iDyn3_model.getCOM()  ; 
  yarp::sig::Vector d_CoM_dw = CoM_w_dw - CoM_w_loop ; 
  if(norm( d_CoM_dw)>0.001 ) { d_CoM_dw = 0.001*d_CoM_dw/(norm( d_CoM_dw)) ;  }
  if(norm( d_CoM_dw)<0.0008) { d_CoM_dw = 0.0*d_CoM_dw  ;  }            // desired displacement
  // L1 error (posistion and orientation)
  yarp::sig::Vector D_l1_com_dw =locoman::utils::getTrasl( T_w_l1_cmd - T_w_l_c1_0) ;
  if(norm( D_l1_com_dw)>0.002 ) { D_l1_com_dw = 0.002*D_l1_com_dw/(norm( D_l1_com_dw)) ;  }
  if(norm( D_l1_com_dw)<0.0008) { D_l1_com_dw = 0.0*D_l1_com_dw  ;  }  // desired displacement
  yarp::sig::Vector e_o_l1_com_dw = locoman::utils::Orient_Error( locoman::utils::getRot(T_w_l1_cmd ) , locoman::utils::getRot( T_w_l_c1_0 ) ) ; 
  if(norm( e_o_l1_com_dw)>0.01 ) { e_o_l1_com_dw = 0.01*e_o_l1_com_dw/(norm( e_o_l1_com_dw )) ; }
  if(norm( e_o_l1_com_dw)<0.001) { e_o_l1_com_dw = 0.0*e_o_l1_com_dw  ; }     // desired orientation variation
  // R1 error (posistion and orientation)
  yarp::sig::Vector D_r1_com_dw =locoman::utils::getTrasl( T_w_r1_cmd - T_w_r_c1_0) ;
  if(norm( D_r1_com_dw)>0.002 ) { D_r1_com_dw = 0.002*D_r1_com_dw/(norm( D_r1_com_dw)) ;  }
  if(norm( D_r1_com_dw)<0.0008) { D_r1_com_dw = 0.0*D_r1_com_dw  ;  }  // desired displacement
  yarp::sig::Vector e_o_r1_com_dw = locoman::utils::Orient_Error( locoman::utils::getRot(T_w_r1_cmd ) , locoman::utils::getRot( T_w_r_c1_0 ) ) ; 
  if(norm( e_o_r1_com_dw)>0.01 ) { e_o_r1_com_dw = 0.01*e_o_r1_com_dw/(norm( e_o_r1_com_dw )) ; }
  if(norm( e_o_r1_com_dw)<0.001) { e_o_r1_com_dw = 0.0*e_o_r1_com_dw  ; }     // desired orientation variation
  //---------------------------------------------------------------------------------------------------
  // Task Error Vector
  yarp::sig::Vector Task_com_up(15,0.0) ;   
  Task_com_up.setSubvector( 0 , (1.0/1.0)* d_CoM_dw       ) ;
  Task_com_up.setSubvector( 3 , (1.0/1.0)* D_l1_com_dw    ) ;
  Task_com_up.setSubvector( 6 , (1.0/1.0)* e_o_l1_com_dw  ) ;
  Task_com_up.setSubvector( 9 , (1.0/1.0)* D_r1_com_dw    ) ;
  Task_com_up.setSubvector( 12, (1.0/1.0)* e_o_r1_com_dw  ) ;  
  std::cout << " Task_com_up = " <<  std::endl << Task_com_up.toString()  << std::endl;   
  //-----------------------------------------------------------------------------------------------------------------
  // Jacobian
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  // J_l_c1_mix_0
  // J_r_c1_mix_0
  yarp::sig::Matrix J_com_dw( 15 , ( robot.getNumberOfKinematicJoints() + 6 ) );
  J_com_dw.setSubmatrix( J_com_w_redu , 0 , 0 ) ;
  J_com_dw.setSubmatrix( J_l_c1_mix_0 , 3 , 0 ) ;
  J_com_dw.setSubmatrix( J_r_c1_mix_0 , 9 , 0 ) ;
//--------------------------------------------------------------------------------------------    
  yarp::sig::Vector d_u_q_com_dw = locoman::utils::Pinv_trunc_SVD( J_com_dw )* Task_com_up ;     
  yarp::sig::Vector d_q_com_dw   = d_u_q_com_dw.subVector( 6, d_u_q_com_dw.length()-1 ) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_com_dw  ;// 
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_com_dw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_com_dw  ; 
  robot.move29( q_ref_ToMove );   
  } 
  
  else if (last_command =="r_hand_z_world_fw")
     {
  std::cout << "R Hand, Z World, moving forward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[2] += 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
  //yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;
  
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;
  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  yarp::sig::Vector d_temp = T_temp_2.getCol(2) ;
  d_temp = -1.0*d_temp.subVector(0,2) ;  // TODO : control the correct direction from aw and the imu
  d_temp = 0.001* d_temp/norm(d_temp) ;
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;
  
 // ------------------------------------------------------------------
  // Other frames for closed loop control
//   yarp::sig::Matrix T_l_hand_des_local  = locoman::utils::iHomogeneous( T_waist_l_hand_0  )* T_waist_l_hand_cmd ;
//   yarp::sig::Matrix T_l1_foot_des_local = locoman::utils::iHomogeneous( T_waist_l1_foot_0 )* T_waist_l1_foot_cmd;
//   yarp::sig::Matrix T_r1_foot_des_local = locoman::utils::iHomogeneous( T_waist_r1_foot_0 )* T_waist_r1_foot_cmd;
// 
//        std::cout << " T_waist_l_hand_cmd = " <<  std::endl << T_waist_l_hand_cmd.toString() << std::endl;
//        std::cout << " T_l_hand_des_local = " <<  std::endl << T_l_hand_des_local.toString() << std::endl;
//   
//        std::cout << " T_waist_r1_foot_cmd = " <<  std::endl << T_waist_r1_foot_cmd.toString() << std::endl;
//        std::cout << " T_r1_foot_des_local = " <<  std::endl << T_r1_foot_des_local.toString() << std::endl;
//   

  
   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,                            //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0,
                                                                J_r_hand_body_0, 
								J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
								J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
      else if (last_command =="r_hand_z_world_bk")
     {
  std::cout << "R Hand, Z World, moving backward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[2] -= 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
//   yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;

  yarp::sig::Matrix T_r_hand_des_local(4,4) ;

//   T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, locoman::utils::getTrasl(T_r_hand_des_local)) ;
//   yarp::sig::Matrix T_temp =   T_aw_w_0*T_w_r_hand_0  ;
//   T_temp[3][2] -= 0.01 ;
//   T_temp =  T_w_aw_0*T_temp ;
//   yarp::sig::Vector d_trasl(3,0.0) ;
//   d_trasl[2] = -0.001 ;
//   T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_trasl) ; // locoman::utils::getTrasl(T_temp)) ;
  
  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  yarp::sig::Vector d_temp = T_temp_2.getCol(2) ;
  d_temp = -1.0*d_temp.subVector(0,2) ;   // TODO : control the correct direction from aw and the imu
  d_temp = -0.001* d_temp/norm(d_temp) ;
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 

  yarp::sig::Vector zer_3(3,0.0) ;  
   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
     }  
     
     else if (last_command =="r_hand_x_world_fw")
     {
  std::cout << "R Hand, X World, moving forward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[0] += 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
//   yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;
   
  //---------------------------------------------------------------------------------------------------------------
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  yarp::sig::Vector d_temp = T_temp_2.getCol(0) ;
  d_temp = d_temp.subVector(0,2) ;  //
  d_temp = 0.001* d_temp/norm(d_temp) ;
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;
  
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
  else if (last_command =="r_hand_x_world_bk")
     {
  std::cout << "R Hand, X World, moving backward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[0] -= 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
//   yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;
    //---------------------------------------------------------------------------------------------------------------
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  yarp::sig::Vector d_temp = T_temp_2.getCol(0) ;
  d_temp = d_temp.subVector(0,2) ;  // 
  d_temp = -0.001* d_temp/norm(d_temp) ;
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;
  
  
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }  
       else if (last_command =="r_hand_x_local_fw")
     {
  std::cout << "R Hand, X Local, moving forward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[0] += 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
//   yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;
   
  //---------------------------------------------------------------------------------------------------------------
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  T_temp_2.eye(); 
  yarp::sig::Vector d_temp = T_temp_2.getCol(0) ;
  d_temp = d_temp.subVector(0,2) ;  //
  d_temp = 0.001* d_temp/norm(d_temp) ;
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;
  
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0, J_r_hand_body_0, 
                                                                J_l_c1_body_0, J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
  else if (last_command =="r_hand_x_local_bk")
     {
  std::cout << "R Hand, X Local, moving backward"<< std::endl ;
    //---------------------------------------------------------------------------------------------------------------
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  yarp::sig::Matrix T_temp_2 = T_r_hand_w_0 * T_w_aw_0 ; 
  T_temp_2.eye() ;
  yarp::sig::Vector d_temp = T_temp_2.getCol(0) ;
  d_temp = d_temp.subVector(0,2) ;  // 
  d_temp = -0.001* d_temp/norm(d_temp) ;
  T_r_hand_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;
  
  
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0, J_r_hand_body_0, 
                                                                J_l_c1_body_0, J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }  
  else if (last_command =="r_hand_z_local_rot_ccw")
     {
  std::cout << "R Hand, Z Local, Rotating counterclockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration

  //----------------------------------------------------------------------------------------------------------------
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des =  locoman::utils::Rot_z(0.1) ; //locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_z(-0.001) ;
     std::cout << " Rot_des = " <<  std::endl << Rot_des.toString() << std::endl; 

  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 

  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );    
     }        
     
   else if (last_command =="r_hand_z_local_rot_cw")
     {
  std::cout << "R Hand, Z Local, Rotating clockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration 
  //----------------------------------------------------------------------------------------------------------------
  
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des = locoman::utils::Rot_z(-0.1) ;// locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_z(0.001) ;
       std::cout << " Rot_des = " <<  std::endl << Rot_des.toString() << std::endl; 
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
     }        
  else if (last_command =="r_hand_x_local_rot_ccw")
     {
  std::cout << "R Hand, X Local, Rotating counterclockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration 
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des = locoman::utils::Rot_x(0.1) ;//locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_x(-0.001) ;
  
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, J_r_hand_body_0, 
								J_l_c1_body_0, J_r_c1_body_0, 
								J_com_waist) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }        
   else if (last_command =="r_hand_x_local_rot_cw")
     {
  std::cout << "R Hand, X Local, Rotating clockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des =   locoman::utils::Rot_x(-0.1) ; //locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_x(0.001) ;
  
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
								Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
								zer_3 , 
								J_l_hand_body_0, 
                                                                J_r_hand_body_0, 
								J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
								J_com_waist) ;
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }      
      else if (last_command =="sw_rg_up_2")
     {
  std::cout << "R Foot, Z World, moving Forward"<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration for r_hand
//   yarp::sig::Matrix Rot_des_waist   = locoman::utils::getRot(   T_waist_r_hand_cmd ) ;
//   yarp::sig::Vector Trasl_des_waist = locoman::utils::getTrasl( T_waist_r_hand_cmd ) ;
//   Trasl_des_waist = R_waist_aw_cmd.transposed()*Trasl_des_waist;
//   Trasl_des_waist[0] -= 1.0 ;
//   Trasl_des_waist = R_waist_aw_cmd*Trasl_des_waist ;
//   yarp::sig::Matrix T_r_hand_des_waist = locoman::utils::Homogeneous(Rot_des_waist, Trasl_des_waist ) ;    
//   yarp::sig::Matrix T_r_hand_des_local = locoman::utils::iHomogeneous( T_waist_r_hand_0 )* T_r_hand_des_waist  ;
    //---------------------------------------------------------------------------------------------------------------
 yarp::sig::Matrix T_r_foot_des_local(4,4) ;
  
  yarp::sig::Matrix T_temp_2 =  T_r_c1_w_0* T_w_aw_0  ; 
  yarp::sig::Vector d_temp = T_temp_2.getCol(2) ;
  d_temp = -1.0*d_temp.subVector(0,2) ;  // TODO : control the correct direction from aw and the imu
  d_temp = 0.0008* d_temp/norm(d_temp) ;
  
  T_r_foot_des_local= locoman::utils::Homogeneous( Eye_3, d_temp) ; 
  
  yarp::sig::Vector zer_3(3,0.0) ;

   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, Eye_4, //T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  T_r_foot_des_local,                            //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0,
                                                                J_r_hand_body_0, 
                                                                J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );  
     }  
     
} // closing General Command part
  
  
  
  
 // desired contact force definition
 //   yarp::sig::Vector fc_des_to_world( size_fc, 0.0)  ;   
 //   fc_des_to_world = FC_DES ;
   
 /* FC_DES_LEFT_sensor  = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
  FC_DES_RIGHT_sensor = map_l_fcToSens * FC_DES.subVector(12,23)  ;
  // 
  yarp::sig::Vector fc_l_c1_filt = FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l_c2_filt = FC_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l_c3_filt = FC_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l_c4_filt = FC_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r_c1_filt = FC_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r_c2_filt = FC_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r_c3_filt = FC_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r_c4_filt = FC_FILTERED.subVector(21,23)  ; 
    
  yarp::sig::Vector d_fc_des_to_world(size_fc)  ;
  d_fc_des_to_world  = FC_DES - FC_FILTERED ; // -fc_to_world_0 ;
//-----------------------------------------------------------------------------------------------------------------
      
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " FC_DES = " <<  std::endl << FC_DES.toString() << std::endl; 
   std::cout << " FC_DES_LEFT_sensor = " <<  std::endl << FC_DES_LEFT_sensor.toString() << std::endl; 
   std::cout << " FC_DES_RIGHT_sensor = " <<  std::endl << FC_DES_RIGHT_sensor.toString() << std::endl;  
   std::cout << " norm(FC_DES_sens - FC_sens)  =  "<< std::endl << norm(FC_DES_LEFT_sensor- FC_FILTERED_LEFT_sensor) + norm(FC_DES_RIGHT_sensor- FC_FILTERED_RIGHT_sensor) << std::endl  ; 
   std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl;    
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
  
 
   //----------------------------------------------------------------------------------------------//
    //   Writing Data on a File
  double err = norm( d_fc_des_to_world )  ;  // d_fc_des_to_world
  double err_min = 0; //10.0 ;
  double err_max = 5;  //40.0 ; 
    
  char file_name[] = "err.m";
  std::ofstream err_cl ( file_name, std::ios::app );
  if( err_cl.is_open() )
  err_cl <<  err << std::endl;  
    
    /*char file_name1[] = "err1.m";
    char temp[] = "temp" ;
    std::ofstream temp ( file_name1, std::ios::app );
    if( &temp.is_open() )
    temp <<  err << std::endl;  */
    
    //-----------------------------------------------------------------------------------------------

 /* double alpha = alpha_filter(err, err_min, err_max) ;
  std::cout << " err = "  << err << std::endl; 

  std::cout << " alpha = "  << alpha << std::endl; 

  yarp::sig::Vector q_ref_ToMove = q_motor_side +  (1.0/1.0)*alpha*d_q_aw_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        
     
   //       q_ref_ToMove = moving_right_arm(-0.01);
  robot.move( q_ref_ToMove);                 //  q_motor_side);//  
  */

  }
   
   
   
   
   
   
   
   
   
   
//}

 
 //---------------------------------------------------------------------//

   //---------------------------------------------------------------------------------------------------------//  
    //  NO DELETE
    //      Writing data on a file
    //std::ofstream r_ankle ;
    //r_ankle.open ("r_ankle.txt");
 /*   std::ofstream r_ankle_cl ( "r_ankle.m", std::ios::app );
    if( r_ankle_cl.is_open() )
    r_ankle_cl <<  ft_r_ankle[2] << std::endl;
    //r_ankle.close();
    
    std::ofstream l_ankle_cl ( "l_ankle.m", std::ios::app );
    if( l_ankle_cl.is_open() )
    l_ankle_cl <<  ft_l_ankle[2] << std::endl;   */
    
//----------------------------------------------------------------------------------------------------------//  
       //--------------------------------------------------------------------------//
    // Getting Contact Forces
/*    RobotUtils::ftPtrMap fts = robot.getftSensors();
    
   //Cheking the existence of the sensors
    //assert(fts.size() > 0 && "no ft found!");
    
    //Printing Sensor Names
    RobotUtils::ftPtrMap::iterator i = fts.begin() ;    
    //::cout << i->second->getReferenceFrame() << std::endl;

    for(RobotUtils::ftPtrMap::iterator i = fts.begin() ;  i != fts.end(); i++)
    {
 //     std::cout << i->first << std::endl;
// Alternative formulations for the iterators      
// std::cout << i->second->getReferenceFrame() << std::endl;     
//      (*i).second->getReferenceFrame();
//      fts[i->first]->getReferenceFrame(); 
    }*/
  
   
   
   /*
 //------------------------------------------------    
 // Computation of V
 double part = -6.0/10.0 ; //portare fuori
 
 double mu_l ;
 double mu_r ;
 
   if( part ==0){ 
     mu_l = 1 ;
     mu_r = 1 ;
  }  ;
   if( part > 0){ 
     mu_l = 1-part +0.1 ;
     mu_r = 1 ;
  }  ;
   if( part < 0){ 
     mu_l = 1;
     mu_r = 1 + part +0.1 ;
  }  ;     
// Contact left 1   
 double V_l_c1 =  V_ij(  sigma_frict(-1.0*fc_l_c1_filt, mu_l)  )   +  V_ij(sigma_max(-1.0*fc_l_c1_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c1_filt, 0)) ;

 // Contact left 2
 double V_l_c2 =   V_ij(sigma_frict(-1.0*fc_l_c2_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c2_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c2_filt, 0)) ;
  
 // Contact left  3
 double V_l_c3 =   V_ij(sigma_frict(-1.0*fc_l_c3_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c3_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c3_filt, 0)) ;
  
 // Contact left  4
 double V_l_c4 =   V_ij(sigma_frict(-1.0*fc_l_c4_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c4_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c4_filt, 0)) ;
 
  // Contact right 1
   double V_r_c1 =   V_ij(sigma_frict(-1.0*fc_r_c1_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_r_c1_filt, 700)) +  V_ij(sigma_min(-1.0*fc_r_c1_filt, 0)) ;
  // Contact right 2
   double V_r_c2 =   V_ij(sigma_frict(-1.0*fc_r_c2_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_r_c2_filt, 700)) +  V_ij(sigma_min(-1.0*fc_r_c2_filt, 0)) ;
  // Contact right 3
   double V_r_c3 =   V_ij(sigma_frict(-1.0*fc_r_c3_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_r_c3_filt, 700)) +  V_ij(sigma_min(-1.0*fc_r_c3_filt, 0)) ;
  // Contact right 4
   double V_r_c4 =   V_ij(sigma_frict(-1.0*fc_r_c4_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_r_c4_filt, 700)) +  V_ij(sigma_min(-1.0*fc_r_c4_filt, 0)) ;
  //
   double V = V_l_c1 + V_l_c2 + V_l_c3 +V_l_c4 + V_r_c1 + V_r_c2 + V_r_c3 + V_r_c4 ;
   //
   std::ofstream V_plot_cl ( "V_plot.m", std::ios::app );
    if( V_plot_cl.is_open() )
    V_plot_cl <<  V << std::endl;  
  //---------------------------------------------------------------------------// 
    */
  
    




bool locoman_control_thread::custom_pause()
{

}

bool locoman_control_thread::custom_resume()
{

}


