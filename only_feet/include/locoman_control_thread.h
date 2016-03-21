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
    
    // 
    unsigned int left_arm_joints;
    double omega;
    double phi;
    int tick;
    unsigned int right_arm_joints;
    unsigned int left_leg_joints;
    unsigned int right_leg_joints;
    unsigned int torso_joints;    
    
    //Example stuff for parameters setting from outside
    yarp::sig::Vector left_arm_configuration;
    yarp::sig::Vector right_arm_configuration;
    yarp::sig::Vector left_leg_configuration;
    yarp::sig::Vector right_leg_configuration;
    yarp::sig::Vector torso_configuration;

    yarp::sig::Vector left_arm_config_0  ;
    yarp::sig::Vector right_arm_config_0 ;
    yarp::sig::Vector left_leg_config_0  ;
    yarp::sig::Vector right_leg_config_0 ;
    yarp::sig::Vector torso_config_0     ;
    
    yarp::sig::Vector left_arm_config_1  ;
    yarp::sig::Vector right_arm_config_1 ;
    yarp::sig::Vector left_leg_config_1  ;
    yarp::sig::Vector right_leg_config_1 ;
    yarp::sig::Vector torso_config_1     ;
    
    double max_vel;
public:
    
    unsigned int size_q ;
    int mg =  290 ; // [N]  295 // mg_coman = 290; mg_bigman = 1000 ;
    int loop_counter;
    int WINDOW_size;
    int FC_size ;  
    bool flag_robot = 1 ;
    bool flag_simulator = 1-flag_robot ;
    
    
    yarp::sig::Vector fc_offset_left ;
    yarp::sig::Vector fc_offset_right ;
    
    yarp::sig::Vector fc_offset_left_hand ;
    yarp::sig::Vector fc_offset_right_hand ;    
    
    yarp::sig::Vector q_offset  ;  
    yarp::sig::Vector q_current_open_loop  ;  

    
    int FC_HANDS_size ;
    
    std::string last_command = "pause" ;
    
    //--------------
    yarp::sig::Vector CoM_waist_cmd  ;  
    yarp::sig::Matrix T_waist_l1_foot_cmd ; 
    yarp::sig::Matrix T_waist_r1_foot_cmd ; 
    yarp::sig::Matrix T_waist_l_hand_cmd ; 
    yarp::sig::Matrix T_waist_r_hand_cmd ; 
    yarp::sig::Matrix R_waist_aw_cmd ;     
    
    //----------------
    
    yarp::sig::Vector CoM_w_cmd ;  // variables registered at command time
    yarp::sig::Vector CoM_w_up ;
    yarp::sig::Vector CoM_w_dw ;
    
    
    yarp::sig::Matrix T_w_l1_cmd ; 
    yarp::sig::Matrix T_w_r1_cmd ; 
    
    yarp::sig::Matrix T_l1_r1_up ;
    yarp::sig::Matrix T_l1_r1_fw ;
    yarp::sig::Matrix T_l1_r1_dw ;
    
    yarp::sig::Matrix T_r1_l1_up ;
    yarp::sig::Matrix T_r1_l1_fw ;
    yarp::sig::Matrix T_r1_l1_dw ;
    
    yarp::sig::Vector FC_DES ;  //     yarp::sig::Vector FC_DES( FC_size   ) ;
    yarp::sig::Vector FC_DES_LEFT_sensor ;
    yarp::sig::Vector FC_DES_RIGHT_sensor ;
    yarp::sig::Vector FC_SUM ;
    yarp::sig::Vector FC_FILTERED ;
    yarp::sig::Matrix FC_WINDOW ;  //    yarp::sig::Matrix FC_WINDOW(FC_size, WINDOW_filter ) ;
    
    yarp::sig::Vector FC_HANDS_DES ;  //     yarp::sig::Vector FC_DES( FC_size   ) ;
    yarp::sig::Vector FC_DES_LEFT_HAND_sensor ;
    yarp::sig::Vector FC_DES_RIGHT_HAND_sensor ;
    yarp::sig::Vector FC_HANDS_SUM ;
    yarp::sig::Vector FC_HANDS_FILTERED ;
    yarp::sig::Matrix FC_HANDS_WINDOW ;  //    yarp::sig::Matrix FC_WINDOW(FC_size, WINDOW_filter ) ;
    
    yarp::sig::Vector zero_3 ;
    yarp::sig::Matrix Zeros_6_6 ;
    yarp::sig::Matrix Eye_6 ;
    yarp::sig::Matrix Eye_3 ; 
    yarp::sig::Matrix Eye_4 ;
    yarp::sig::Matrix B ;
    
    unsigned int size_u = 6 ;
    unsigned int size_fc = 24;
    double kc ;
    yarp::sig::Matrix Kq ;
    yarp::sig::Matrix Kc ;
    yarp::sig::Matrix Kc_f_rh ;
    yarp::sig::Vector ft_l_ankle ;
    yarp::sig::Vector ft_r_ankle ;
    yarp::sig::Vector ft_l_wrist ;
    yarp::sig::Vector ft_r_wrist ;
    
    unsigned int waist_index ;
    unsigned int l_ankle_index ;
    unsigned int l_c1_index ;
    unsigned int l_c2_index ;
    unsigned int l_c3_index ;
    unsigned int l_c4_index ;
    unsigned int r_ankle_index ;
    unsigned int r_c1_index ;
    unsigned int r_c2_index ;
    unsigned int r_c3_index ;
    unsigned int r_c4_index ;
    unsigned int l_hand_index ;
    unsigned int r_hand_index ;
    unsigned int l_wrist_index ;
    unsigned int l_hand_c1_index ;
    unsigned int l_hand_c2_index ;
    unsigned int l_hand_c3_index ;
    unsigned int l_hand_c4_index ;
    unsigned int r_wrist_index ;
    unsigned int r_hand_c1_index ;
    unsigned int r_hand_c2_index ;
    unsigned int r_hand_c3_index ;
    unsigned int r_hand_c4_index ;
   
    yarp::sig::Matrix map_l_fcToSens ;
    yarp::sig::Matrix map_r_fcToSens ;  
    yarp::sig::Matrix map_l_hand_fcToSens ;    
    yarp::sig::Matrix map_r_hand_fcToSens ;
    
    yarp::sig::Matrix map_l_fcToSens_PINV ;
    yarp::sig::Matrix map_r_fcToSens_PINV ;
    yarp::sig::Matrix map_l_hand_fcToSens_PINV ;
    yarp::sig::Matrix map_r_hand_fcToSens_PINV ;

  yarp::sig::Vector fc_l_c1_filt ; //= FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l_c2_filt ; //= FC_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l_c3_filt ; //= FC_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l_c4_filt ; //= FC_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r_c1_filt ; //= FC_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r_c2_filt ; //= FC_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r_c3_filt ; //= FC_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r_c4_filt ; //= FC_FILTERED.subVector(21,23)  ; 
  
  yarp::sig::Vector fc_l1_hand_filt ; //= FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  yarp::sig::Vector fc_l2_hand_filt ; // = FC_HANDS_FILTERED.subVector(3,5)  ;
  yarp::sig::Vector fc_l3_hand_filt ; //= FC_HANDS_FILTERED.subVector(6,8)  ;
  yarp::sig::Vector fc_l4_hand_filt ; //= FC_HANDS_FILTERED.subVector(9,11)  ;

  yarp::sig::Vector fc_r1_hand_filt ; //= FC_HANDS_FILTERED.subVector(12,14)  ; 
  yarp::sig::Vector fc_r2_hand_filt ; //= FC_HANDS_FILTERED.subVector(15,17)  ; 
  yarp::sig::Vector fc_r3_hand_filt ; //= FC_HANDS_FILTERED.subVector(18,20)  ; 
  yarp::sig::Vector fc_r4_hand_filt ; //= FC_HANDS_FILTERED.subVector(21,23)  ;   

  yarp::sig::Matrix T_w_aw_0 ; //= locoman::utils::AW_world_posture(model, robot) ;
  yarp::sig::Matrix T_aw_w_0 ; //= locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  yarp::sig::Matrix T_w_waist_0  ; // = model.iDyn3_model.getPosition(waist_index) ;  
  yarp::sig::Matrix T_w_l_ankle_0; // = model.iDyn3_model.getPosition(l_ankle_index) ;
  yarp::sig::Matrix T_w_l_c1_0    ; //= model.iDyn3_model.getPosition(l_c1_index)    ;    
  yarp::sig::Matrix T_w_l_c2_0   ; // = model.iDyn3_model.getPosition(l_c2_index)    ;  
  yarp::sig::Matrix T_w_l_c3_0   ; // = model.iDyn3_model.getPosition(l_c3_index)    ;
  yarp::sig::Matrix T_w_l_c4_0   ; // = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_ankle_0 ; //= model.iDyn3_model.getPosition(r_ankle_index) ;
  yarp::sig::Matrix T_w_r_c1_0   ; // = model.iDyn3_model.getPosition(r_c1_index)    ;    
  yarp::sig::Matrix T_w_r_c2_0   ; // = model.iDyn3_model.getPosition(r_c2_index)    ;  
  yarp::sig::Matrix T_w_r_c3_0   ; // = model.iDyn3_model.getPosition(r_c3_index)    ;
  yarp::sig::Matrix T_w_r_c4_0   ; // = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  yarp::sig::Matrix T_w_l_hand_0  ; //= model.iDyn3_model.getPosition( l_hand_index ) ;
  yarp::sig::Matrix T_w_r_hand_0  ; //= model.iDyn3_model.getPosition( r_hand_index ) ;   

  yarp::sig::Matrix T_w_l_wrist_0 ; //= model.iDyn3_model.getPosition(l_wrist_index) ;
  yarp::sig::Matrix T_w_l1_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_l2_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_l3_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  yarp::sig::Matrix T_w_l4_hand_0 ; //= model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  yarp::sig::Matrix T_w_r_wrist_0 ; //= model.iDyn3_model.getPosition(r_wrist_index) ;
  yarp::sig::Matrix T_w_r1_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  yarp::sig::Matrix T_w_r2_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  yarp::sig::Matrix T_w_r3_hand_0 ;  // = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  yarp::sig::Matrix T_w_r4_hand_0 ; //= model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  // -----------------------------------------------------------------------
  yarp::sig::Matrix T_waist_w_0   ; //= locoman::utils::iHomogeneous(T_w_waist_0)  ;
  yarp::sig::Matrix T_l_ankle_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  yarp::sig::Matrix T_l_c1_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  yarp::sig::Matrix T_l_c2_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  yarp::sig::Matrix T_l_c3_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  yarp::sig::Matrix T_l_c4_w_0    ; //= locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  yarp::sig::Matrix T_r_ankle_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  yarp::sig::Matrix T_r_c1_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  yarp::sig::Matrix T_r_c2_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  yarp::sig::Matrix T_r_c3_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  yarp::sig::Matrix T_r_c4_w_0    ; //= locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  yarp::sig::Matrix T_l_wrist_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  yarp::sig::Matrix T_l1_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  yarp::sig::Matrix T_l2_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  yarp::sig::Matrix T_l3_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  yarp::sig::Matrix T_l4_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  yarp::sig::Matrix T_r_wrist_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  yarp::sig::Matrix T_r1_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  yarp::sig::Matrix T_r2_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  yarp::sig::Matrix T_r3_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  yarp::sig::Matrix T_r4_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  //---------------------------------------------------------------------
  
  yarp::sig::Matrix T_l_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  yarp::sig::Matrix T_r_hand_w_0 ; //= locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  yarp::sig::Matrix T_aw_l_c1_0 ; //= T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l_c2_0 ; //= T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l_c3_0 ; //= T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l_c4_0 ; //= T_aw_w_0 * T_w_l_c4_0 ;

  yarp::sig::Matrix T_aw_r_c1_0 ; //= T_aw_w_0 * T_w_r_c1_0 ;
  yarp::sig::Matrix T_aw_r_c2_0 ; //= T_aw_w_0 * T_w_r_c2_0 ;
  yarp::sig::Matrix T_aw_r_c3_0 ; //= T_aw_w_0 * T_w_r_c3_0 ;
  yarp::sig::Matrix T_aw_r_c4_0 ; //= T_aw_w_0 * T_w_r_c4_0 ; 

  yarp::sig::Matrix T_aw_l1_hand_0 ; //= T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  yarp::sig::Matrix T_aw_l2_hand_0 ; //= T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  yarp::sig::Matrix T_aw_l3_hand_0 ; //= T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  yarp::sig::Matrix T_aw_l4_hand_0 ; //= T_aw_w_0 * T_w_l4_hand_0 ;

  yarp::sig::Matrix T_aw_r1_hand_0 ; //= T_aw_w_0 * T_w_r1_hand_0 ;
  yarp::sig::Matrix T_aw_r2_hand_0 ; //= T_aw_w_0 * T_w_r2_hand_0 ;
  yarp::sig::Matrix T_aw_r3_hand_0 ; //= T_aw_w_0 * T_w_r3_hand_0 ;
  yarp::sig::Matrix T_aw_r4_hand_0 ; //= T_aw_w_0 * T_w_r4_hand_0 ;   
  
  //--------------------------------------------------
  
   // Jacobian Matrices 
  yarp::sig::Matrix J_l_c1_mix_0 ; //( 6, ( size_q + 6 ) ) ; //
  yarp::sig::Matrix J_l_c2_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c3_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l_c4_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r_c1_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c2_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c3_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_c4_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  yarp::sig::Matrix J_l1_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l2_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l3_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_l4_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Matrix J_r1_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; // robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r2_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r3_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  yarp::sig::Matrix J_r4_hand_mix_0 ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //------------------------------------------------
  yarp::sig::Matrix J_l_c1_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  yarp::sig::Matrix J_l_c2_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  yarp::sig::Matrix J_l_c3_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  yarp::sig::Matrix J_l_c4_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  yarp::sig::Matrix J_r_c1_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  yarp::sig::Matrix J_r_c2_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  yarp::sig::Matrix J_r_c3_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  yarp::sig::Matrix J_r_c4_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  yarp::sig::Matrix J_l_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  yarp::sig::Matrix J_r_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  yarp::sig::Matrix J_l1_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  yarp::sig::Matrix J_l2_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  yarp::sig::Matrix J_l3_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  yarp::sig::Matrix J_l4_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  yarp::sig::Matrix J_r1_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  yarp::sig::Matrix J_r2_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  yarp::sig::Matrix J_r3_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  yarp::sig::Matrix J_r4_hand_body_0 ; //= locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Introducing Spatial Jacobian terms: Fixed base in {AW}
  
  yarp::sig::Matrix J_aw_l_c1_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l_c2_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l_c3_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l_c4_spa_0 ; //= locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r_c1_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r_c2_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r_c3_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r_c4_spa_0 ; //= locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  yarp::sig::Matrix J_aw_l1_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  yarp::sig::Matrix J_aw_l2_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  yarp::sig::Matrix J_aw_l3_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  yarp::sig::Matrix J_aw_l4_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  yarp::sig::Matrix J_aw_r1_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  yarp::sig::Matrix J_aw_r2_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  yarp::sig::Matrix J_aw_r3_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  yarp::sig::Matrix J_aw_r4_hand_spa_0 ; //= locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  //-----------------------------------------------------------
  
  yarp::sig::Matrix Q_aw_l_c1 ; //(size_q+ 6, size_q + 6)   ; //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  yarp::sig::Matrix Q_aw_l_c2 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_l_c3 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_l_c4 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  yarp::sig::Matrix Q_aw_r_c1 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c2 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r_c3 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r_c4 ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  yarp::sig::Matrix Q_aw_r1_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r2_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  yarp::sig::Matrix Q_aw_r3_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  yarp::sig::Matrix Q_aw_r4_hand ; //(size_q+ 6, size_q + 6)   ; // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  yarp::sig::Matrix Q_aw_l_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  yarp::sig::Matrix Q_aw_r_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  yarp::sig::Matrix Q_aw_r_hand_tot ; //(size_q+ 6, size_q + 6)   ; // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;

  yarp::sig::Matrix Q_aw_c ; //(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_cont ; //( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_cont ; //( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  
  yarp::sig::Matrix Q_aw_c_f_rh ; //(size_q+ 6, size_q + 6)   ; // =  Q_aw_l_tot + Q_aw_r_tot ;  
  yarp::sig::Matrix U_aw_s_c_f_rh ; //( 6 , 6) ; // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  yarp::sig::Matrix Q_aw_s_c_f_rh ; //( 6 , size_q ) ; //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  //----------------------------------------------------------------------------------------
  yarp::sig::Vector d_fc_des_to_world ; //(size_fc)  ;
  yarp::sig::Vector d_EE_r_des ; //(6,0.0) ;
  yarp::sig::Vector d_EE_l_des ; //(6,0.0) ;
  yarp::sig::Matrix T_l_c1_r_c1_loop ; //(4,4) ;
  yarp::sig::Matrix T_r_c1_l_c1_loop ; //(4,4) ;
  yarp::sig::Matrix J_com_w ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_com_w_redu ; //( 3,  ( size_q + 6 )  ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_aw ; //( 3,  ( size_q + 6 ) ) ; //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  yarp::sig::Matrix J_com_waist ; //( 3,  ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ))   ;

  yarp::sig::Matrix J_r_c1_aw ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  yarp::sig::Matrix J_l_c1_aw ; //( 6, ( size_q + 6 ) ) ; //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  yarp::sig::Vector q_ref_ToMove ; //( ( size_q + 6 ) , 0.0) ;
  //
  
    /**
     * @brief tutorial_control_thread constructor
     * @param module_prefix passed from the module
     * @param rf passed from the module
     * @param ph a param helper builted while generating the control thread
     */
    locoman_control_thread(     std::string module_prefix, 
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

    //------------------------------------------------------------





//          //----------------------------------------------------------------------------
//      /**
//      * @brief  Q_ci compute the derivative of the spatial Jacobian 
//      * @param  J_spa_i is a 6xc yarp matrix describing a spatial Jacobian
//      * @param  T_a_ci is the homogeneous transformation between the floating base and the contact frame 
//      * @param  f_ci contact force vector
//      * @return qxq yarp matrix 
//      */
//     yarp::sig::Matrix Q_ci( const yarp::sig::Matrix J_spa_i, const yarp::sig::Matrix T_a_ci , const yarp::sig::Vector f_ci) ;
//     
    
//      //----------------------------------------------------------------------------
//      /**
//      * @brief  RoundMatrix computes the round of M to k decimal places 
//      * @param  M yarp matrix to round
//      * @param  k number of decimal places 
//      * @return rounded yarp matrix 
//      */
//     yarp::sig::Matrix RoundMatrix( const yarp::sig::Matrix M, const int k) ;
//     
    
//      //------------------------------------------------------------------------------------
//      /**
//      * @brief  FLMM_ext computes the FLMM for a compliant humanoid robot 
//      * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
//      * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
//      * @param  Q_j is a joints x joints yarp matrix about the derivative of the Jacobian 
//      * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
//      * @param  U_j is a joints x 6 yarp matrix about the derivative of the Jacobian 
//      * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
//      * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
//      * @param  K_q is a joints x joints yarp matrix describing the joint stiffness matrix
//      * @return FLMM_ext is the Fundamental Loco-Manipulation Matrix
//      */
//     yarp::sig::Matrix FLMM_ext( const yarp::sig::Matrix J_c ,
// 			    const yarp::sig::Matrix S_c ,
// 			    const yarp::sig::Matrix Q_j,
// 			    const yarp::sig::Matrix Q_s,
// 			    const yarp::sig::Matrix U_j,
// 			    const yarp::sig::Matrix U_s,
// 			    const yarp::sig::Matrix K_c,
// 			    const yarp::sig::Matrix K_q
// 			      ) ;

/*   //------------------------------------------------------------------------------------
     /**
     * @brief  Rf_ext computes the joints-forces map for a compliant humanoid robot 
     * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
     * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
     * @param  Q_j is a joints x joints yarp matrix about the derivative of the Jacobian 
     * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
     * @param  U_j is a joints x 6 yarp matrix about the derivative of the Jacobian 
     * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
     * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
     * @param  K_q is a joints x joints yarp matrix describing the joint stiffness matrix
     * @return Rf_ext computes the joint-forces map
     */
/*    yarp::sig::Matrix Rf_ext( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_j,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_j,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c,
			    const yarp::sig::Matrix K_q
			      ) ;*/			      
	      
//      //------------------------------------------------------------------------------------
//      /**
//      * @brief  FLMM_redu computes a basic version of the FLMM for a rigid humanoid robot 
//      * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
//      * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
//      * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
//      * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
//      * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
//      * @return FLMM_ext is the Fundamental Loco-Manipulation Matrix
//      */
//     yarp::sig::Matrix FLMM_redu( const yarp::sig::Matrix J_c ,
// 			    const yarp::sig::Matrix S_c ,
// 			    const yarp::sig::Matrix Q_s,
// 			    const yarp::sig::Matrix U_s,
// 			    const yarp::sig::Matrix K_c
// 			      ) ; 			      
//      //------------------------------------------------------------------------------------
//      /**
//      * @brief  Rf_redu computes the joints-forces map for a rigid humanoid robot 
//      * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
//      * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
//      * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
//      * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
//      * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
//      * @return Rf_redu is the Fundamental Loco-Manipulation Matrix
//      */
//     yarp::sig::Matrix Rf_redu( const yarp::sig::Matrix J_c ,
// 			    const yarp::sig::Matrix S_c ,
// 			    const yarp::sig::Matrix Q_s,
// 			    const yarp::sig::Matrix U_s,
// 			    const yarp::sig::Matrix K_c
// 			      ) ;			      
// 			      
//        //------------------------------------------------------------------------------------
//      /**
//      * @brief  Ru_redu computes the joints-movements map for a a rigid humanoid robot 
//      * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
//      * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
//      * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
//      * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
//      * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
//      * @return Ru_redu is the Fundamental Loco-Manipulation Matrix
//      */
//     yarp::sig::Matrix Ru_redu( const yarp::sig::Matrix J_c ,
// 			    const yarp::sig::Matrix S_c ,
// 			    const yarp::sig::Matrix Q_s,
// 			    const yarp::sig::Matrix U_s,
// 			    const yarp::sig::Matrix K_c
// 			      ) ;		
/*//------------------------------------------------------------------------------------
 /*
     * @brief  Pinv_trunc_SVD computes the pseudoinverse of A via the truncated SVD method 
     * @param  A is the matrix to pseudo-inverse
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return Pinv_trunc_SVD is the pseudo-inverse of A
     */
//      yarp::sig::Matrix Pinv_trunc_SVD( const yarp::sig::Matrix A ,
			               // const double k = 1E-4 
			            //  ) ;   // 
//------------------------------------------------------------------------------------
     /**
//     * @brief  Pinv_Regularized computes the Levemberg Regularized pseudo-inverse of A 
     * @param  A is the matrix to pseudo-inverse
     * @param  k is the regularization factor
     * @return Pinv_Regularized is the pseudo-inverse of A
     */ 
/*      yarp::sig::Matrix Pinv_Regularized( const yarp::sig::Matrix A ,
			                  const double k  
			                ) ;*/     
					
     /**
     * @brief  Pinv_Marq computes the Levemberg-Marquard Regularized pseudo-inverse of A 
     * @param  A is the matrix to pseudo-inverse
     * @param  k is the regularization factor
     * @return Pinv_Marq is the pseudo-inverse of A
     */ 
/*      yarp::sig::Matrix Pinv_Marq( const yarp::sig::Matrix A ,
			                  const double k  
			                ) ;*/    
//------------------------------------------------------------------------------------
     /**
     * @brief  x_Pinv_Iter computes the variable x: Ax=b via the Landweber iteration method
     * @param  A is the matrix to pseudo-inverse
     * @param  b is the vector of known terms
     * @param  n is the maximum number of steps to be performed (less that the minimum dimension of A)
     * @return x_Pinv_Iter is the solution vetor
     */
/*      yarp::sig::Vector x_Pinv_Iter( const yarp::sig::Matrix A , 
				   const yarp::sig::Vector b , 
			           double n 
			           ) ; */  // 			      
     /**
     * @brief  orth_SVD computes a basis for the span of A via the truncated SVD method 
     * @param  A is the matrix of which a basis is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return orth_SVD is a basis for the span of A
     */
/*      yarp::sig::Matrix orth_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ; */  // 			           
     /**
     * @brief  null_SVD computes a basis for the nullspace of A via the truncated SVD method 
     * @param  A is the matrix of which a basis for the nullspace is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return null_SVD is a basis for the nullspace of A
     */
//       yarp::sig::Matrix null_SVD( const yarp::sig::Matrix A ,
// 			                const double k = 1E-4 
// 			              ) ;  
           
     /**
     * @brief  filter_SVD computes a "filtered" version of A via the truncated SVD  
     * @param  A is the matrix of which the filtered version is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return filter_SVD is the filtered version of A
     */
/*      yarp::sig::Matrix filter_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ;	*/			      


//      /**
//      * @brief  sigma_frict computes the metrics measuring the goodness of the contact force with respect to friction limits 
//      * @param  fc is the contact force. The normal is assumed to be n = [0 0 1]^T 
//      * @param  mu is the friction coefficient
//      * @return the value of sigma_frict
//      */
//       double sigma_frict( const yarp::sig::Vector fc ,
// 			  const double mu 
// 			              ) ; 

//      /**
//      * @brief  sigma_min computes the distance (along a certain metric) with respect to minimum force allowed 
//      * @param  fc is the contact force.  
//      * @param  f_min is the minimum module of the force allowed
//      * @return the value of sigma_min
//      */
//       double sigma_min( const yarp::sig::Vector fc,
//          		const double f_min 
// 			              ) ; 

//      /**
//      * @brief  sigma_max computes the distance (along a certain metric) with respect to maximum force allowed 
//      * @param  fc is the contact force.  
//      * @param  f_max is the maximum module of the force allowed
//      * @return the value of sigma_min
//      */
//       double sigma_max( const yarp::sig::Vector fc,
//          		const double f_max 
// 			              ) ; 
	      
//      /**
//      * @brief  V_ij computes the distance (along a certain metric) with respect to the contact limits
//      * @param  sigma is one exit of the functions sigma_frict, sigma_min, sigma_max
//      * @param  toll is the admitted tolerance with respect to sigma limit value (= 0)
//      * @return the value of V_ij
//      */
//       double V_ij( const double sigma, 
//                    const double toll  = 1E-7
// 			              ) ; 
				      
//      /**
//      * @brief  provide the initial configuration
//      * @return  yarp vector 
//      */
//      yarp::sig::Vector q_init( void ) ; 
    
/*     /**
     * @brief  provide the desired contact force distribution => 100% on the right
     * @return  0 if ok 
     */
//      int FC_DES_right( void ) ;   */  

//      /**
//      * @brief  provide the desired contact force distribution => 100% on the left
//      * @return  0 if ok
//      */
//      int FC_DES_left( void ) ;   

/*     /**
     * @brief  provide the desired contact force distribution => 50% on the left/right
     * @return  0 if ok
     */
//      int FC_DES_center( void ) ;  */ 
  
//      /**
//      * @brief  easy way for rotating the right shoulder 
//      * @param alpha rotation angle [rad], angluar step at each loop
//      * @return the uptated joint vector configuration 
//      */
//      yarp::sig::Vector moving_right_arm( const double alpha ) ;  
       
//      /**
//      * @brief linear function from the values (0,err_min) to (1,err_max)
//      * @param err the point on which the filtering function has to be computed
//      * @param err_min minimum error value (positive)
//      * @param err_max maximum error value (greather than err_min)
//      * @return the filtering value
//      */
//      double alpha_filter( double err, double err_min, double err_max ) ;  
     
//      /**
//      * @brief The function computes and returns the pose of the frame auxiliary world frame {AW} with 
//      * @return respect to the world
//      */
//      yarp::sig::Matrix AW_world_posture( void  ) ;  

     /**
     * @brief linear function from the values (0,err_min) to (1,err_max)
     * @param err the point on which the filtering function has to be computed
     * @param err_min minimum error value (positive)
     * @param err_max maximum error value (greather than err_min)
     * @return the filtering value 
     */
    // double data_on_file( char file_name, double data  ) ;  
     
     
     
//      /**
//      * @brief  Rot2Quat computes the quaterion components given the rotation matrix
//      * @param  Rot is 3 x 3 rotation matrix
//      * @return a vector of 4 elements; the first element is the scalar part of the quaternion
//      */
//     yarp::sig::Vector Rot2Quat( const yarp::sig::Matrix Rot) ;		
//      
//      /**
//      * @brief  Orient_Error computes the orientation error based on the quaternion representation of the rotation matrices
//      * @param  Rot_des is 3 x 3 rotation matrix representing the desired orientation
//      * @param  Rot_cur is 3 x 3 rotation matrix representing the current orientation
//      * @return a vector of 3 elements representing the orientation error
//      */
//     yarp::sig::Vector Orient_Error( const yarp::sig::Matrix Rot_des, const yarp::sig::Matrix Rot_cur) ;		
// 
//      /**
//      * @brief  Inv_quaternion computes the inverse of a given quaternion
//      * @param  quat is 4 elements vector describing a quaternion
//      * @return a vector of 4 elements representing the inverse of the input quaternion
//      */
//     yarp::sig::Vector Inv_quaternion( const yarp::sig::Vector quat ) ;	
// 
//      /**
//      * @brief  Computes the product of two quaternions
//      * @param  quat_1 and quat_2 are 4 elements vectors describing quaternions
//      * @return a vector of 4 elements representing the product of the two input quaternions
//      */
//     yarp::sig::Vector quat_Product( const yarp::sig::Vector quat_1 , const yarp::sig::Vector quat_2) ;	
//     
//     /**
//      * @brief  Computes the rotation matrix about x axis
//      * @param  phi_x rotation angle
//      * @return a 3x3 yarp Matrix
//      */
//     yarp::sig::Matrix Rot_x( const double phi_x ) ;	
//         
//     /**
//      * @brief  Computes the rotation matrix about y axis
//      * @param  theta_y rotation angle
//      * @return a 3x3 yarp Matrix
//      */   
//     yarp::sig::Matrix Rot_y( const double theta_y ) ;
//     
//      /**
//      * @brief  Computes the rotation matrix about z axis
//      * @param  psi_z rotation angle
//      * @return a 3x3 yarp Matrix
//      */
//     yarp::sig::Matrix Rot_z( const double psi_z) ;	
//      

    //----------------------------------------------------------------------
     
/*     /**
     * @brief  WB_Cartesian_Tasks computes the whole-body displacement for achieving the 
     *         desired configurations of the End Effectors and of the CoM (all together)
     * @param  T_l1_foot_des desired pose of the left foot (first contact) in local frame
     * @param  T_r1_foot_des desired pose of the right foot (first contact) in local frame
     * @param  T_l_hand_des  desired pose of the left hand in local frame
     * @param  T_r_hand_des  desired pose of the right hand in local frame
     * @param  CoM_waist_cmd desired position of the CoM w.r.t the wrist frame
     * 
     * @param  J_l1_foot_body is a (6 x (joints+6)) body Jacobian of the left  foot (first contact) 
     * @param  J_r1_foot_body is a (6 x (joints+6)) body Jacobian of the right foot (first contact) 
     * @param  J_l_hand_body  is a (6 x (joints+6)) body Jacobian of the left  hand
     * @param  J_r_hand_body  is a (6 x (joints+6)) body Jacobian of the right hand 
     * @param  J_waist_CoM    is a (3 x (joints+6)) Jacobian of the CoM expressed in {Waist} frame
     * @return the desired delta_q vector 
     */
/*    yarp::sig::Vector WB_Cartesian_Tasks( 
			    const yarp::sig::Matrix T_l_hand_des,
			    const yarp::sig::Matrix T_r_hand_des,
                            const yarp::sig::Matrix T_l1_foot_des ,
			    const yarp::sig::Matrix T_r1_foot_des ,
			    const yarp::sig::Vector CoM_waist_cmd ,
			    const yarp::sig::Matrix J_l_hand_body ,
			    const yarp::sig::Matrix J_r_hand_body ,
			    const yarp::sig::Matrix J_l1_foot_body ,
			    const yarp::sig::Matrix J_r1_foot_body ,
			    const yarp::sig::Matrix J_waist_CoM 
					) ;*/ 		
    
    
};




#endif
