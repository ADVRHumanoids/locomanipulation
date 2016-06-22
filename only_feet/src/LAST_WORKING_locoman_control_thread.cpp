// This is the last working version before the architecture control modification, 
// to be maintained as a backup
//
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
                                        std::shared_ptr< paramHelp::ParamHelperServer > ph ) :
    control_thread( module_prefix, rf, ph ),  
    size_q(robot.getNumberOfKinematicJoints()),
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
    FC_HANDS_WINDOW(FC_HANDS_size, WINDOW_size ) ,
    //---------------------------------
    // number of fc components on the feet
    zero_3(3, 0.0) ,
    Zeros_6_6(6,6) ,
    Eye_6(6,6) ,
    Eye_3(3,3) ,
    Eye_4(4,4) ,
    B(6,3) ,
    Kc( 24 , 24) ,
    Kc_f_rh( 36 ,36 ) ,
    ft_l_ankle(6,0.0) ,
    ft_r_ankle(6,0.0) ,
    ft_l_wrist(6,0.0) ,
    ft_r_wrist(6,0.0) ,
    fc_offset_left(12, 0.0) ,
    fc_offset_right(12, 0.0) ,
    fc_offset_left_hand(12, 0.0) ,
    fc_offset_right_hand(12, 0.0),
    
    //
    map_l_fcToSens(6,12) ,
    map_r_fcToSens(6,12) ,  
    map_r_hand_fcToSens(6,12) ,
    map_l_hand_fcToSens(6,12) ,
    map_l_fcToSens_PINV(12,6) ,
    map_r_fcToSens_PINV(12,6) ,
    map_l_hand_fcToSens_PINV(12,6) ,
    map_r_hand_fcToSens_PINV(12,6) ,
    //
  fc_l_c1_filt(3, 0.0) , // = FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l_c2_filt(3, 0.0) , // = FC_FILTERED.subVector(3,5)  ;
  fc_l_c3_filt(3, 0.0) , // = FC_FILTERED.subVector(6,8)  ;
  fc_l_c4_filt(3, 0.0) , // = FC_FILTERED.subVector(9,11)  ;

  fc_r_c1_filt(3, 0.0) , // = FC_FILTERED.subVector(12,14)  ; 
  fc_r_c2_filt(3, 0.0) , // = FC_FILTERED.subVector(15,17)  ; 
  fc_r_c3_filt(3, 0.0) , // = FC_FILTERED.subVector(18,20)  ; 
  fc_r_c4_filt(3, 0.0) , // = FC_FILTERED.subVector(21,23)  ; 
  
  fc_l1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(3,5)  ;
  fc_l3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(6,8)  ;
  fc_l4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(9,11)  ;

  fc_r1_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(12,14)  ; 
  fc_r2_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(15,17)  ; 
  fc_r3_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(18,20)  ; 
  fc_r4_hand_filt(3, 0.0) , // = FC_HANDS_FILTERED.subVector(21,23)  ;   
    
  d_EE_r_des(6, 0.0 ) ,
  d_EE_l_des(6 , 0.0) ,  
    
  T_w_aw_0(4,4) , // = locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  T_w_waist_0(4,4) , //    = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0(4,4) , //     = model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0(4,4) , //     = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0(4,4) , //     = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0(4,4) , //     = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0(4,4) , //  = model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0(4,4) , //     = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0(4,4) , //     = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0(4,4) , //     = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0(4,4) , //     = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0(4,4) , //   = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0(4,4) , //   = model.iDyn3_model.getPosition( r_hand_index ) ;   

  T_w_l_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(l_wrist_index) ;
  T_w_l1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  T_w_l2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  T_w_l3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  T_w_l4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  T_w_r_wrist_0(4,4) , //  = model.iDyn3_model.getPosition(r_wrist_index) ;
  T_w_r1_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  T_w_r2_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  T_w_r3_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  T_w_r4_hand_0(4,4) , //  = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  T_waist_w_0(4,4) , //    = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  T_l_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_l_c4_0) ;    
    
  T_r_ankle_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c1_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  T_r_c2_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0(4,4) , //     = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  T_l_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  T_l1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  T_l2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  T_l3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  T_l4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  T_r_wrist_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  T_r1_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  T_r2_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  T_r3_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  T_r4_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  T_l_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  T_r_hand_w_0(4,4) , //  = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  T_aw_l_c1_0(4,4) , //  = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  T_aw_l_c2_0(4,4) , //  = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l_c3_0(4,4) , //  = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  T_aw_l_c4_0(4,4) , //  = T_aw_w_0 * T_w_l_c4_0 ;

  T_aw_r_c1_0(4,4) , //  = T_aw_w_0 * T_w_r_c1_0 ;
  T_aw_r_c2_0(4,4) , //  = T_aw_w_0 * T_w_r_c2_0 ;
  T_aw_r_c3_0(4,4) , //  = T_aw_w_0 * T_w_r_c3_0 ;
  T_aw_r_c4_0(4,4) , //  = T_aw_w_0 * T_w_r_c4_0 ; 

  T_aw_l1_hand_0(4,4) , //  = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  T_aw_l2_hand_0(4,4) , //  = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l3_hand_0(4,4) , //  = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  T_aw_l4_hand_0(4,4) , //  = T_aw_w_0 * T_w_l4_hand_0 ;

  T_aw_r1_hand_0(4,4) , //  = T_aw_w_0 * T_w_r1_hand_0 ;
  T_aw_r2_hand_0(4,4) , //  = T_aw_w_0 * T_w_r2_hand_0 ;
  T_aw_r3_hand_0(4,4) , //  = T_aw_w_0 * T_w_r3_hand_0 ;
  T_aw_r4_hand_0(4,4) , //  = T_aw_w_0 * T_w_r4_hand_0 ;   
    
  //-----------------------------------------------------
  J_l_c1_mix_0( 6, ( size_q + 6 ) ) , //
  J_l_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_r_c1_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c2_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c3_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_c4_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  
  J_l1_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_l4_hand_mix_0( 6, ( size_q + 6 ) ) ,//robot.getNumberOfKinematicJoints() + 6 ) ) ;

  J_r1_hand_mix_0( 6, ( size_q + 6 ) ) , // robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r2_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r3_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ; 
  J_r4_hand_mix_0( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;

  //-------------------------------------------
  
  J_l_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  J_l_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  J_l_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  J_l_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  J_r_c1_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  J_r_c2_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  J_r_c3_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  J_r_c4_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  J_l_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  J_r_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  J_l1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  J_l2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  J_l3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  J_l4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  J_r1_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  J_r2_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  J_r3_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  J_r4_hand_body_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r_c1_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r_c2_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r_c3_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r_c4_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  J_aw_l1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;
 
  J_aw_r1_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r2_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r3_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r4_hand_spa_0( 6, ( size_q + 6 ) ) , // = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
  //------------------------------------------------
  Q_aw_l_c1(size_q+ 6, size_q + 6)   , //= Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  Q_aw_l_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;

  Q_aw_r_c1(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c2(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r_c3(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r_c4(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c4_spa_0, T_aw_r_c4_0, fc_r_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  
  Q_aw_r1_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c1_spa_0, T_aw_r_c1_0, fc_r_c1_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r2_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c2_spa_0, T_aw_r_c2_0, fc_r_c2_filt ) ; //(size_q+ 6, size_q + 6) ;
  Q_aw_r3_hand(size_q+ 6, size_q + 6)   , // = Q_ci(J_aw_r_c3_spa_0, T_aw_r_c3_0, fc_r_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_r4_hand(size_q+ 6, size_q + 6)   ,

  Q_aw_l_tot(size_q+ 6, size_q + 6)   , // = Q_aw_l_c1 + Q_aw_l_c2 + Q_aw_l_c3 + Q_aw_l_c4;
  Q_aw_r_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_r_hand_tot(size_q+ 6, size_q + 6)   , // = Q_aw_r_c1 + Q_aw_r_c2 + Q_aw_r_c3 + Q_aw_r_c4;
  Q_aw_c(size_q+ 6, size_q + 6)   , // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_cont( 6 , 6) , // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_cont( 6 , size_q ) , //  = Q_aw_c.submatrix( 0  , 5,  6,  (Q_aw_c.cols()-1)  ) ;
  Q_aw_c_f_rh(size_q+ 6, size_q + 6)   , // =  Q_aw_l_tot + Q_aw_r_tot ;  
  U_aw_s_c_f_rh( 6 , 6) , // = Q_aw_c.submatrix( 0 ,  5 , 0, 5) ;     
  Q_aw_s_c_f_rh( 6 , size_q ) ,  //  = Q_aw_c.submatrix( 0  , 5,  6,
  
  //------------------------------------------------------------------
  d_fc_des_to_world(size_fc, 0.0)  ,

  T_l_c1_r_c1_loop(4,4) ,
  T_r_c1_l_c1_loop(4,4) ,
//   T_l_c1_r_c1_loop.zero() ,
//   T_r_c1_l_c1_loop.zero() ,  
  J_com_w( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_com_w_redu( 3,  ( size_q + 6 )  ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_aw( 3,  ( size_q + 6 ) ) , //( robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_com_waist( 3,  ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ))   ;
  J_r_c1_aw( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  J_l_c1_aw( 6, ( size_q + 6 ) ) , //robot.getNumberOfKinematicJoints() + 6 ) ) ;
  q_ref_ToMove( size_q  , 0.0 )  // 
  
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
    
    left_arm_config_0.resize(left_arm_joints);
    right_arm_config_0.resize(right_arm_joints);
    torso_config_0.resize(torso_joints);
    left_leg_config_0.resize(left_leg_joints);
    right_leg_config_0.resize(right_leg_joints);

    left_arm_config_1.resize(left_arm_joints);
    right_arm_config_1.resize(right_arm_joints);
    torso_config_1.resize(torso_joints);
    left_leg_config_1.resize(left_leg_joints);
    right_leg_config_1.resize(right_leg_joints);
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
 //   ////std::cout << " FC_WINDOW  =  "  << std::endl << FC_WINDOW.toString() << std::endl  ; 
 //   ////std::cout << " FC_DES  =  "  << std::endl << FC_DES.toString() << std::endl  ;     
 //   ////std::cout << " FC_DES_LEFT_sensor  =  "  << std::endl << FC_DES_LEFT_sensor.toString() << std::endl  ; 
 //   ////std::cout << " FC_DES_RIGHT_sensor  =  "  << std::endl << FC_DES_RIGHT_sensor.toString() << std::endl  ;     
 //   ////std::cout << " FC_SUM  =  "  << std::endl << FC_SUM.toString() << std::endl  ; 
    //
    // Hands
    FC_HANDS_FILTERED = FC_HANDS_DES ;
    for(int t=0; t<WINDOW_size ; t++ )
    {
      FC_HANDS_WINDOW.setCol(t, FC_HANDS_DES )   ;
    }
    FC_HANDS_SUM = WINDOW_size * FC_HANDS_DES ;
  //  //std::cout << " FC_HANDS_WINDOW  =  "  << std::endl << FC_HANDS_WINDOW.toString() << std::endl  ; 
  //  //std::cout << " FC_HANDS_DES  =  "  << std::endl << FC_HANDS_DES.toString() << std::endl  ;     
   // //std::cout << " FC_DES_LEFT_HAND_sensor  =  "  << std::endl << FC_DES_LEFT_HAND_sensor.toString() << std::endl  ; 
   // //std::cout << " FC_DES_RIGHT_HAND_sensor  =  "  << std::endl << FC_DES_RIGHT_HAND_sensor.toString() << std::endl  ;     
  //  //std::cout << " FC_HANDS_SUM  =  "  << std::endl << FC_HANDS_SUM.toString() << std::endl  ; 
//     if(robot.idynutils.getRobotName() == "bigman") {
//     mg = 1200;
//     }

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
    
    //------------------------------------------------------------
    ph->linkParam( PARAM_ID_LEFT_LEG_0 , left_leg_config_0.data() );
    ph->linkParam( PARAM_ID_RIGHT_LEG_0, right_leg_config_0.data() );
    ph->linkParam( PARAM_ID_TORSO_0    , torso_config_0.data() );
    ph->linkParam( PARAM_ID_LEFT_ARM_0 , left_arm_config_0.data() );
    ph->linkParam( PARAM_ID_RIGHT_ARM_0, right_arm_config_0.data() );
    
    //------------------------------------------------------------
    ph->linkParam( PARAM_ID_LEFT_LEG_1 , left_leg_config_1.data() );
    ph->linkParam( PARAM_ID_RIGHT_LEG_1, right_leg_config_1.data() );
    ph->linkParam( PARAM_ID_TORSO_1    , torso_config_1.data() );
    ph->linkParam( PARAM_ID_LEFT_ARM_1 , left_arm_config_1.data() );
    ph->linkParam( PARAM_ID_RIGHT_ARM_1, right_arm_config_1.data() );
}


bool locoman_control_thread::custom_init()
{    
    struct sched_param thread_param;
    
    thread_param.sched_priority = 99;
    //pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    link_locoman_params();
    
    model.setFloatingBaseLink("Waist");
    yarp::sig::Vector q_current(robot.getNumberOfKinematicJoints(),0.0) ; // = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
   robot.setPositionDirectMode();    
   
   
   //--------------------------------------------------------------------------------------------------------------
   // YARP Port
   
   if(!to_service_1.open(std::string("/" + get_module_prefix() + "/test"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/test") << std::endl;
        return false;    }
   yarp::os::Network::connect( std::string("/" + get_module_prefix() + "/test"), "/locoman_service_2/test");  

    
   if(!sending_q.open(std::string("/" + get_module_prefix() + "/sending_q"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/sending_q") << std::endl;
        return false; } 
   yarp::os::Network::connect( std::string("/" + get_module_prefix() + "/sending_q"), "/locoman_service_2/receiving_q");       
    
   if(!sending_fc.open(std::string("/" + get_module_prefix() + "/sending_fc"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/sending_fc") << std::endl;
        return false; } 
   yarp::os::Network::connect( std::string("/" + get_module_prefix() + "/sending_fc"), "/locoman_service_2/receiving_fc");       
    
   
   
   if(!from_service_1.open(std::string("/" + get_module_prefix() + "/test_input"))) {
        std::cout << "ERROR: cannot open YARP port " << std::string(get_module_prefix() + "/test_input") << std::endl;
        return false;  }
  if(!yarp::os::Network::connect(  "/locoman_service_2/test_output", std::string("/" + get_module_prefix() + "/test_input"))){
      std::cout << "ERROR connecting YARP ports " << std::endl ;
      return false ;
  }
      
   from_service_1_initted = false;
   
   //----------------------------------------------------------------------------------------------------------------
    
  
   
  //-----------------------------
  yarp::sig::Vector q_motor_0 =locoman::utils::senseMotorPosition(robot, flag_robot) ;                  
  yarp::sig::Vector q_des(robot.getNumberOfKinematicJoints() ) ;     
  yarp::sig::Vector d_q_des(robot.getNumberOfKinematicJoints() ) ;     
  yarp::sig::Vector q_motor_act = locoman::utils::senseMotorPosition(robot, flag_robot) ; 
  
  double steps = 380.0 ;  // slower on the real robot for safety 
  if(flag_simulator){steps = 100.0 ; } //faster on the simulator
  //
//   //---------------------------------------------------------
//   // A First Homing
//   robot.fromRobotToIdyn29(  right_arm_configuration ,//right_arm_configuration
//                             left_arm_configuration  , //left_arm_configuration
//                             torso_configuration     ,
//                             right_leg_configuration ,  //right_leg_configuration
//                             left_leg_configuration  ,  //left_leg_configuration
//                             q_des                   );   
    robot.fromRobotToIdyn29(  right_arm_config_0 ,
                            left_arm_config_0  ,
                            torso_config_0     ,
                            right_leg_config_0 ,
                            left_leg_config_0  ,
                            q_des              );   
  
  q_motor_0 = locoman::utils::senseMotorPosition(robot, flag_robot) ; // this function uses manually imposed joint stiffness values
  d_q_des = (q_des - q_motor_0); //
  locoman::utils::Joint_Trajectory(robot, flag_robot, q_motor_0, q_des, steps  ) ;   
  std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl;     
  usleep(100*1000) ; // usleep(milliseconds*1000)
//   
// //   //-------------------------------------------

// //         // robot.left_arm.move(q_ref_ToMove_left_arm);  

 //------------------------------------------------------------------------------------------------------------------

  // Defining Various Parameters
  //yarp::sig::Vector zero_3(3, 0.0) ;
  //yarp::sig::Matrix Zeros_6_6(6,6) ;
  Zeros_6_6.zero();
  //yarp::sig::Matrix Eye_6(6,6) ;
  Eye_6.eye() ;
  //yarp::sig::Matrix Eye_3(3,3) ;
  Eye_3.eye() ;
  Eye_4.eye() ;
  //yarp::sig::Matrix B( 6 , 3 ) ;
  B.zero();
  B.setSubmatrix( Eye_3 , 0 , 0 ) ;
  
  /*size_q = robot.getNumberOfKinematicJoints()*/ ; // getNumberOfKinematicJoints = 31 (29 + 2 for the hands)
  size_u = 6 ;
  size_fc = 24 ; // number of fc components on the feet
  kc = 1E6 ;
  Kq = locoman::utils::getKq(robot) ;
  Kc.eye() ;
  Kc = kc*Kc ;    //   ;// 1E6*Kc ;    1E8*Kc ;  
  Kc_f_rh.eye() ;
  Kc_f_rh = (kc/10)*Kc_f_rh ;
  
 
  // 
  
  waist_index   = model.iDyn3_model.getLinkIndex("Waist");  
  l_ankle_index = model.iDyn3_model.getLinkIndex("l_leg_ft") ; // sensors are placed in *_ankle in the model
//     int l_ankle_index = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
  l_c1_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link") ;
  l_c2_index    = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
  l_c3_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link") ;
  l_c4_index    = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

  r_ankle_index = model.iDyn3_model.getLinkIndex("r_leg_ft") ;
//     int r_ankle_index = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link") ;
  r_c1_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
  r_c2_index    = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
  r_c3_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
  r_c4_index    = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

  l_hand_index  = model.iDyn3_model.getLinkIndex("LSoftHand");
  r_hand_index  = model.iDyn3_model.getLinkIndex("RSoftHand");    

  l_wrist_index  = model.iDyn3_model.getLinkIndex("l_arm_ft") ;
  l_hand_c1_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link");  // r_foot_upper_left_link
  l_hand_c2_index = model.iDyn3_model.getLinkIndex("l_hand_lower_right_link");  // r_foot_upper_right_link
  l_hand_c3_index = model.iDyn3_model.getLinkIndex("l_hand_upper_left_link");   // r_foot_lower_left_link
  l_hand_c4_index = model.iDyn3_model.getLinkIndex("l_hand_lower_left_link");  // r_foot_lower_right_link
//     
    
  r_wrist_index   = model.iDyn3_model.getLinkIndex("r_arm_ft") ;
  r_hand_c1_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link");  // r_foot_upper_left_link
  r_hand_c2_index = model.iDyn3_model.getLinkIndex("r_hand_lower_right_link");  // r_foot_upper_right_link
  r_hand_c3_index = model.iDyn3_model.getLinkIndex("r_hand_upper_left_link");   // r_foot_lower_left_link
  r_hand_c4_index = model.iDyn3_model.getLinkIndex("r_hand_lower_left_link");  // r_foot_lower_right_link
     
  map_l_fcToSens =  locoman::utils::fConToSens( l_ankle_index, 
                                                l_c1_index  , 
                                                l_c2_index  ,                                
                                                l_c3_index  , 
                                                l_c4_index,
                                                model
                                                ) ;
                            
  map_r_fcToSens =  locoman::utils::fConToSens( r_ankle_index, 
                                                r_c1_index, 
                                                r_c2_index,
                                                r_c3_index, 
                                                r_c4_index,
                                                model
                                                ) ;
                      // 
  map_l_hand_fcToSens = locoman::utils::fConToSens( l_wrist_index, 
                                                    l_hand_c1_index, 
                                                    l_hand_c2_index,
                                                    l_hand_c3_index, 
                                                    l_hand_c4_index,
                                                    model
                                                    ) ;
    
  map_r_hand_fcToSens = locoman::utils::fConToSens( r_wrist_index, 
                                                    r_hand_c1_index, 
                                                    r_hand_c2_index,
                                                    r_hand_c3_index, 
                                                    r_hand_c4_index ,
                                                    model
                                                    ) ;
                                                                        
  map_l_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_fcToSens, 1E-10 ) ; //*  ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_fcToSens, 1E-10 ) ; // *  ft_r_ankle  ;  */// yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
  map_l_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_l_hand_fcToSens, 1E-10 ) ; //*  ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  map_r_hand_fcToSens_PINV = locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10 ) ; //    
//   //-----------------------------------------------------
//   // Offset Evaluation

  int dim_offeset = 1000    ;
   
  yarp::sig::Matrix offset_window(robot.getNumberOfKinematicJoints(), dim_offeset);
  for(int k=0; k<dim_offeset ; k++ ){
       q_current += locoman::utils::sense_position_no_hands(robot); //if sense returns motorPosition       
       usleep(1*1000) ;  
     }
     
  q_current = q_current/dim_offeset ;  
 
  yarp::sig::Vector q_motor_init = q_des  ;
//    robot.fromRobotToIdyn(  right_arm_configuration ,
//                            left_arm_configuration  ,
//                            torso_configuration     ,
//                            right_leg_configuration ,
//                            left_leg_configuration  ,
//                            q_motor_init            )  ;
  q_offset = q_motor_init - q_current ;
  //q_current_open_loop = q_current + q_offset; 
  
   //----------------------------------------------------------
    // // -----------------------------------------------------------
  // // Evaluating offset for contact forces
  int dim_fc_offset = 1000 ;  
 
  for(int k=0; k<dim_fc_offset ; k++ ){
  robot.senseftSensor("l_leg_ft", ft_l_ankle) ;
  robot.senseftSensor("r_leg_ft", ft_r_ankle) ;
  fc_offset_left  += map_l_fcToSens_PINV * ft_l_ankle ;
  fc_offset_right += map_r_fcToSens_PINV * ft_r_ankle ;
  //
  robot.senseftSensor("l_arm_ft", ft_l_wrist) ;
  robot.senseftSensor("r_arm_ft", ft_r_wrist) ;  
  fc_offset_left_hand  += map_l_hand_fcToSens_PINV * ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  fc_offset_right_hand += map_r_hand_fcToSens_PINV * ft_r_wrist  ; 
  //
  // std::cout << " fc_offset_left = " << fc_offset_left.toString() << std::endl;
 //std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl;
  usleep(1*1000) ; 
  }

  fc_offset_left       = fc_offset_left/ dim_fc_offset ;
  fc_offset_right      = fc_offset_right/ dim_fc_offset ;
  fc_offset_left_hand  = fc_offset_left_hand/ dim_fc_offset ;
  fc_offset_right_hand = fc_offset_right_hand/ dim_fc_offset ;
 
 std::cout << " fc_offset_left = " << fc_offset_left.toString() << std::endl;
 std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl;
  // // -----------------------------------------------------------

  //---------------------------------------------------------------------------------------------
  // fc offset evaluation
//   fc_offset_left(size_fc/2, 0.0) ;
//   fc_offset_right(size_fc/2,0.0) ;



// // //--------------------------------------------------------
//   fc_l_c1_filt(3) ; //= FC_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
//   fc_l_c2_filt(3) ; //= FC_FILTERED.subVector(3,5)  ;
//   fc_l_c3_filt(3) ; //= FC_FILTERED.subVector(6,8)  ;
//   fc_l_c4_filt(3) ; //= FC_FILTERED.subVector(9,11)  ;
// 
//   fc_r_c1_filt(3) ; //= FC_FILTERED.subVector(12,14)  ; 
//   fc_r_c2_filt(3) ; //= FC_FILTERED.subVector(15,17)  ; 
//   fc_r_c3_filt(3) ; //= FC_FILTERED.subVector(18,20)  ; 
//   fc_r_c4_filt(3) ; //= FC_FILTERED.subVector(21,23)  ; 
//   
//   fc_l1_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
//   fc_l2_hand_filt(3) ; // = FC_HANDS_FILTERED.subVector(3,5)  ;
//   fc_l3_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(6,8)  ;
//   fc_l4_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(9,11)  ;
// 
//   fc_r1_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(12,14)  ; 
//   fc_r2_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(15,17)  ; 
//   fc_r3_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(18,20)  ; 
//   fc_r4_hand_filt(3) ; //= FC_HANDS_FILTERED.subVector(21,23)  ;   
// 
//   std::cout << "qui 4" << std::cout ;
  T_w_aw_0.zero()     ; //= locoman::utils::AW_world_posture(model, robot) ;
  T_aw_w_0.zero()  ; //= locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  T_w_waist_0.zero()   ; // = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0.zero()   ; // = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0.zero()    ; //= model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0.zero()    ; // = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0.zero()   ; // = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0.zero()    ; // = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0.zero()  ; //= model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0.zero()    ; // = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0.zero()    ; // = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0.zero()   ; // = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0.zero()    ; // = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0.zero()  ; //= model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0.zero() ;   

  T_w_l_wrist_0.zero()  ;
  T_w_l1_hand_0.zero()  ;
  T_w_l2_hand_0.zero()  ;
  T_w_l3_hand_0.zero()  ;
  T_w_l4_hand_0.zero()  ;
    
  T_w_r_wrist_0.zero()  ;
  T_w_r1_hand_0.zero()  ;
  T_w_r2_hand_0.zero()  ;
  T_w_r3_hand_0.zero()  ;
  T_w_r4_hand_0.zero()  ;
  
  // -----------------------------------------------------------------------
  T_waist_w_0.zero()  ;
  T_l_ankle_w_0.zero()  ;
  T_l_c1_w_0.zero()  ;
  T_l_c2_w_0.zero()  ;
  T_l_c3_w_0.zero()  ;
  T_l_c4_w_0.zero()  ;
    
  T_r_ankle_w_0.zero()  ;
  T_r_c1_w_0.zero()  ;
  T_r_c2_w_0.zero()  ;
  T_r_c3_w_0.zero()  ;
  T_r_c4_w_0.zero()  ;

  T_l_wrist_w_0.zero()  ;
  T_l1_hand_w_0.zero()  ;
  T_l2_hand_w_0.zero()  ;
  T_l3_hand_w_0.zero()  ;
  T_l4_hand_w_0.zero()  ;
      
  T_r_wrist_w_0.zero()  ;
  T_r1_hand_w_0.zero()  ;
  T_r2_hand_w_0.zero()  ;
  T_r3_hand_w_0.zero()  ;
  T_r4_hand_w_0.zero()  ;

  //---------------------------------------------------------------------
  
  T_l_hand_w_0.zero()  ;
  T_r_hand_w_0.zero()  ;
  
  T_aw_l_c1_0.zero()  ;
  T_aw_l_c2_0.zero()  ;
  T_aw_l_c3_0.zero()  ;
  T_aw_l_c4_0.zero()  ;

  T_aw_r_c1_0.zero()  ;
  T_aw_r_c2_0.zero()  ;
  T_aw_r_c3_0.zero()  ;
  T_aw_r_c4_0.zero()  ;

  T_aw_l1_hand_0.zero()  ;
  T_aw_l2_hand_0.zero()  ;
  T_aw_l3_hand_0.zero()  ;
  T_aw_l4_hand_0.zero()  ;

  T_aw_r1_hand_0.zero()  ;
  T_aw_r2_hand_0.zero()  ;
  T_aw_r3_hand_0.zero()  ;
  T_aw_r4_hand_0.zero()  ;
    
   //  Jacobian Matrices 
  J_l_c1_mix_0.zero()  ;
  J_l_c2_mix_0.zero()  ;
  J_l_c3_mix_0.zero()  ;
  J_l_c4_mix_0.zero()  ;
  
  J_r_c1_mix_0.zero()  ;
  J_r_c2_mix_0.zero()  ;
  J_r_c3_mix_0.zero()  ;
  J_r_c4_mix_0.zero()  ;
  
  J_l_hand_mix_0.zero()  ;
  J_r_hand_mix_0.zero()  ;  
  J_l1_hand_mix_0.zero()  ;
  J_l2_hand_mix_0.zero()  ;
  J_l3_hand_mix_0.zero()  ;
  J_l4_hand_mix_0.zero()  ;

  J_r1_hand_mix_0.zero()  ;
  J_r2_hand_mix_0.zero()  ;
  J_r3_hand_mix_0.zero()  ;
  J_r4_hand_mix_0.zero()  ;

 // -------------------------------------------
  
  J_l_c1_body_0.zero()  ;
  J_l_c2_body_0.zero()  ;
  J_l_c3_body_0.zero()  ;
  J_l_c4_body_0.zero()  ;

  J_r_c1_body_0.zero()  ;
  J_r_c2_body_0.zero()  ;
  J_r_c3_body_0.zero()  ;
  J_r_c4_body_0.zero()  ;

  J_l_hand_body_0.zero()  ;
  J_r_hand_body_0.zero()  ;

  J_l1_hand_body_0.zero()  ;
  J_l2_hand_body_0.zero()  ;
  J_l3_hand_body_0.zero()  ;
  J_l4_hand_body_0.zero()  ;

  J_r1_hand_body_0.zero()  ;
  J_r2_hand_body_0.zero()  ;
  J_r3_hand_body_0.zero()  ;
  J_r4_hand_body_0.zero()  ;
  
  ////---------------------------------------------------------------------------------------------------------------------------------------------------------------  
  J_aw_l_c1_spa_0.zero()  ;
  J_aw_l_c2_spa_0.zero()  ;
  J_aw_l_c3_spa_0.zero()  ;
  J_aw_l_c4_spa_0.zero()  ;

  J_aw_r_c1_spa_0.zero()  ;
  J_aw_r_c2_spa_0.zero()  ;
  J_aw_r_c3_spa_0.zero()  ;
  J_aw_r_c4_spa_0.zero()  ;

  J_aw_l1_hand_spa_0.zero()  ;
  J_aw_l2_hand_spa_0.zero()  ;
  J_aw_l3_hand_spa_0.zero()  ;
  J_aw_l4_hand_spa_0.zero()  ;
 
  J_aw_r1_hand_spa_0.zero()  ;
  J_aw_r2_hand_spa_0.zero()  ;
  J_aw_r3_hand_spa_0.zero()  ;
  J_aw_r4_hand_spa_0.zero()  ;
  
  ////------------------------------------------------
  Q_aw_l_c1.zero()  ;
  Q_aw_l_c2.zero()  ;
  Q_aw_l_c3.zero()  ;
  Q_aw_l_c4.zero()  ;

  Q_aw_r_c1.zero()  ;
  Q_aw_r_c2.zero()  ;
  Q_aw_r_c3.zero()  ;
  Q_aw_r_c4.zero()  ;
  
  Q_aw_r1_hand.zero()  ;
  Q_aw_r2_hand.zero()  ;
  Q_aw_r3_hand.zero()  ;
  Q_aw_r4_hand.zero()  ;
    
  Q_aw_l_tot.zero()  ;
  Q_aw_r_tot.zero()  ;
  Q_aw_r_hand_tot.zero()  ;
  Q_aw_c.zero()  ;
  U_aw_s_cont.zero()  ;
  Q_aw_s_cont.zero()  ;
  Q_aw_c_f_rh.zero()  ;
  U_aw_s_c_f_rh.zero()  ;
  Q_aw_s_c_f_rh.zero()  ;
  //// ---------------------------------------------------------
// 
  d_fc_des_to_world.zero();  ;
// 
  T_l_c1_r_c1_loop.zero()  ;
  T_r_c1_l_c1_loop.zero()  ;
  T_l_c1_r_c1_loop.zero() ;
  T_r_c1_l_c1_loop.zero() ;  
  J_com_w.zero()  ;
  J_com_w_redu.zero()  ;
  J_com_aw.zero()  ;
  J_com_waist.zero()  ;
  J_r_c1_aw.zero()  ;
  J_l_c1_aw.zero()  ;
  q_ref_ToMove( size_q   ) ; // 
  
  // // 
    bool traject ;
    char vai ;
    std::cout << " waiting... " << std::endl ;
    std::cin >> vai ;
    // //   //------------------------------------------------------------
// //   //  Homing to config  _0
//   robot.fromRobotToIdyn29(  right_arm_config_0 ,
//                             left_arm_config_0  ,
//                             torso_config_0     ,
//                             right_leg_config_0 ,
//                             left_leg_config_0  ,
//                             q_des              );    
//   q_motor_0 = locoman::utils::senseMotorPosition(robot, flag_robot) ; // this function uses manually imposed joint stiffness values
//   d_q_des = (q_des - q_motor_0); //
//   traject = locoman::utils::Joint_Trajectory(robot, flag_robot, q_motor_0, q_des, steps  ) ;   
//   std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl;     
//   usleep(100*1000) ; // usleep(milliseconds*1000)
// //   //----------------------------------------------------------------
//     std::cout << " waiting... " << std::endl ;
//     std::cin >> vai ;
// // //   //------------------------------------------------------------ 
// // //   //  Homing to config  _1
//   robot.fromRobotToIdyn29(  right_arm_config_1 ,
//                             left_arm_config_1  ,
//                             torso_config_1     ,
//                             right_leg_config_1 ,
//                             left_leg_config_1  ,
//                             q_des              );    
//   q_motor_0 = locoman::utils::senseMotorPosition(robot, flag_robot) ; // this function uses manually imposed joint stiffness values
//   d_q_des = (q_des - q_motor_0); //
//   traject = locoman::utils::Joint_Trajectory(robot, flag_robot, q_motor_0, q_des, steps  ) ;   
//   std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl;     
//   usleep(100*1000) ; // usleep(milliseconds*1000)
//   //----------------------------------------------------------------
//   
  fc_offset_left_hand.zero();
  fc_offset_right_hand.zero();
  
  for(int k=0; k<dim_fc_offset ; k++ ){
//   robot.senseftSensor("l_leg_ft", ft_l_ankle) ;
//   robot.senseftSensor("r_leg_ft", ft_r_ankle) ;
//   fc_offset_left  += map_l_fcToSens_PINV * ft_l_ankle ;
//   fc_offset_right += map_r_fcToSens_PINV * ft_r_ankle ;
  //
  robot.senseftSensor("l_arm_ft", ft_l_wrist) ;
  robot.senseftSensor("r_arm_ft", ft_r_wrist) ;  
  fc_offset_left_hand  += map_l_hand_fcToSens_PINV * ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
  fc_offset_right_hand += map_r_hand_fcToSens_PINV * ft_r_wrist  ; 
  //
  // std::cout << " fc_offset_left = " << fc_offset_left.toString() << std::endl;
 //std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl;
  usleep(1*1000) ; 
  }

  fc_offset_left       = fc_offset_left/ dim_fc_offset ;
  fc_offset_right      = fc_offset_right/ dim_fc_offset ;
  fc_offset_left_hand  = fc_offset_left_hand/ dim_fc_offset ;
  fc_offset_right_hand = fc_offset_right_hand/ dim_fc_offset ;
 
 std::cout << " fc_offset_left = " << fc_offset_left.toString() << std::endl;
 std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl;
  
//       std::cout << " waiting... " << std::endl ;
//     std::cin >> vai ;
  //-------------------------------------------------------------------
  std::cout << " Beginning the -run- loop...  " << std::endl ;
  return true ;

}


void locoman_control_thread::run()
{     
   double tic = locoman::utils::Tic() ;

   std::cout << " loop_counter = " <<  loop_counter << std::endl; 

//------------------------------------------------------------------------------------------------------------------
////     robot.setReferenceSpeed(max_vel) ;

    //std::cout << robot.getNumberOfKinematicJoints() << " getNumberOfKinematicJoints " << std::endl;
    yarp::sig::Vector q_current = locoman::utils::sense_position_no_hands(robot);
    std::cout << "q_current = " << q_current.toString() << std::endl ;
    
    //std::cout << " sense size : " << q_current.size() << std::endl;
    // robot.idynutils.updateiDyn3Model( q_current, true ); //update model first
    
    yarp::sig::Vector tau_current = locoman::utils::sense_torque_no_hands(robot) ;
    yarp::sig::Vector q_motor_side(size_q ) ; //robot.getNumberOfKinematicJoints() ) ;
    // using the q_ in open loop
    //robot.idynutils.updateiDyn3Model( q_current_open_loop, true );   // q_current_open_loop    //  q_current

    yarp::sig::Vector q_senseRefFeedback =  locoman::utils::senseMotorPosition( robot, flag_robot ) ;            // Ok, verified! We can use it instead of q_sensed + offset                 
    std::cout << "q_senseRefFeedback = " << q_senseRefFeedback.toString() << std::endl ;
   
   robot.idynutils.updateiDyn3Model( q_senseRefFeedback, true ); //update model first

    
    //----------------------------------------------------------------------------------------
    // YARP Ports
    //
    yarp::sig::Vector& v_to_service_1 = to_service_1.prepare();
    v_to_service_1.resize( q_senseRefFeedback.size() ) ;
    v_to_service_1 = q_senseRefFeedback ;
    to_service_1.write() ;
        
    yarp::sig::Vector &sending_q_vect = sending_q.prepare() ;
    sending_q_vect.resize( q_senseRefFeedback.size() ) ;
    sending_q_vect = q_senseRefFeedback ;
    sending_q.write() ;
    
    
     //-----------------------------------------------------------------------------------------------
    
    //--------------------------------------------------------------------//
    //Getting Force/Torque Sensor Measures

    if(!robot.senseftSensor("l_leg_ft", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    if(!robot.senseftSensor("r_leg_ft", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;     
    if(!robot.senseftSensor("l_arm_ft", ft_l_wrist)) std::cout << "ERROR READING SENSOR l_wrist" << std::endl; 
    if(!robot.senseftSensor("r_arm_ft", ft_r_wrist)) std::cout << "ERROR READING SENSOR r_wrist" << std::endl;     
    
//     std::cout << " ft_l_ankle = " << ft_l_ankle.toString() << std::endl;
//     std::cout << " ft_r_ankle = " << ft_r_ankle.toString() << std::endl;
//     std::cout << " ft_l_wrist = " << ft_l_wrist.toString() << std::endl;
//     std::cout << " ft_r_wrist = " << ft_r_wrist.toString() << std::endl;

    yarp::sig::Vector fc_l_c_to_robot = map_l_fcToSens_PINV *  ft_l_ankle  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_c_to_robot = map_r_fcToSens_PINV *  ft_r_ankle  ;    
    yarp::sig::Vector fc_l_c_to_world =  - 1.0 * fc_l_c_to_robot     ;
    yarp::sig::Vector fc_r_c_to_world =  - 1.0 * fc_r_c_to_robot     ;
    
//     std::cout << " fc_l_c_to_world = " << fc_l_c_to_world.toString() << std::endl;
//     std::cout << " fc_offset_left = " << fc_offset_left.toString() << std::endl;
//     std::cout << " fc_r_c_to_world = " << fc_r_c_to_world.toString() << std::endl;
//     std::cout << " fc_offset_right = " << fc_offset_right.toString() << std::endl; 

    fc_l_c_to_world =  - 1.0 * fc_l_c_to_robot  + fc_offset_left   ;
    fc_r_c_to_world =  - 1.0 * fc_r_c_to_robot  + fc_offset_right  ;

//     std::cout << " fc_l_c_to_world = " << fc_l_c_to_world.toString() << std::endl;
//     std::cout << " fc_r_c_to_world = " << fc_r_c_to_world.toString() << std::endl; 
// 
        
    //---------------------------------------
    //
    if(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman"){  // Changing the sign again in case of walkman real robot
          fc_l_c_to_world  = -1.0*fc_l_c_to_world ;                  // in the walkman (real) robot the feet sensors provide to_wolrd measures
          fc_r_c_to_world  = -1.0*fc_r_c_to_world ;
    }
    
    yarp::sig::Vector fc_l_c_hand_to_world = map_l_hand_fcToSens_PINV * ft_l_wrist  ;  // yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_c_hand_to_world = map_r_hand_fcToSens_PINV * ft_r_wrist  ;  // yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
   

/*    yarp::sig::Vector fc_l_c_hand_to_robot =  - 1.0 * fc_l_c_hand_to_world   ;
    yarp::sig::Vector fc_r_c_hand_to_robot =  - 1.0 * fc_r_c_hand_to_world   ;  */  
    
    yarp::sig::Vector fc_hand_to_world_0(size_fc) ;
    fc_hand_to_world_0.setSubvector(0, fc_l_c_hand_to_world ) ;
    fc_hand_to_world_0.setSubvector(fc_l_c_hand_to_world.length(), fc_r_c_hand_to_world ) ;  
    if(flag_robot ==1  && robot.idynutils.getRobotName() == "bigman"){  // Changing the sign again in case of walkman real robot
          fc_hand_to_world_0  = -1.0*fc_hand_to_world_0 ;
          fc_hand_to_world_0  = -1.0*fc_hand_to_world_0 ;   // 
    }
    
    std::cout << " fc_r_c_hand_to_world = " << fc_r_c_hand_to_world.toString() << std::endl;

    
    fc_r_c_hand_to_world -= fc_offset_right_hand ;
    std::cout << " fc_r_c_hand_to_world = " << fc_r_c_hand_to_world.toString() << std::endl;

    
    
    fc_hand_to_world_0.setSubvector(12, fc_r_c_hand_to_world) ;
    
//     std::cout << " ft_r_wrist = " << ft_r_wrist.toString() << std::endl;    
//     std::cout << " fc_offset_right_hand = " << fc_offset_right_hand.toString() << std::endl;
// 
//     std::cout << " fc_r_c_hand_to_world = " << fc_r_c_hand_to_world.toString() << std::endl;
//     
//     
  
    yarp::sig::Vector fc_to_world_0(size_fc) ;
    fc_to_world_0.setSubvector(0, fc_l_c_to_world ) ;
    fc_to_world_0.setSubvector(fc_l_c_to_world.length(), fc_r_c_to_world ) ;    
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
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
//    std::cout << " loop_counter = " <<  loop_counter << std::endl; 
//    std::cout << " counter_window = " <<  counter_window << std::endl; 
//    std::cout << " FC_WINDOW = " <<  std::endl << (FC_WINDOW.submatrix(0,FC_WINDOW.rows()-1,(FC_WINDOW.cols()-5),(FC_WINDOW.cols()-1) )).toString() << std::endl;
//    std::cout << " FC_FILTERED_LEFT_sensor  =  "  << std::endl << FC_FILTERED_LEFT_sensor.toString() << std::endl  ; 
//    std::cout << " FC_FILTERED_RIGHT_sensor  =  "  << std::endl << FC_FILTERED_RIGHT_sensor.toString() << std::endl  ;     
//    std::cout << " FC_FILTERED = " <<  std::endl << FC_FILTERED.toString() << std::endl; 
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
//    std::cout << " FC_FILTERED_LEFT_HAND_sensor  =  "  << std::endl << FC_FILTERED_LEFT_HAND_sensor.toString() << std::endl  ; 
//    std::cout << " FC_FILTERED_RIGHT_HAND_sensor  =  "  << std::endl << FC_FILTERED_RIGHT_HAND_sensor.toString() << std::endl  ;
//    std::cout << " FC_FILTERED_RIGHT_HAND  =  "  << std::endl << FC_HANDS_FILTERED.subVector(12,23).toString() << std::endl  ;

    
   d_fc_des_to_world  = FC_DES - FC_FILTERED ; //  
   yarp::sig::Vector d_fc_r_hand  = FC_HANDS_DES.subVector(12,23) -  FC_HANDS_FILTERED.subVector(12,23) ;
   yarp::sig::Vector d_fc_des_f_r_hand(size_fc+size_fc/2,0.0) ;
    
    
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
    
    
    
  yarp::sig::Vector FC_to_send(size_fc*2,0.0);
  FC_to_send.setSubvector(0, FC_DES) ;
  FC_to_send.setSubvector(size_fc, FC_FILTERED) ;  
   
         // Yarp Port about contact forces 
   yarp::sig::Vector &sending_fc_vect = sending_fc.prepare() ;
   sending_fc_vect.resize( FC_to_send.size() ) ;
   sending_fc_vect = FC_to_send ;
   sending_fc.write() ;
    
    
    std::cout << "ricevendo ... " << std::endl ;
    if(!from_service_1_initted){   // if we are in the first loop...
        yarp::sig::Vector *data = from_service_1.read(); //porta bloccante
        if(data){ data_from_service_1 = *data; }
    }
    else {
        yarp::sig::Vector *data = from_service_1.read(false); //porta non bloccante
        if(data){ data_from_service_1 = *data; }
    };
        
//    yarp::sig::Vector *data = from_service_1.read(); //(false) porta non bloccante
//            if(data){ data_from_service_1 = *data; }
//    data_from_service_1 = *data; 
//    yarp::sig::Vector *data = from_service_1.read(!from_service_1_initted);
//      std::cout << "ricevuto ! " << std::cout ;
//      from_service_1_initted = true;
//      if(data){ data_from_service_1 = *data; }

     
     
     std::cout << "data_from_service_1 = " << data_from_service_1.toString() << std::endl ;
 
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   

   
   loop_counter++ ;   
  // End of Contact Forces Filtering
  //
  //-----------------------------------------------------------------------------------------
  // Contact Forces
  fc_l_c1_filt = FC_FILTERED.subVector(0,2) ;// - fc_offset_left.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l_c2_filt = FC_FILTERED.subVector(3,5) ;//- fc_offset_left.subVector(3,5)  ;
  fc_l_c3_filt = FC_FILTERED.subVector(6,8) ;//- fc_offset_left.subVector(6,8)  ;
  fc_l_c4_filt = FC_FILTERED.subVector(9,11);// - fc_offset_left.subVector(9,11) ;

  fc_r_c1_filt = FC_FILTERED.subVector(12,14) ;//- fc_offset_right.subVector(0,2) ; 
  fc_r_c2_filt = FC_FILTERED.subVector(15,17) ;//- fc_offset_right.subVector(3,5)  ; 
  fc_r_c3_filt = FC_FILTERED.subVector(18,20) ;//- fc_offset_right.subVector(6,8)  ; 
  fc_r_c4_filt = FC_FILTERED.subVector(21,23) ;//- fc_offset_right.subVector(9,11)  ; 
   
  fc_l1_hand_filt = FC_HANDS_FILTERED.subVector(0,2)  ;  // Applied from the robot to the world
  fc_l2_hand_filt = FC_HANDS_FILTERED.subVector(3,5)  ;
  fc_l3_hand_filt = FC_HANDS_FILTERED.subVector(6,8)  ;
  fc_l4_hand_filt = FC_HANDS_FILTERED.subVector(9,11)  ;

  fc_r1_hand_filt = FC_HANDS_FILTERED.subVector(12,14)  ; 
  fc_r2_hand_filt = FC_HANDS_FILTERED.subVector(15,17)  ; 
  fc_r3_hand_filt = FC_HANDS_FILTERED.subVector(18,20)  ; 
  fc_r4_hand_filt = FC_HANDS_FILTERED.subVector(21,23)  ;   
  
  //----------------------------------------------------------------------------------------- 
  // Defining the "Auxiliary World" Frame => {AW} // 
  
  T_w_aw_0 = locoman::utils::AW_world_posture(model, robot) ;
 
  //
  
  
  //---------------------------------------
  T_aw_w_0 = locoman::utils::iHomogeneous(T_w_aw_0) ;    

  //-------------------------------------------------------------------------------------------------------------    
  // Defining Useful Transformations
  T_w_waist_0   = model.iDyn3_model.getPosition(waist_index) ;  
  T_w_l_ankle_0 = model.iDyn3_model.getPosition(l_ankle_index) ;
  T_w_l_c1_0    = model.iDyn3_model.getPosition(l_c1_index)    ;    
  T_w_l_c2_0    = model.iDyn3_model.getPosition(l_c2_index)    ;  
  T_w_l_c3_0    = model.iDyn3_model.getPosition(l_c3_index)    ;
  T_w_l_c4_0    = model.iDyn3_model.getPosition(l_c4_index)    ;    
    
  T_w_r_ankle_0 = model.iDyn3_model.getPosition(r_ankle_index) ;
  T_w_r_c1_0    = model.iDyn3_model.getPosition(r_c1_index)    ;    
  T_w_r_c2_0    = model.iDyn3_model.getPosition(r_c2_index)    ;  
  T_w_r_c3_0    = model.iDyn3_model.getPosition(r_c3_index)    ;
  T_w_r_c4_0    = model.iDyn3_model.getPosition(r_c4_index)    ;   
    
  T_w_l_hand_0  = model.iDyn3_model.getPosition( l_hand_index ) ;
  T_w_r_hand_0  = model.iDyn3_model.getPosition( r_hand_index ) ;   

  T_w_l_wrist_0 = model.iDyn3_model.getPosition(l_wrist_index) ;
  T_w_l1_hand_0 = model.iDyn3_model.getPosition(l_hand_c1_index)    ;    
  T_w_l2_hand_0 = model.iDyn3_model.getPosition(l_hand_c2_index)    ;  
  T_w_l3_hand_0 = model.iDyn3_model.getPosition(l_hand_c3_index)    ;
  T_w_l4_hand_0 = model.iDyn3_model.getPosition(l_hand_c4_index)    ;    
    
  T_w_r_wrist_0 = model.iDyn3_model.getPosition(r_wrist_index) ;
  T_w_r1_hand_0 = model.iDyn3_model.getPosition(r_hand_c1_index)    ;    
  T_w_r2_hand_0 = model.iDyn3_model.getPosition(r_hand_c2_index)    ;  
  T_w_r3_hand_0 = model.iDyn3_model.getPosition(r_hand_c3_index)    ;
  T_w_r4_hand_0 = model.iDyn3_model.getPosition(r_hand_c4_index)    ;     
  
  // -----------------------------------------------------------------------
  T_waist_w_0   = locoman::utils::iHomogeneous(T_w_waist_0)  ;
  T_l_ankle_w_0 = locoman::utils::iHomogeneous(T_w_l_ankle_0) ;
  T_l_c1_w_0    = locoman::utils::iHomogeneous(T_w_l_c1_0) ;    
  T_l_c2_w_0    = locoman::utils::iHomogeneous(T_w_l_c2_0) ;  
  T_l_c3_w_0    = locoman::utils::iHomogeneous(T_w_l_c3_0) ;
  T_l_c4_w_0    = locoman::utils::iHomogeneous(T_w_l_c4_0) ;   
    
  T_r_ankle_w_0 = locoman::utils::iHomogeneous(T_w_r_ankle_0) ;
  T_r_c1_w_0    = locoman::utils::iHomogeneous(T_w_r_c1_0) ;    
  T_r_c2_w_0    = locoman::utils::iHomogeneous(T_w_r_c2_0) ;  
  T_r_c3_w_0    = locoman::utils::iHomogeneous(T_w_r_c3_0) ;
  T_r_c4_w_0    = locoman::utils::iHomogeneous(T_w_r_c4_0) ;    

  T_l_wrist_w_0 = locoman::utils::iHomogeneous(T_w_l_wrist_0)  ;
  T_l1_hand_w_0 = locoman::utils::iHomogeneous(T_w_l1_hand_0) ;    
  T_l2_hand_w_0 = locoman::utils::iHomogeneous(T_w_l2_hand_0) ;  
  T_l3_hand_w_0 = locoman::utils::iHomogeneous(T_w_l3_hand_0) ;
  T_l4_hand_w_0 = locoman::utils::iHomogeneous(T_w_l4_hand_0) ;    
    
  T_r_wrist_w_0 = locoman::utils::iHomogeneous(T_w_r_wrist_0) ;
  T_r1_hand_w_0 = locoman::utils::iHomogeneous(T_w_r1_hand_0) ;    
  T_r2_hand_w_0 = locoman::utils::iHomogeneous(T_w_r2_hand_0) ;  
  T_r3_hand_w_0 = locoman::utils::iHomogeneous(T_w_r3_hand_0) ;
  T_r4_hand_w_0 = locoman::utils::iHomogeneous(T_w_r4_hand_0) ;    

  //---------------------------------------------------------------------
  
  T_l_hand_w_0 = locoman::utils::iHomogeneous(T_w_l_hand_0) ;
  T_r_hand_w_0 = locoman::utils::iHomogeneous(T_w_r_hand_0) ;   
  
  T_aw_l_c1_0 = T_aw_w_0 * T_w_l_c1_0 ;  // {AW} is fixed in a loop
  T_aw_l_c2_0 = T_aw_w_0 * T_w_l_c2_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l_c3_0 = T_aw_w_0 * T_w_l_c3_0 ;  // coincident with {AW}
  T_aw_l_c4_0 = T_aw_w_0 * T_w_l_c4_0 ;

  T_aw_r_c1_0 = T_aw_w_0 * T_w_r_c1_0 ;
  T_aw_r_c2_0 = T_aw_w_0 * T_w_r_c2_0 ;
  T_aw_r_c3_0 = T_aw_w_0 * T_w_r_c3_0 ;
  T_aw_r_c4_0 = T_aw_w_0 * T_w_r_c4_0 ; 

  T_aw_l1_hand_0 = T_aw_w_0 * T_w_l1_hand_0 ;  // {AW} is fixed in a loop
  T_aw_l2_hand_0 = T_aw_w_0 * T_w_l2_hand_0 ;  // in every loop the floating base is re-initialized 
  T_aw_l3_hand_0 = T_aw_w_0 * T_w_l3_hand_0 ;  // coincident with {AW}
  T_aw_l4_hand_0 = T_aw_w_0 * T_w_l4_hand_0 ;

  T_aw_r1_hand_0 = T_aw_w_0 * T_w_r1_hand_0 ;
  T_aw_r2_hand_0 = T_aw_w_0 * T_w_r2_hand_0 ;
  T_aw_r3_hand_0 = T_aw_w_0 * T_w_r3_hand_0 ;
  T_aw_r4_hand_0 = T_aw_w_0 * T_w_r4_hand_0 ; 
  

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

  J_l_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
  J_l_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
  J_l_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
  J_l_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

  J_r_c1_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
  J_r_c2_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
  J_r_c3_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
  J_r_c4_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

  J_l_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l_hand_w_0), zero_3 ))* J_l_hand_mix_0 ;
  J_r_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r_hand_w_0), zero_3 ))* J_r_hand_mix_0 ;

  J_l1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l1_hand_w_0), zero_3 ))* J_l1_hand_mix_0 ;
  J_l2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l2_hand_w_0), zero_3 ))* J_l2_hand_mix_0 ;
  J_l3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l3_hand_w_0), zero_3 ))* J_l3_hand_mix_0 ;
  J_l4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_l4_hand_w_0), zero_3 ))* J_l4_hand_mix_0 ;

  J_r1_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r1_hand_w_0), zero_3 ))* J_r1_hand_mix_0 ;
  J_r2_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r2_hand_w_0), zero_3 ))* J_r2_hand_mix_0 ;
  J_r3_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r3_hand_w_0), zero_3 ))* J_r3_hand_mix_0 ;
  J_r4_hand_body_0 = locoman::utils::Adjoint( locoman::utils::Homogeneous(locoman::utils::getRot(T_r4_hand_w_0), zero_3 ))* J_r4_hand_mix_0 ;
  
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------
  // Introducing Spatial Jacobian terms: Fixed base in {AW}
  
  J_aw_l_c1_spa_0 = locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l_c2_spa_0 = locoman::utils::Adjoint(T_aw_l_c2_0)* J_l_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l_c3_spa_0 = locoman::utils::Adjoint(T_aw_l_c3_0)* J_l_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l_c4_spa_0 = locoman::utils::Adjoint(T_aw_l_c4_0)* J_l_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r_c1_spa_0 = locoman::utils::Adjoint(T_aw_r_c1_0)* J_r_c1_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r_c2_spa_0 = locoman::utils::Adjoint(T_aw_r_c2_0)* J_r_c2_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r_c3_spa_0 = locoman::utils::Adjoint(T_aw_r_c3_0)* J_r_c3_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r_c4_spa_0 = locoman::utils::Adjoint(T_aw_r_c4_0)* J_r_c4_body_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;

  J_aw_l1_hand_spa_0 = locoman::utils::Adjoint(T_aw_l1_hand_0)* J_l1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c1_spa_0 ;// locoman::utils::Adjoint(T_aw_l_c1_0)* J_l_c1_body_0
  J_aw_l2_hand_spa_0 = locoman::utils::Adjoint(T_aw_l2_hand_0)* J_l2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c2_spa_0 ;
  J_aw_l3_hand_spa_0 = locoman::utils::Adjoint(T_aw_l3_hand_0)* J_l3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c3_spa_0 ;
  J_aw_l4_hand_spa_0 = locoman::utils::Adjoint(T_aw_l4_hand_0)* J_l4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_l_c4_spa_0 ;

  J_aw_r1_hand_spa_0 = locoman::utils::Adjoint(T_aw_r1_hand_0)* J_r1_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c1_spa_0 ;
  J_aw_r2_hand_spa_0 = locoman::utils::Adjoint(T_aw_r2_hand_0)* J_r2_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c2_spa_0 ;
  J_aw_r3_hand_spa_0 = locoman::utils::Adjoint(T_aw_r3_hand_0)* J_r3_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c3_spa_0 ;
  J_aw_r4_hand_spa_0 = locoman::utils::Adjoint(T_aw_r4_hand_0)* J_r4_hand_mix_0 ; // locoman::utils::Adjoint( T_aw_waist_0)* J_waist_r_c4_spa_0 ;
  
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

  // -------------------------------------------------------------------------------------------------------------
  // Defining derivative Terms

  // Computing Derivative Terms
  Q_aw_l_c1 = locoman::utils::Q_ci(J_aw_l_c1_spa_0, T_aw_l_c1_0, fc_l_c1_filt ) ;
  Q_aw_l_c2 = locoman::utils::Q_ci(J_aw_l_c2_spa_0, T_aw_l_c2_0, fc_l_c2_filt ) ; // (size_q+ 6, size_q + 6) ;
  Q_aw_l_c3 = locoman::utils::Q_ci(J_aw_l_c3_spa_0, T_aw_l_c3_0, fc_l_c3_filt ) ; //(size_q+ 6, size_q + 6) ; 
  Q_aw_l_c4 = locoman::utils::Q_ci(J_aw_l_c4_spa_0, T_aw_l_c4_0, fc_l_c4_filt ) ; //(size_q+ 6, size_q + 6) ;
  //

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
  

  //--------------------------------------------------------------------------------------------------------------------------------------------  
  // Input-Guided State Machine => Building the proper FLLM and control in each state
    
  double toc = locoman::utils::Toc(tic) ;
  std::cout << "tic-toc = " << toc << " seconds" << std::endl ;
  //----------------------------------------------
  char file_name[] = "tic_toc.txt";   // writing
  std::ofstream tictoc_cl ( file_name, std::ios::app );
  if( tictoc_cl.is_open() )
  tictoc_cl <<  toc << std::endl;  
  
  //------------------------------------------------
  
  std::string command  ; //
  if(bool ifCommand = command_interface.getCommand(command) ){
       //  if(command!=last_command){
  //std::cout << " ifCommand  =  "<< std::endl << ifCommand << std::endl  ; 
  CoM_w_cmd = model.iDyn3_model.getCOM()  ;  //   ...cmd is for "@ command time"
  //-------------------------------------------------------------------------------
  T_waist_l_hand_cmd  = T_waist_w_0 * T_w_l_hand_0  ;    
  T_waist_r_hand_cmd  = T_waist_w_0 * T_w_r_hand_0  ;    
  T_waist_l1_foot_cmd = T_waist_w_0 * T_w_l_c1_0 ; 
  T_waist_r1_foot_cmd = T_waist_w_0 * T_w_r_c1_0 ; 
  CoM_waist_cmd       = locoman::utils::getRot( T_waist_w_0 ) * CoM_w_cmd ;
  R_waist_aw_cmd      = locoman::utils::getRot(T_waist_w_0 *T_w_aw_0) ; 
  //------------------------------------------------------------------------------_
  // //std::cout << " CoM_w_cmd  =  "<< std::endl << CoM_w_cmd.toString() << std::endl  ; 
  
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
  //  <<<<<  Right Foot --- UP --- Configuration  >>>>>
  T_l1_r1_up = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
  double z_r1_up = 0.07 ;
  yarp::sig::Matrix T_aw_r1_up = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_l_c1_0),zero_3)* T_l1_r1_up ;
  T_aw_r1_up[2][3] += z_r1_up;
  T_l1_r1_up = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_l_c1_0)),zero_3)*T_aw_r1_up  ;
  //
  //-----------------------------------------------------------------------------------
  //  <<<<<  Right Foot --- FORWARD --- Configuration  >>>>>
  //
  T_l1_r1_fw = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
//   yarp::sig::Matrix Rot_l1_r1 = locoman::utils::getRot(T_l1_r1_fw) ;
//   yarp::sig::Vector x_l1_r1(3,0.0) ;
//   x_l1_r1[0]=  T_l1_r1_fw[0][0] ;
//   x_l1_r1[1]=  T_l1_r1_fw[1][0] ;
//   x_l1_r1[2]=  T_l1_r1_fw[2][0] ;
  //
  double x_r1_fw = 0.15 ;
  T_l1_r1_fw[0][3] += x_r1_fw*T_l1_r1_fw[0][0]   ; 
  T_l1_r1_fw[1][3] += x_r1_fw*T_l1_r1_fw[1][0]  ; 
  T_l1_r1_fw[2][3] += x_r1_fw*T_l1_r1_fw[2][0]  ; 
  //
  //-----------------------------------------------------------------------------------
  //  <<<<<  Right Foot --- DOWN --- Configuration  >>>>>
  T_l1_r1_dw = locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0 ;
  double z_r1_dw = -0.15 ; // Going down the foot should stop the motion when in touch
  yarp::sig::Matrix T_aw_r1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_l_c1_0),zero_3)* T_l1_r1_dw ;
  T_aw_r1_dw[2][3] += z_r1_dw ;
  T_l1_r1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_l_c1_0)),zero_3)*T_aw_r1_dw  ;
 // T_l1_r1_dw.setSubmatrix( Rot_des_r,0,0 );  
  
  
  
  
  
  
  
  
  
  //---------------------------------------------------------------------
  //----------------------------------------------------------------------------------- 
  //    T_r1_l1_up  
  T_r1_l1_up = locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0;
  double z_l1_up = 0.04 ;
  yarp::sig::Matrix T_aw_l1_up = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_r_c1_0),zero_3)* T_r1_l1_up ;
  T_aw_l1_up[2][3] += z_l1_up;
  T_r1_l1_up = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_r_c1_0)),zero_3)*T_aw_l1_up  ;
  T_r1_l1_up.setSubmatrix( Rot_des_l,0,0 );
  ////std::cout << " T_r1_l1_up  =  "<< std::endl << T_r1_l1_up.toString() << std::endl  ; 
  ////std::cout << " T_r1_l1_up  =  "<< std::endl << T_r1_l1_up.toString() << std::endl  ;   
  
  
  
//   //--------------------------
//   yarp::sig::Vector x_l1(3,0.0) ;
//   x_l1[0]=  1.0 ;
//   x_l1[1]=  0.3 ;
//   x_l1[2]=  0.0 ;
//   x_l1 =  x_l1/(norm(x_l1)) ;
//   
//   yarp::sig::Vector x_fw(3,0.0) ;
//   
//   x_fw[0] = alpha_left*x_l1[0] + alpha_right*x_l1_r1[0] ;
//   x_fw[1] = alpha_left*x_l1[1] + alpha_right*x_l1_r1[1];
//   x_fw[2] = alpha_left*x_l1[2] + alpha_right*x_l1_r1[2];
//   x_fw = x_fw/(norm(x_fw)) ;
//   
//   
//   T_l1_r1_fw[0][3] += x_r1_fw*x_fw[0] ;
//   T_l1_r1_fw[1][3] += x_r1_fw*x_fw[1] ;
//   T_l1_r1_fw[2][3] += x_r1_fw*x_fw[2] ;
//   //T_l1_r1_fw.setSubmatrix( Eye_3,0,0 );
//   //std::cout << " T_l1_r1_fw  =  "<< std::endl << T_l1_r1_fw.toString() << std::endl  ; 

  
  //----------------------------------------------------------------------------
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
  double alpha_left = 1.0 ;
  double alpha_right = 1.0 - alpha_left ;
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
  //std::cout << " T_r1_l1_fw  =  "<< std::endl << T_r1_l1_fw.toString() << std::endl  ; 
  

  //std::cout << " T_l1_r1_dw  =  "<< std::endl << T_l1_r1_dw.toString() << std::endl  ; 
//  
//    T_r1_l1_dw  
  T_r1_l1_dw = locoman::utils::iHomogeneous(T_aw_r_c1_0 )* T_aw_l_c1_0 ;
  double z_l1_dw = -10.0*z_l1_up ;
  yarp::sig::Matrix T_aw_l1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(T_aw_r_c1_0),zero_3)* T_r1_l1_dw ;
  T_aw_l1_dw[2][3] += z_l1_dw ;
  T_r1_l1_dw = locoman::utils::Homogeneous(locoman::utils::getRot(locoman::utils::iHomogeneous(T_aw_r_c1_0)),zero_3)*T_aw_l1_dw  ;
  T_r1_l1_dw.setSubmatrix( Rot_des_l,0,0 );  
  //std::cout << " T_r1_l1_dw  =  "<< std::endl << T_r1_l1_dw.toString() << std::endl  ; 
//  
//   
 // }
  }
  
  if (command != "")    {
   last_command = command ; 
  }
       //std::cout << " last_command  =  "<< std::endl << last_command << std::endl  ; 

  
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
       //std::cout << " CoM_w_0  =  "<< std::endl << CoM_w_0.toString() << std::endl  ; 

     }
    else if (last_command =="to_cr" || last_command =="center")
     { 
        locoman::utils::FC_DES_center(FC_DES, mg) ;  // half weight on the right, half on the left foot
     }
  
 // desired contact force definition  
  FC_DES_LEFT_sensor  = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
  FC_DES_RIGHT_sensor = map_l_fcToSens * FC_DES.subVector(12,23)  ;
 
  d_fc_des_to_world  = FC_DES - FC_FILTERED ; //  
 
  //-----------------------------------------------------------------------------------------------------

  yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c, S_c, Q_aw_s_cont, U_aw_s_cont, Kc ) ;
  yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
   
  yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
  yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 

  
  double err = norm( d_fc_des_to_world )  ;  // d_fc_des_to_world

  double regu_filter = 1E8 ; 
  
  
//   if( last_command == "center"){
//       if (err > 200.0){regu_filter = 1E8 ;}
//       else if (err > 100.0){regu_filter = 1E10 ;}
//       else
//       regu_filter = 1E13 ;
// }
//   std::cout << " regu_filter  =  "<< std::endl << regu_filter << std::endl  ;  

  
  yarp::sig::Vector d_q_dsp_6 = -1.0* locoman::utils::Pinv_Regularized( Rf_temp_2_filt, regu_filter)* d_fc_des_to_world ;
  std::cout << " d_q_dsp_6  =  "<< std::endl << d_q_dsp_6.toString() << std::endl  ;  

  //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------

  // Projecting in null space of the r-hand
  //1) computind d_u
  
  
  yarp::sig::Matrix Ru_temp_2 = cFLMM.submatrix(size_fc, size_fc+6-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
//  yarp::sig::Matrix Ru_temp_2_filt = locoman::utils::filter_SVD( Ru_temp_2,  1E-10); 
  
  yarp::sig::Matrix J_u_r_hand = J_r1_hand_body_0.submatrix(0,5,0,5) ;
  yarp::sig::Matrix J_q_r_hand = J_r1_hand_body_0.submatrix(0,5,6,J_r1_hand_body_0.cols()-1) ;
  
  yarp::sig::Matrix J_u_r_foot = J_r_c1_body_0.submatrix(0,5,0,5)  ;
  yarp::sig::Matrix J_q_r_foot = J_r_c1_body_0.submatrix(0,5,6,J_r_c1_body_0.cols()-1)  ;
  
  yarp::sig::Matrix J_u_l_foot = J_l_c1_body_0.submatrix(0,5,0,5)  ;
  yarp::sig::Matrix J_q_l_foot = J_l_c1_body_0.submatrix(0,5,6,J_l_c1_body_0.cols()-1)  ;
  
  yarp::sig::Matrix J_1(12,6) ;
  yarp::sig::Matrix J_2(12,d_q_dsp_6.length() ) ;
  
  J_1.setSubmatrix( J_u_r_hand, 0,  0 ) ;
  J_1.setSubmatrix( J_u_r_foot, 6,  0 ) ;
  //J_1.setSubmatrix( J_u_l_foot, 12, 0 ) ;
 
  J_2.setSubmatrix( J_q_r_hand, 0,  0 ) ;
  J_2.setSubmatrix( J_q_r_foot, 6,  0 ) ;
  //J_2.setSubmatrix( J_q_l_foot, 12, 0 ) ;
  
  yarp::sig::Matrix J_aux = -1.0*J_1*Ru_temp_2 + 1.0*J_2;
  J_aux = locoman::utils::filter_SVD( J_aux,  1E-10); 


  yarp::sig::Vector T_aux(12, 0.0) ;
  if(loop_counter%800 < 400 ){T_aux[0] = 0.001 ;
         std::cout << " qui 1 = loop_counter%200 < 100   =  "  << loop_counter%200    << std::endl  ;  }
  else{T_aux[0] = -0.001 ;
               std::cout << " qui 2 = loop_counter%200 < 100   =  "  << loop_counter%200    << std::endl  ;  
}

  yarp::sig::Vector d_q_r_hand = pinv(J_aux)*T_aux ;
  
  //  
  yarp::sig::Vector d_q_projected = nullspaceProjection(J_aux,1E-3)* d_q_dsp_6;

  
  
  //std::cout << " Ru_temp_2.rows  =  "<< std::endl << Ru_temp_2.rows() << std::endl  ;  
  //std::cout << " Ru_temp_2.cols  =  "<< std::endl << Ru_temp_2.cols() << std::endl  ;  
  
 // yarp::sig::Vector d_u_6 = -1.0*  Ru_temp_2_filt* d_q_dsp_6  ;
  
  //std::cout << " d_u_6.length()  =  "<< std::endl << d_u_6.length() << std::endl  ;  

 /* 
  yarp::sig::Vector d_q_star(d_u_6.length()+d_q_dsp_6.length());
  d_q_star.setSubvector(0,d_u_6) ;
  d_q_star.setSubvector(6,d_q_dsp_6) ;
  
  yarp::sig::Vector d_q_star_projected = nullspaceProjection(J_r1_hand_body_0,1E-4)* d_q_star;
  
  yarp::sig::Vector d_q_projected = d_q_star_projected.subVector(6,d_q_star_projected.length()-1) ;
   
     */
 
    std::cout << " d_q_r_hand  =  "<< std::endl << d_q_r_hand.toString() << std::endl  ;  

   std::cout << " d_q_dsp_6.toString()  =  "<< std::endl << d_q_dsp_6.toString() << std::endl  ;  

 std::cout << " d_q_projected.toString()  =  "<< std::endl << d_q_projected.toString() << std::endl  ;  

   
 //
 
 //T_l_c1_w_0 *T_w_waist_0 *T_waist_l1_foot_cmd ;
 //T_r_c1_w_0 *T_w_waist_0 *T_waist_r1_foot_cmd ;
 
//    yarp::sig::Matrix T_des_r_hand = T_r_hand_w_0 * T_w_waist_0 *T_waist_r_hand_cmd ;
//      yarp::sig::Vector zer_3(3,0.0) ;
//      
// 
//     // ------------------------------------------------------------------
//   yarp::sig::Vector d_q_r_hand = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_des_r_hand,
//                                                                      Eye_4 , //T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
//                                                                      Eye_4,                              //T_l1_foot_des_local, T_r1_foot_des_local , 
//                                                                 zer_3 , 
//                                                                 J_l_hand_body_0,
//                                                                 J_r_hand_body_0, 
//                                                                 J_l_c1_body_0, 
//                                                                 J_r_c1_body_0, 
//                                                                 J_com_waist) ;
//  
//  
//    if(norm(d_q_r_hand)>0.0018){d_q_r_hand =  0.0018 *d_q_r_hand/ norm(d_q_r_hand) ; //d_q_dsp_7 ; //0.012 *d_q_move/ norm(d_q_move) ;
//   }  // 0.0015
// 
// //   if(norm(d_q_r_hand)<0.0002){
// //         std::cout << " norm(d_q_r_hand) = " << norm(d_q_r_hand) << std::endl  ;  
// //       d_q_r_hand =  0.0002 *d_q_r_hand/ norm(d_q_r_hand) ;
// //   }   
//  
//     std::cout << " d_q_r_hand.length  =  "<< std::endl << d_q_r_hand.length() << std::endl  ;  
// 
//     std::cout << " d_q_projected.length  =  "<< std::endl << d_q_projected.length() << std::endl  ;  
//  
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
 
  
  yarp::sig::Vector d_q_move = 2.0*d_q_r_hand+ d_q_projected; // d_q_dsp_6 ; // d_q_dsp_5_m   ; // d_q_dsp_6 ; //
  
  if(norm(d_q_move)>0.0018){d_q_move =  0.0018 *d_q_move/ norm(d_q_move) ; //d_q_dsp_7 ; //0.012 *d_q_move/ norm(d_q_move) ;
  }  // 0.0015
  if(norm(d_q_move)<0.0002){d_q_move =  0.0002 *d_q_move/ norm(d_q_move) ;
  }   
   std::cout << " d_q_move.toString()  =  "<< std::endl << d_q_move.toString() << std::endl  ;  

  //   
  double err_min = 70.0 ; //10.0 ;
  double err_max = 250.0 ;  //40.0 ; 
    
  char file_name[] = "err.m";   // writing
  std::ofstream err_cl ( file_name, std::ios::app );
  if( err_cl.is_open() )
  err_cl <<  err << std::endl;  
      
//   double peso = -1.0*FC_FILTERED_LEFT_sensor[2] -1.0*FC_FILTERED_RIGHT_sensor[2]  ;
//   char file_name_2[] = "peso.m";   // writing
//   std::ofstream peso_cl ( file_name_2, std::ios::app );
//   if( peso_cl.is_open() )
//   peso_cl <<  peso << std::endl;  
  //  std::cout << " peso  =  "<< std::endl << peso << std::endl  ;  

  
  //---------------------------------------------------- -------------------------------------------

  double alpha = locoman::utils::alpha_filter(err, err_min, err_max) ;

//   std::cout << " FC_DES  =  "<< std::endl << FC_DES.toString() << std::endl  ; 
//   std::cout << " FC_FILTERED =  "<< std::endl << FC_FILTERED.toString() << std::endl  ; 
  std::cout << " err  =  "<< std::endl << err << std::endl  ;  
//  std::cout << " alpha  =  "<< std::endl << alpha << std::endl  ;  
  
//   q_ref_ToMove = q_motor_side + q_offset +  (0.0/1.0)* alpha*d_q_move ;  //d_q_temp_var ; // alpha*d_q_aw_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        
//   q_ref_ToMove = q_current    + q_offset +  (0.0/1.0)* alpha*d_q_move  ;  // on the real robot
//   q_ref_ToMove = q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ;    // q in open loop
//   q_current_open_loop =  q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ; 

  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* alpha * d_q_move  ; 
  
 //------------------------------------------------------------------------------------------------------  

  //std::cout << "Double Stance Phase "<< std::endl ; 
 // robot.move29( q_ref_ToMove);                 //  q_motor_side);//

  } // closing of the Double Stance Phase
//----------------------------------------------------------------------------------
else if (last_command =="feet_r_hand" )//|| last_command =="resume" ||            
  {  // Feet and R-Hand Stance Phase  

  // locoman::utils::FC_DES_right(FC_DES, mg) ; //  
  locoman::utils::FC_DES_left(FC_DES, mg) ; // all the weight on the left foot    
   // desired contact force definition  
  FC_DES_LEFT_sensor  = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
  FC_DES_RIGHT_sensor = map_r_fcToSens * FC_DES.subVector(12,23)  ;
  //    
//   FC_DES_RIGHT_HAND_sensor.zero(); ;
//   FC_DES_RIGHT_HAND_sensor[1] = mg ;
  FC_HANDS_DES.zero();
  FC_HANDS_DES[13] =  mg/4.0 ;// .setSubvector(0 , locoman::utils::Pinv_trunc_SVD( map_r_hand_fcToSens, 1E-10) * FC_DES_RIGHT_HAND_sensor) ;
  FC_HANDS_DES[16] =  mg/4.0 ;
  FC_HANDS_DES[19] =  mg/4.0 ;
  FC_HANDS_DES[22] =  mg/4.0 ;  // Positive force along y-axis on the contact points

  // Partitioning the forces between r-hand and left foot
  
 // desired contact force definition  
  double part_r_hand= 1.0/10.0 ;
  FC_HANDS_DES =  part_r_hand * FC_HANDS_DES ;  
  FC_DES       = (1-part_r_hand)* FC_DES ;
  // desired contact force variation
  d_fc_des_to_world  = (FC_DES - FC_FILTERED) ; //  d_fc to the feet
  
//   yarp::sig::Vector d_fc_r_hand  = FC_HANDS_DES.subVector(12,23) -  FC_HANDS_FILTERED.subVector(12,23) ;
//   yarp::sig::Vector d_fc_des_f_r_hand(size_fc+size_fc/2,0.0) ;
  d_fc_des_f_r_hand.setSubvector(0,d_fc_des_to_world) ;
  d_fc_des_f_r_hand.setSubvector(size_fc ,   d_fc_r_hand) ;
  //  //std::cout << " size_fc = " << size_fc <<  std::endl ;

  //std::cout << " d_fc_des_to_world = " << d_fc_des_to_world.toString() <<  std::endl ;
  
  //FC_HANDS_FILTERED
  //
  //-----------------------------------------------------------------------------------------------------------------     
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
//    std::cout << " FC_DES_feet = " << std::endl << FC_DES.toString() << std::endl; 
//    std::cout << " FC_FILTERED_feet = " << std::endl << FC_FILTERED.toString() << std::endl; 
//    std::cout << " d_fc_des_to_world = " <<  std::endl << d_fc_des_to_world.toString() << std::endl;  
//    std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl;    
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
//    std::cout << " FC_R_HAND_DES = " <<  std::endl << FC_HANDS_DES.subVector(12,23).toString() << std::endl;    
//    std::cout << " FC_R_HAND_FILTERED = " <<  std::endl << FC_HANDS_FILTERED.subVector(12,23) .toString() << std::endl;    
//    std::cout << " d_fc_r_hand = " <<  std::endl << d_fc_r_hand.toString() << std::endl;  
//    std::cout << " norm( d_fc_r_hand ) = " <<  std::endl << norm( d_fc_r_hand ) << std::endl;    
//    std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   //std::cout << " d_fc_des_f_r_hand = " <<  std::endl << d_fc_des_f_r_hand.toString() << std::endl;  
   //std::cout << " norm( d_fc_des_f_r_hand ) = " <<  std::endl << norm( d_fc_des_f_r_hand ) << std::endl;       
   //std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
  //-----------------------------------------------------------------------------------------------------
   
  yarp::sig::Matrix FLMM  = locoman::utils::FLMM_redu(J_c_f_rh, S_c_f_rh, Q_aw_s_c_f_rh, U_aw_s_c_f_rh, Kc_f_rh ) ;
  yarp::sig::Matrix cFLMM = locoman::utils::Pinv_trunc_SVD(FLMM.submatrix(0, FLMM.rows()-1 , 0, FLMM.rows()-1), 1E-10 ) * FLMM;
     
  yarp::sig::Matrix Rf_temp_2 = cFLMM.submatrix(0, size_fc+size_fc/2-1, cFLMM.cols()-size_q, cFLMM.cols()-1) ;  
  yarp::sig::Matrix Rf_temp_2_filt = locoman::utils::filter_SVD( Rf_temp_2,  1E-10); 

  yarp::sig::Vector weights(size_fc+size_fc/2,1.0) ;
  double weight_hand = 1.0 ;
  weights.setSubvector(size_fc, weight_hand*weights.subVector(size_fc, size_fc+size_fc/2-1 )   ) ;
  //std::cout << " weights = " <<  std::endl << weights.toString()  << std::endl;  

  yarp::sig::Matrix W_f_r_hand(size_fc+size_fc/2, size_fc+size_fc/2);
  W_f_r_hand.zero();
  W_f_r_hand.diagonal(weights) ;
  W_f_r_hand = W_f_r_hand/weight_hand ;  // *W_f_r_hand 

  yarp::sig::Vector d_q_dsp_6  = -1.0*  locoman::utils::Pinv_Regularized( Rf_temp_2_filt, 1E2)* d_fc_des_f_r_hand ; 
   
 
  //std::cout << " norm(d_q_dsp_6) = " <<  std::endl << norm(d_q_dsp_6)  << std::endl;  

  yarp::sig::Vector d_q_move = d_q_dsp_6 ; // d_q_dsp_5_m   ; // d_q_dsp_6 ; //
  
  if(norm(d_q_move)>0.0016){d_q_move = 0.0016 *d_q_move/ norm(d_q_move) ; //d_q_dsp_7 ; //0.012 *d_q_move/ norm(d_q_move) ;
  }
  if(norm(d_q_move)<0.0002){d_q_move = 0.0002 *d_q_move/ norm(d_q_move)  ;
  }
  //std::cout << " d_q_move = " <<  std::endl << d_q_move.toString()  << std::endl;   
   
  //   
  double err = norm( d_fc_des_f_r_hand )  ;  // d_fc_des_to_world
  double err_min = 100.0 ; //10.0 ;
  double err_max = 600.0 ;  //40.0 ; 
    
  char file_name[] = "err.m";   // writit
  std::ofstream err_cl ( file_name, std::ios::app );
  if( err_cl.is_open() )
  err_cl <<  err << std::endl;  
    
  
  double err_feet =  norm( d_fc_des_to_world ) ;
  char file_name_feet[] = "err_feet.m";   // writit
  std::ofstream err_cl_feet ( file_name_feet, std::ios::app );
  if( err_cl_feet.is_open() )
  err_cl_feet <<  err_feet << std::endl;  
  
  double err_r_hand =  norm( d_fc_r_hand )  ;
  char file_name_r_hand[] = "err_r_hand.m";   // writit
  std::ofstream err_cl_r_hand ( file_name_r_hand, std::ios::app );
  if( err_cl_r_hand.is_open() )
  err_cl_r_hand <<  err_r_hand << std::endl;  
  
    /*char file_name1[] = "err1.m";
    char temp[] = "temp" ;
    std::ofstream temp ( file_name1, std::ios::app );
    if( &temp.is_open() )
    temp <<  err << std::endl;  */
    
    //---------------------------------------------------- -------------------------------------------

  double alpha = locoman::utils::alpha_filter(err, err_min, err_max) ;
  std::cout << " err = "  << err << std::endl; 
  std::cout << " alpha = "  << alpha << std::endl; 
  
  //q_ref_ToMove = q_motor_side + q_offset +  (0.0/1.0)* alpha*d_q_move ;  //d_q_temp_var ; // alpha*d_q_aw_2_6 ;  // +  (1.0/5.0)*d_q_0_project ;        
 // q_ref_ToMove = q_current    + q_offset +  (0.0/1.0)* alpha*d_q_move  ;  // on the real robot
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* alpha*d_q_move  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)* alpha*d_q_move  ; 
 
  //std::cout << " q_ref_ToMove = "  << q_ref_ToMove.toString() << std::endl; 

  //std::cout << "Feet and R-hand Stance Phase "<< std::endl ; 
  robot.move29( q_ref_ToMove);                 //  q_motor_side);//
  } // End of Feet + R-Hand Phase

  
  
//------------------------------------------------------------------------------------
else if (last_command == "sw_rg_up" || last_command == "sw_rg_fw" || last_command == "sw_rg_dw" || 
         last_command == "sw_lf_up" || last_command == "sw_lf_fw" || last_command == "sw_lf_dw"  )
/*   last_command == "com_up"   || last_command == "com_dw"   ||
     last_command =="r_hand_z_world_fw" || last_command =="r_hand_z_world_bk" || 
     last_command =="r_hand_x_world_fw" || last_command =="r_hand_x_world_bk" ||
     last_command =="r_hand_z_world_rot_ccw" ||
     last_command =="r_hand_z_world_rot_cw"  ||   
     last_command =="r_hand_x_world_rot_ccw" ||
     last_command =="r_hand_x_world_rot_cw" */  
   { // Single Stance Phase  
  
  if (last_command =="sw_rg_up")
     {
  // Moving {r_c1} frame in positive direction along Z_aw axis
  // We suppose to maintain fixed both the {l_c1} frame and the CoM 
  // (in order to avoid falling down)         
        
 //  std::cout << "qui"<< std::endl ;
  T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  // translation error
  yarp::sig::Matrix D_T_r_up = T_l1_r1_up - T_l_c1_r_c1_loop ; // toward the desired displacement
  
//   std::cout << "T_l1_r1_up "<<  T_l1_r1_up.toString() <<std::endl ;
// 
//   std::cout << "T_l_c1_r_c1_loop "<<  T_l_c1_r_c1_loop.toString() <<std::endl ;
// 
//   std::cout << "D_T_r_up "<<  D_T_r_up.toString() <<std::endl ;
 
  yarp::sig::Vector d_l1_r1_up(3, 0.0);
  d_l1_r1_up[0] = D_T_r_up[0][3] ;
  d_l1_r1_up[1] = D_T_r_up[1][3];
  d_l1_r1_up[2] = D_T_r_up[2][3]; // toward the desired displacement
  if(norm( d_l1_r1_up ) > 0.001  ){ d_l1_r1_up = 0.001*d_l1_r1_up/(norm( d_l1_r1_up)) ;  }
  if(norm( d_l1_r1_up ) < 0.0008 ){ d_l1_r1_up = 0.0*d_l1_r1_up  ;  }  // desired displacement
  // Orientation error
  yarp::sig::Vector e_o = locoman::utils::Orient_Error( locoman::utils::getRot(T_l1_r1_up ) , 
                                                        locoman::utils::getRot(T_l_c1_r_c1_loop ) ) ; 
  if(norm( e_o )>0.01 ){ e_o = 0.01*e_o/(norm( e_o)) ; }
  if(norm( e_o )<0.001){ e_o = 0.0*e_o  ; }     // desired orientation variation
  
  d_EE_r_des.setSubvector(0, (1.0/1.0)*d_l1_r1_up ) ;  
  d_EE_r_des.setSubvector(3, (1.0/1.0)*e_o ) ;        // Delta_Configuration Vector for the frame
  std::cout << "  d_EE_r_des  = " <<  std::endl << d_EE_r_des.toString()   << std::endl;   

  //-----------------------------------------------------------------------------------------------------------------
  // Here I consider an absolute frame with origin in {l1} and orientation parallel to {AW}
  // {AW_l1} => fixed and with z vertical
  
  model.iDyn3_model.getCOMJacobian(J_com_w) ;
  J_com_w_redu = J_com_w.submatrix(0,2 , 0 , J_com_w.cols()-1 ) ;  
  J_com_aw = locoman::utils::getRot(T_aw_w_0) *J_com_w_redu;
 
  //J_r_c1_aw =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_l_c1_r_c1_loop), zero_3) )* J_r_c1_body_0 ;
 
  yarp::sig::Matrix J_l_c1_r_c1 =  locoman::utils::Adjoint(locoman::utils::Homogeneous(locoman::utils::getRot(  T_l_c1_r_c1_loop), zero_3) )* J_r_c1_body_0 ;
  
  yarp::sig::Matrix J_sw_rg_up( 15 , ( robot.getNumberOfKinematicJoints() + 6 ));
  J_sw_rg_up.setSubmatrix(J_com_aw,0,0) ;
  J_sw_rg_up.setSubmatrix(J_l_c1_body_0,3,0) ;
  J_sw_rg_up.setSubmatrix(J_l_c1_r_c1,9,0) ;
   
  yarp::sig::Vector Task_sw_rg_up(15,0.0) ;   
  Task_sw_rg_up.setSubvector( 9 , (1.0/1.0)* d_EE_r_des  ) ;
 // //std::cout << " Task_sw_rg_up = " <<  std::endl << Task_sw_rg_up.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_rg_up = locoman::utils::Pinv_trunc_SVD(J_sw_rg_up)* Task_sw_rg_up ;     
  yarp::sig::Vector d_q_sw_rg_up = d_u_q_sw_rg_up.subVector( 6, d_u_q_sw_rg_up.length()-1) ;
  
  if(norm(d_q_sw_rg_up)>0.002 ){d_q_sw_rg_up = 0.002*d_q_sw_rg_up/norm(d_q_sw_rg_up) ; }  
  
  q_ref_ToMove = q_senseRefFeedback  + (1.0/1.0)* d_q_sw_rg_up ;
   
  robot.move29( q_ref_ToMove );           
  }
     
  else if (last_command =="sw_rg_fw")
     {
  //std::cout << "swinging the right foot forward"<< std::endl ;
      
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
    
  yarp::sig::Vector d_u_q_sw_rg_fw = locoman::utils::Pinv_trunc_SVD(J_sw_rg_up)* Task_sw_rg_fw ;     
  yarp::sig::Vector d_q_sw_rg_fw = d_u_q_sw_rg_fw.subVector( 6, d_u_q_sw_rg_fw.length()-1) ;
  
  if(norm(d_q_sw_rg_fw)>0.002 ){d_q_sw_rg_fw = 0.002*d_q_sw_rg_fw/norm(d_q_sw_rg_fw) ; }  

  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_sw_rg_fw  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_rg_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }
     
  else if (last_command =="sw_rg_dw")
     {
  //std::cout << "swinging the right foot down"<< std::endl ;
  
  T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  
  yarp::sig::Matrix D_T_r_dw = T_l1_r1_dw - T_l_c1_r_c1_loop ; // toward the desired displacement
  yarp::sig::Vector d_l1_r1_dw(3, 0.0);
  d_l1_r1_dw[0] = D_T_r_dw[0][3] ;
  d_l1_r1_dw[1] = D_T_r_dw[1][3] ;
  d_l1_r1_dw[2] = D_T_r_dw[2][3] ; // toward the desired displacement
  if(norm( d_l1_r1_dw)>0.001){ d_l1_r1_dw = 0.001*d_l1_r1_dw/(norm( d_l1_r1_dw)) ;  }
  if(norm( d_l1_r1_dw)<0.0008){ d_l1_r1_dw = 0.0*d_l1_r1_dw  ;  }  // desired displacement
  
  if(FC_FILTERED_RIGHT_sensor[2]<-5.0){
       std::cout << " ****** !!!!!!!!!!!! TOUCHED !!!!!!!!!!!! ******" << std::endl ;
      d_l1_r1_dw = 0.0*d_l1_r1_dw  ;  }
  
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
 // //std::cout << " Task_sw_rg_up = " <<  std::endl << Task_sw_rg_up.toString()  << std::endl;   
    
  yarp::sig::Vector d_u_q_sw_rg_dw = locoman::utils::Pinv_trunc_SVD(J_sw_rg_dw)* Task_sw_rg_dw ;     
  yarp::sig::Vector d_q_sw_rg_dw = d_u_q_sw_rg_dw.subVector( 6, d_u_q_sw_rg_dw.length()-1) ;
  
  if(norm(d_q_sw_rg_dw)>0.002 ){d_q_sw_rg_dw = 0.002*d_q_sw_rg_dw/norm(d_q_sw_rg_dw) ; }  

  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_sw_rg_dw  ;    // q in open loop
 //  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_rg_dw  ; 
  robot.move29( q_ref_ToMove );   
  
     }
     
     
  else if (last_command =="sw_lf_up")
     {
  //std::cout << "swinging the left foot up"<< std::endl ; 

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
  //std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   
  
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
 // //std::cout << " Task_sw_lf_up = " <<  std::endl << Task_sw_lf_up.toString()  << std::endl;   
  yarp::sig::Vector d_u_q_sw_lf_up = locoman::utils::Pinv_trunc_SVD(J_sw_lf_up)* Task_sw_lf_up ;     
  yarp::sig::Vector d_q_sw_lf_up = d_u_q_sw_lf_up.subVector( 6, d_u_q_sw_lf_up.length()-1) ;
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_lf_up  ;  

  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_sw_lf_up  ;    // q in open loop
  // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_lf_up  ; 
  robot.move29( q_ref_ToMove );           
     }
     
else if (last_command =="sw_lf_fw")
     {
  //std::cout << "swinging the left foot forward"<< std::endl ;
      
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
  //std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   
  
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
 // //std::cout << " Task_sw_lf_fw = " <<  std::endl << Task_sw_lf_fw.toString()  << std::endl;   
  yarp::sig::Vector d_u_q_sw_lf_fw = locoman::utils::Pinv_trunc_SVD(J_sw_lf_up)* Task_sw_lf_fw ;     
  yarp::sig::Vector d_q_sw_lf_fw = d_u_q_sw_lf_fw.subVector( 6, d_u_q_sw_lf_fw.length()-1) ;
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_sw_lf_fw  ;  

  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_sw_lf_fw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_sw_lf_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     

   else if (last_command =="sw_lf_dw")
     {
  //std::cout << "swinging the left foot down"<< std::endl ;
  
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
  //std::cout << "  d_EE_l_des  = " <<  std::endl << d_EE_l_des.toString()   << std::endl;   

  
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
 // //std::cout << " Task_sw_lf_dw = " <<  std::endl << Task_sw_lf_dw.toString()  << std::endl;   
    
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
  //std::cout << "Moving CoM up"<< std::endl ;
  
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
  //std::cout << " Task_com_up = " <<  std::endl << Task_com_up.toString()  << std::endl;   
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
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_com_up  ;// 
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_com_up  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_com_up  ; 
  robot.move29( q_ref_ToMove );   
  }
  else if (last_command == "com_dw")
  { 
  //std::cout << "Moving CoM dw"<< std::endl ; //
  
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
  //std::cout << " Task_com_up = " <<  std::endl << Task_com_up.toString()  << std::endl;   
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
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_com_dw  ;// 
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_com_dw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_com_dw  ; 
  robot.move29( q_ref_ToMove );   
  } 
   else if (last_command =="sw_rg_up_2")
     {
  //std::cout << "R Foot, Z Local, Moving Up"<< std::endl ;
   T_l_c1_r_c1_loop = (locoman::utils::iHomogeneous(T_aw_l_c1_0 )* T_aw_r_c1_0) ; // where r1 is with respect to l1
  // translation error
  yarp::sig::Matrix D_T_r_up = T_l1_r1_up - T_l_c1_r_c1_loop ;
    yarp::sig::Vector d_l1_r1_up(3, 0.0);
  d_l1_r1_up[0] = D_T_r_up[0][3] ;
  d_l1_r1_up[1] = D_T_r_up[1][3];
  d_l1_r1_up[2] = D_T_r_up[2][3]; // toward the desired displacement
  double d ;
  if(norm( d_l1_r1_up ) > 0.0008  ){ d = 0.0008 ;  }
  if(norm( d_l1_r1_up ) < 0.0002 ){ d = 0.0  ;  }  // desired displacement
         
  yarp::sig::Matrix T_temp_2 = Eye_4 ;
  T_temp_2[2][3] = 0.0003 ;
   
  yarp::sig::Vector zer_3(3,0.0) ;

   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_z_local_up = locoman::utils::WB_Cartesian_Tasks( Eye_4, Eye_4 , //T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  T_temp_2,                            //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0,
                                                                J_r_hand_body_0, 
                                                                J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_z_local_up  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );  
  
     }    
  
  else if (last_command =="r_hand_z_world_fw")
     {
  //std::cout << "R Hand, Z World, moving forward"<< std::endl ;
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
//        //std::cout << " T_waist_l_hand_cmd = " <<  std::endl << T_waist_l_hand_cmd.toString() << std::endl;
//        //std::cout << " T_l_hand_des_local = " <<  std::endl << T_l_hand_des_local.toString() << std::endl;
//   
//        //std::cout << " T_waist_r1_foot_cmd = " <<  std::endl << T_waist_r1_foot_cmd.toString() << std::endl;
//        //std::cout << " T_r1_foot_des_local = " <<  std::endl << T_r1_foot_des_local.toString() << std::endl;
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
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
      else if (last_command =="r_hand_z_world_bk")
     {
  //std::cout << "R Hand, Z World, moving backward"<< std::endl ;
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
  
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
     }  
     
     else if (last_command =="r_hand_x_world_fw")
     {
  //std::cout << "R Hand, X World, moving forward"<< std::endl ;
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
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
  else if (last_command =="r_hand_x_world_bk")
     {
  //std::cout << "R Hand, X World, moving backward"<< std::endl ;
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
  
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }  
       else if (last_command =="r_hand_x_local_fw")
     {
  //std::cout << "R Hand, X Local, moving forward"<< std::endl ;
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
  
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }     
  else if (last_command =="r_hand_x_local_bk")
     {
  //std::cout << "R Hand, X Local, moving backward"<< std::endl ;
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
  
//  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
//  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }  
  else if (last_command =="r_hand_z_local_rot_ccw")
     {
  //std::cout << "R Hand, Z Local, Rotating counterclockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration

  //----------------------------------------------------------------------------------------------------------------
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des =  locoman::utils::Rot_z(0.1) ; //locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_z(-0.001) ;
     //std::cout << " Rot_des = " <<  std::endl << Rot_des.toString() << std::endl; 

  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 

  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
                                zer_3 , 
                                J_l_hand_body_0, J_r_hand_body_0, 
                                J_l_c1_body_0, J_r_c1_body_0, 
                                J_com_waist) ;
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );    
     }        
     
   else if (last_command =="r_hand_z_local_rot_cw")
     {
  //std::cout << "R Hand, Z Local, Rotating clockwise "<< std::endl ;
 // ------------------------------------------------------------------
  // Computing desired configuration 
  //----------------------------------------------------------------------------------------------------------------
  
  yarp::sig::Vector zer_3(3,0.0) ;
  yarp::sig::Matrix Rot_des = locoman::utils::Rot_z(-0.1) ;// locoman::utils::getRot( T_r_hand_w_0 *T_w_aw_0  ) * locoman::utils::Rot_z(0.001) ;
       //std::cout << " Rot_des = " <<  std::endl << Rot_des.toString() << std::endl; 
  yarp::sig::Matrix T_r_hand_des_local(4,4) ;  
  
  T_r_hand_des_local= locoman::utils::Homogeneous( Rot_des, zer_3) ; 
  // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                Eye_4,  Eye_4,             //T_l1_foot_des_local, T_r1_foot_des_local , 
                                zer_3 , 
                                J_l_hand_body_0, J_r_hand_body_0, 
                                J_l_c1_body_0, J_r_c1_body_0, 
                                J_com_waist) ;
//  q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
//  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
     }        
  else if (last_command =="r_hand_x_local_rot_ccw")
     {
  //std::cout << "R Hand, X Local, Rotating counterclockwise "<< std::endl ;
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
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
//  q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }        
   else if (last_command =="r_hand_x_local_rot_cw")
     {
  //std::cout << "R Hand, X Local, Rotating clockwise "<< std::endl ;
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
 // q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
 // q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );   
  
     }      
      else if  (last_command =="r_hand_x_local_fw_2")
     {
  //std::cout << "R Hand, X Local, moving Forward"<< std::endl ;
 // ------------------------------------------------------------------

        
  
  yarp::sig::Matrix T_temp_2 = Eye_4 ;
  T_temp_2[0][3] = 0.0008 ;
   
  yarp::sig::Vector zer_3(3,0.0) ;

   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks( Eye_4, T_temp_2, //T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  Eye_4,                            //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0,
                                                                J_r_hand_body_0, 
                                                                J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
  robot.move29( q_ref_ToMove );  
     }  
           else if  (last_command =="r_hand_y_local_bk")
     {
   std::cout << "R Hand, Y Local, moving Forward"<< std::endl ;
 // ------------------------------------------------------------------

  yarp::sig::Matrix T_temp_2 = Eye_4 ;
  T_temp_2[1][3] = 0.0003 ;
   
  yarp::sig::Vector zer_3(3,0.0) ;

   // ------------------------------------------------------------------
  yarp::sig::Vector d_q_r_hand_z_world_fw = locoman::utils::WB_Cartesian_Tasks(   Eye_4, T_temp_2,//T_r_hand_des_local, //T_l_hand_des_local, T_r_hand_des_local, 
                                                                Eye_4,  Eye_4,                            //T_l1_foot_des_local, T_r1_foot_des_local , 
                                                                zer_3 , 
                                                                J_l_hand_body_0,
                                                                J_r_hand_body_0, 
                                                                J_l_c1_body_0, 
                                                                J_r_c1_body_0, 
                                                                J_com_waist) ;
  
  //q_ref_ToMove = q_motor_side + (1.0/1.0) * d_q_r_hand_z_world_fw  ;  
  
  q_ref_ToMove = q_senseRefFeedback +  (1.0/1.0)* d_q_r_hand_z_world_fw  ;    // q in open loop
  //q_current_open_loop =  q_current_open_loop +  (1.0/1.0)*d_q_r_hand_z_world_fw  ; 
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
      
   //std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   //std::cout << " FC_DES = " <<  std::endl << FC_DES.toString() << std::endl; 
   //std::cout << " FC_DES_LEFT_sensor = " <<  std::endl << FC_DES_LEFT_sensor.toString() << std::endl; 
   //std::cout << " FC_DES_RIGHT_sensor = " <<  std::endl << FC_DES_RIGHT_sensor.toString() << std::endl;  
   //std::cout << " norm(FC_DES_sens - FC_sens)  =  "<< std::endl << norm(FC_DES_LEFT_sensor- FC_FILTERED_LEFT_sensor) + norm(FC_DES_RIGHT_sensor- FC_FILTERED_RIGHT_sensor) << std::endl  ; 
   //std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl;    
   //std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
  
 
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
  //std::cout << " err = "  << err << std::endl; 

  //std::cout << " alpha = "  << alpha << std::endl; 

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
 //     //std::cout << i->first << std::endl;
// Alternative formulations for the iterators      
// //std::cout << i->second->getReferenceFrame() << std::endl;     
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


