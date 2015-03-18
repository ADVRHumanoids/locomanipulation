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

using namespace yarp::math;

locoman_control_thread::locoman_control_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph ):
    control_thread( module_prefix, rf, ph ), command_interface(module_prefix)
{
    left_arm_joints = robot.left_arm.getNumberOfJoints();
    omega = 0.1;
    phi = 10;
    tick = 0;
    max_vel=20;
    left_arm_configuration.resize(left_arm_joints);
    //
    right_arm_joints = robot.right_arm.getNumberOfJoints();
    right_arm_configuration.resize(right_arm_joints);
    torso_joints = robot.torso.getNumberOfJoints();
    torso_configuration.resize(torso_joints);
    left_leg_joints = robot.left_leg.getNumberOfJoints();
    left_leg_configuration.resize(left_leg_joints);
    right_leg_joints = robot.right_leg.getNumberOfJoints();
    right_leg_configuration.resize(right_leg_joints);
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
    
    robot.idynutils.setFloatingBaseLink("base_link");
    yarp::sig::Vector q_current = robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_current, true);
    
    robot.setPositionDirectMode();
    
    //---------------------------------------------------------------------------//
        
    yarp::sig::Vector q_motor_0(robot.getNumberOfJoints() ) ;		    
     
    q_motor_0 = senseMotorPosition() ;

    yarp::sig::Vector q_des(robot.getNumberOfJoints() ) ;		    
    yarp::sig::Vector q_right_arm_des(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_left_arm_des(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_torso_des(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_right_leg_des(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_left_leg_des(robot.left_leg.getNumberOfJoints()) ;    
    
    q_des = q_motor_0; 
    
    robot.fromIdynToRobot(  q_des,
                            q_right_arm_des,
                            q_left_arm_des,
                            q_torso_des,
                            q_right_leg_des,
                            q_left_leg_des  ) ; 
    
    q_right_arm_des = right_arm_configuration  ; 			    
    q_left_arm_des  = left_arm_configuration   ;
    q_torso_des     = torso_configuration      ;
    
    robot.fromRobotToIdyn( q_right_arm_des ,
                           q_left_arm_des  ,
                           q_torso_des     ,
                           q_right_leg_des ,
                           q_left_leg_des  ,
                           q_des            );     
     
    yarp::sig::Vector q_motor_act(robot.getNumberOfJoints() ) ;		    
    yarp::sig::Vector q_motor_right_arm_act(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_motor_left_arm_act(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_motor_torso_act(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_motor_right_leg_act(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_motor_left_leg_act(robot.left_leg.getNumberOfJoints()) ;

    yarp::sig::Vector q_ref_ToMove(robot.getNumberOfJoints()) ;     
       
    yarp::sig::Vector d_q_des(robot.getNumberOfJoints()) ;     
    yarp::sig::Vector d_q_des_right_arm(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector d_q_des_left_arm(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector d_q_des_torso(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector d_q_des_right_leg(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector d_q_des_left_leg(robot.left_leg.getNumberOfJoints()) ;  
    
    d_q_des = (q_des - q_motor_0)/100 ;
    
    robot.move(q_des) ; 
//     
    q_motor_act = senseMotorPosition() ;
    double err_0 = norm(q_motor_act- q_des) +0.1 ;
    double err_1 = norm(q_motor_act- q_des)  ;
  
    while ( norm(q_motor_act- q_des)>0.01 && fabs(err_0-err_1)>0.000001 ) //( ( (norm(q_motor_act- q_des)>0.01)  )) // &&  ( abs(err_0-err_1)>0.000001 )) )
    {




   // std::cout << " err_0 = " <<  err_0 << std::endl; 
    err_0 = err_1 ;
    usleep(100*1000) ;
    q_motor_act = senseMotorPosition() ;
    err_1 = norm(q_motor_act- q_des)  ;
    std::cout << " norm(q_motor_act- q_des) =  " <<  norm(q_motor_act- q_des) << std::endl; 
    std::cout << " fabs(err_0-err_1) =  " <<  fabs(err_0-err_1) << std::endl; 
   // std::cout << " err_1 = " << err_1 << std::endl; 
  //  std::cout << "  ( fabs(err_0-err_1)<0.00001 ) =  " <<   ( fabs(err_0-err_1)<0.00001 ) << std::endl; 
   // std::cout << "  ( fabs(err_0-err_1)>0.000001 )  =  " <<   ( fabs(err_0-err_1)>0.000001 ) << std::endl; 
   // std::cout << "  ( (err_0-err_1) )  =  " <<   ( (err_0-err_1)) << std::endl; 
   // std::cout << "  ( fabs(err_0-err_1) )  =  " <<   fabs( (err_0-err_1)) << std::endl; 

//    std::cout << "  ( (norm(q_motor_act- q_des)>0.01  &&  ( fabs(err_0-err_1)>0.000001 )) =  " <<  ( norm(q_motor_act- q_des)>0.01 && fabs(err_0-err_1)>0.000001 )  << std::endl; 
 //   std::cout << "  ( 1 &&  1) =  " <<  ( 1 && 1 )  << std::endl; 
     continue ; 
    }
    std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl; 
    std::cout << " final fabs(err_0-err_1) =  " <<  fabs(err_0-err_1) << std::endl; 

    //std::cout << " left_arm  =  " <<  left_arm_configuration.toString() << std::endl; 

    usleep(5000*1000) ; // usleep(milliseconds*1000)
    // robot.left_arm.move(q_ref_ToMove_left_arm);
    return true;
}


yarp::sig::Matrix locoman_control_thread::getKq()
{
    yarp::sig::Vector Kq_vec_right_arm(  robot.right_arm.getNumberOfJoints() ) ;
    yarp::sig::Vector Kq_vec_left_arm(   robot.left_arm.getNumberOfJoints() ) ;  
    yarp::sig::Vector Kq_vec_torso(      robot.torso.getNumberOfJoints() ) ;
    yarp::sig::Vector Kq_vec_right_leg(  robot.right_leg.getNumberOfJoints() ) ;
    yarp::sig::Vector Kq_vec_left_leg(   robot.left_leg.getNumberOfJoints() ) ;
    // RIGHT ARM    
    Kq_vec_right_arm[0] = 1000.0 ;
    Kq_vec_right_arm[1] = 1000.0 ;
    Kq_vec_right_arm[2] = 600.0 ;
    Kq_vec_right_arm[3] = 1000.0 ;
    Kq_vec_right_arm[4] = 100.0 ;
    Kq_vec_right_arm[5] = 100.0 ;
    Kq_vec_right_arm[6] = 10.0 ;
    // LEFT ARM   
    Kq_vec_left_arm[0] = 1000.0 ;
    Kq_vec_left_arm[1] = 1000.0 ;
    Kq_vec_left_arm[2] = 600.0 ;
    Kq_vec_left_arm[3] = 1000.0 ;
    Kq_vec_left_arm[4] = 100.0 ;
    Kq_vec_left_arm[5] = 100.0 ;
    Kq_vec_left_arm[6] = 10.0 ;
    //TORSO
    Kq_vec_torso[0] = 1000.0 ;
    Kq_vec_torso[1] = 1000.0 ;
    Kq_vec_torso[2] = 1000.0 ;
    // RIGHT LEG
    Kq_vec_right_leg[0] = 3000.0 ;
    Kq_vec_right_leg[1] = 5000.0 ;
    Kq_vec_right_leg[2] = 3000.0 ;
    Kq_vec_right_leg[3] = 3000.0 ;
    Kq_vec_right_leg[4] = 4000.0 ;
    Kq_vec_right_leg[5] = 3000.0 ;
    // LEFT LEG
    Kq_vec_left_leg[0] = 3000.0 ;
    Kq_vec_left_leg[1] = 5000.0 ;
    Kq_vec_left_leg[2] = 3000.0 ;
    Kq_vec_left_leg[3] = 3000.0 ;
    Kq_vec_left_leg[4] = 4000.0 ;
    Kq_vec_left_leg[5] = 3000.0 ;
    //
    yarp::sig::Vector Kq_vec( robot.getNumberOfJoints()  )  ;     
    robot.fromRobotToIdyn( Kq_vec_right_arm ,
                           Kq_vec_left_arm  ,
                           Kq_vec_torso  ,
                           Kq_vec_right_leg ,
                           Kq_vec_left_leg  ,
                           Kq_vec ); 
    yarp::sig::Matrix Kq_matrix(  robot.getNumberOfJoints(), robot.getNumberOfJoints() )  ; 
    Kq_matrix.diagonal(  Kq_vec ) ;
  
    return Kq_matrix ;
}




yarp::sig::Vector locoman_control_thread::senseMotorPosition()
{
    yarp::sig::Vector q_link = robot.sensePosition() ;
    yarp::sig::Vector tau  = robot.senseTorque() ;
    //     
    
    
    
    /*yarp::sig::Vector Cq_vec_right_arm(  robot.right_arm.getNumberOfJoints() ) ;
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
    Cq_matrix.diagonal(  Cq_vec ) ;  */
    yarp::sig::Matrix Kq_matrix = getKq() ;
    yarp::sig::Matrix Cq_matrix = yarp::math::luinv(Kq_matrix) ;
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
    T_ab[3][3] = 1 ; 
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

yarp::sig::Vector locoman_control_thread::SkewToVect(const yarp::sig::Matrix Skew)
{
    yarp::sig::Vector Vect(3) ;
    
    Vect[0] = Skew[2][1] ;
    Vect[1] = Skew[0][2] ;
    Vect[2] = Skew[1][0] ;
    return Vect ;
}

yarp::sig::Matrix locoman_control_thread::AdjToPose(const yarp::sig::Matrix Adj)
{
    yarp::sig::Matrix T(4,4) ;
    T.setSubmatrix( Adj.submatrix(0,2,0,2), 0,0 ); // = Adj.submatrix(0,2,0,2) ;
    yarp::sig::Matrix R = getRot(T)  ;
    //std::cout << "R = "  << R.toString()  << std::endl ;
    yarp::sig::Matrix Temp = Adj.submatrix(0,2,3,5) ;
    //std::cout << "Temp = "  << Temp.toString()  << std::endl ;
    yarp::sig::Matrix Temp_2 =  Temp*R.transposed(); 
    //std::cout << "Temp_2 = "  <<  Temp_2.toString()  << std::endl ;
    yarp::sig::Vector d = SkewToVect(Temp_2) ;
    //std::cout << "d = "  << d.toString()  << std::endl ;
    yarp::sig::Matrix d_matr(3,1) ;
    d_matr[0][0] = d[0] ;
    d_matr[1][0] = d[1] ;    
    d_matr[2][0] = d[2] ;
    //    std::cout << "d_matr = "  << d_matr.toString()  << std::endl ;
    T.setSubmatrix(d_matr , 0,3)  ; 
    //    std::cout << "T = "  << T.toString()  << std::endl ;
    T[3][3] = 1 ;
    return T ;
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
    //::cout << i->second->getReferenceFrame() << std::endl;

    for(RobotUtils::ftPtrMap::iterator i = fts.begin() ;  i != fts.end(); i++)
    {
      std::cout << i->first << std::endl;
// Alternative formulations for the iterators      
// std::cout << i->second->getReferenceFrame() << std::endl;     
//      (*i).second->getReferenceFrame();
//      fts[i->first]->getReferenceFrame(); 
    }
  
  //--------------------------------------------------------------------//
    //Getting Sensor Measures

    yarp::sig::Vector ft_r_ankle(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    
    yarp::sig::Vector ft_l_ankle(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    
  /* // std::cout << "start r_ankle" << std::endl;
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
    } */
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
 
    int l_ankle_index = model.iDyn3_model.getLinkIndex("l_ankle") ; // sensors are placed in _ankle in the model
    int l_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link");
    int l_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");
    int l_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link");
    int l_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
    
    int r_ankle_index = model.iDyn3_model.getLinkIndex("r_ankle") ;
    int r_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
    int r_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");
    int r_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
    int r_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
    

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
    
    /*std::cout << " T_l_l1 = " << T_l_l1.toString() << std::endl ;
    std::cout << " T_l_l2 = " << T_l_l2.toString() << std::endl ;
    std::cout << " T_l_l3 = " << T_l_l3.toString() << std::endl ;
    std::cout << " T_l_l4 = " << T_l_l4.toString() << std::endl ;  */
    
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

    T_r_r1 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_upper_left_link  ; 
    T_r_r2 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_upper_right_link ; 
    T_r_r3 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_lower_left_link  ;
    T_r_r4 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_lower_right_link ;
    
    std::cout << "  ---------------------------------------- "  << std::endl ;     
    
    yarp::sig::Matrix Ad_g_rr1(  6 ,   6 ) ;
    yarp::sig::Matrix Ad_g_MT_rr1(  6 ,   6 ) ;
    
    Ad_g_rr1 = Adjoint(T_r_r1) ;
    Ad_g_MT_rr1 = Adjoint_MT(T_r_r1) ;

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

    yarp::sig::Vector fc_r_contacts =  yarp::math::pinv( map_r_fcToSens)  *  ft_r_ankle     ;
    yarp::sig::Vector fc_l_contacts =  yarp::math::pinv( map_l_fcToSens)  *  ft_l_ankle     ;



     
     
     
     
						      
						      
						      
						      
						      
    
//--------------------------------------------------------------------------------------------------------//
//                 ----    Jacobians     -----    
//--------------------------------------------------------------------------------------------------------//
//-------------------------------------------------------------------------//						      
  
						      
						      
						       // Jacobian tests   
						        // Getting Jacobians of the left foot
        


  /*  yarp::sig::Matrix Jac_l_ankle_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) )  ; 
   // yarp::sig::Matrix Jac_l_foot_lower_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
   // yarp::sig::Matrix Jac_l_foot_lower_right_link_body( 6  , ( robot.getNumberOfJoints() + 6)  ) ; 
   // yarp::sig::Matrix Jac_l_foot_upper_left_link_body(  6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
   // yarp::sig::Matrix Jac_l_foot_upper_right_link_body( 6  , ( robot.getNumberOfJoints() + 6 ) ) ; 
    
    model.iDyn3_model.getJacobian( l_ankle_index, Jac_l_ankle_body, true ) ; //false= mixed version jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_left_link_index  , Jac_l_foot_lower_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_right_link_index , Jac_l_foot_lower_right_link_body , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_upper_left_link_index  , Jac_l_foot_upper_left_link_body  , true  ) ; //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_upper_right_link_index , Jac_l_foot_upper_right_link_body , true  ) ; //true= body jacobian

    // Getting Jacobians of the right foot
   
    
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
    
    yarp::sig::Matrix Jac_l_ankle_mix_1(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    
    model.iDyn3_model.getJacobian( l_ankle_index, Jac_l_ankle_mix_1, false ) ;      

   // std::cout << "Jac_l_ankle_mix_1 = " <<  std::endl << Jac_l_ankle_mix_1.toString() << std::endl; 
     
    
    //Verifying equalityies.........................................
    
    yarp::sig::Matrix Eye_3( 3 , 3 ) ;
    Eye_3.eye() ;
    
    yarp::sig::Vector zer_3( 3  ) ;
    zer_3[0] = 0 ;
    zer_3[1] = 0 ;
    zer_3[2] = 0 ;
    yarp::sig::Matrix Jac_l_ankle_mix_2(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_ankle_mix_3(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_ankle_mix_4(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_ankle_mix_5(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;
    
    //------------------------------------------------------------
    Jac_l_ankle_mix_2 = Adjoint( iHomogeneous(Homogeneous(getRot(T_w_l_ankle),zer_3)))*Jac_l_ankle_body; 
    yarp::sig::Matrix Temp_2 = Jac_l_ankle_body.submatrix(0,5,0,5) ; 
   // std::cout << "Temp_2 = " << Temp_2.toString() <<  std::endl ;
    
    yarp::sig::Matrix Temp_3 = Jac_l_ankle_mix_1.submatrix(0,5,0,5) ;       */
    // std::cout << "Temp_3 = " << Temp_3.toString() <<  std::endl ;
   
    
    // WAIST
    int waist_index = model.iDyn3_model.getLinkIndex("Waist") ;
    yarp::sig::Matrix  T_w_waist = model.iDyn3_model.getPosition(waist_index) ;
    //std::cout << "T_w_waist = " <<  std::endl << T_w_waist.toString() << std::endl; 
    
    yarp::sig::Matrix  T_waist_w = iHomogeneous(T_w_waist) ;
    //std::cout << "T_waist_w = " <<  std::endl << T_waist_w.toString() << std::endl; 
    
    //yarp::sig::Matrix   Test_ihom = T_waist_w*T_w_waist +T_w_waist*T_waist_w;
    //std::cout << "Test_ihom = " <<  std::endl << Test_ihom.toString() << std::endl;
    
    yarp::sig::Matrix Jac_waist_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( waist_index, Jac_waist_mix, false ) ;  
    //std::cout << "Jac_waist_mix = " <<  std::endl << Jac_waist_mix.toString() << std::endl; 

    //std::cout << "Jac_waist_mix = " <<  std::endl << Jac_waist_mix.submatrix(0,5,0,5).toString() << std::endl; 
    //std::cout << " Jac_waist_mix = " <<  std::endl << Jac_waist_mix.submatrix(0,1,0,29).toString() << std::endl; 

    yarp::sig::Matrix Jac_waist_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    model.iDyn3_model.getJacobian( waist_index, Jac_waist_body, true ) ; 
    
    //std::cout << "Jac_waist_body = " <<  std::endl << Jac_waist_body.submatrix(0,5,0,5).toString() << std::endl; 
    //std::cout << "Jac_waist_body = " <<  std::endl << Jac_waist_body.submatrix(0,1,0,29).toString() << std::endl; 

    
    //std::cout <<"Virtual_Pose_waist = " <<  std::endl<<  AdjToPose(Jac_waist_body.submatrix(0,5,0,5)).toString()<<  std::endl;
    
     // L_SOLE  
    std::cout << " -----------------------------------------------" <<   std::endl; 

    int l_sole_index = model.iDyn3_model.getLinkIndex("l_sole") ;
    yarp::sig::Matrix Jac_l_sole_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( l_sole_index, Jac_l_sole_mix, false ) ;  
    //std::cout << " Jac_l_sole_mix = " <<  std::endl << Jac_l_sole_mix.submatrix(0,5,0,5).toString() << std::endl; 
    //std::cout << " Jac_l_sole_mix = " <<  std::endl << Jac_l_sole_mix.submatrix(0,1,0,29).toString() << std::endl; 

    
    yarp::sig::Matrix  T_w_l_sole = model.iDyn3_model.getPosition(l_sole_index) ;
    //std::cout << " T_w_l_sole = " <<  std::endl << T_w_l_sole.toString() << std::endl; 

    yarp::sig::Vector d_w_l_sole(3) ;
    yarp::sig::Vector d_w_waist(3)  ; 
    d_w_l_sole[0] = T_w_l_sole[0][3] ;
    d_w_l_sole[1] = T_w_l_sole[1][3] ;
    d_w_l_sole[2] = T_w_l_sole[2][3] ;

    d_w_waist[0] = T_w_waist[0][3] ;    //    T_w_waist
    d_w_waist[1] = T_w_waist[1][3] ; 
    d_w_waist[2] = T_w_waist[2][3] ;
    
    yarp::sig::Vector d_w_l_sole_waist =  d_w_waist - d_w_l_sole ;
    //std::cout << " d_w_l_sole_waist = " <<  std::endl << d_w_l_sole_waist.toString() << std::endl; 

    //std::cout << " -----------------------------------------------" <<   std::endl; 
    
    yarp::sig::Matrix T_waist_l_sole = iHomogeneous(T_w_waist)*T_w_l_sole ;    
    //std::cout <<"T_waist_l_sole = " <<  std::endl<<  T_waist_l_sole.toString()<<  std::endl;
    yarp::sig::Matrix T_l_sole_waist = iHomogeneous(T_waist_l_sole)  ;    
    //std::cout <<"T_l_sole_waist = " <<  std::endl<<  T_l_sole_waist.toString()<<  std::endl;

    //std::cout << "  ---------------------------------------- "  << std::endl ;     
    
    yarp::sig::Matrix Jac_l_sole_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    model.iDyn3_model.getJacobian( l_sole_index, Jac_l_sole_body, true ) ; 
    //std::cout << "Jac_l_sole_body = " <<  std::endl << Jac_l_sole_body.submatrix(0,5,0,5).toString() << std::endl;     
        
    //yarp::sig::Matrix Adj_l_sole =    Jac_l_sole_body.submatrix(0,5,0,5) *    yarp::math::luinv(Jac_l_sole_mix.submatrix(0,5,0,5)) ; 
    
    //std::cout <<"Virtual_Pose_l_sole = " <<  std::endl<<  AdjToPose(Jac_l_sole_mix.submatrix(0,5,0,5)).toString()<<  std::endl;

    //std::cout <<"Virtual_Pose_l_sole_body = " <<  std::endl<<  AdjToPose(Jac_l_sole_body.submatrix(0,5,0,5)).toString()<<  std::endl;
    
    // yarp::sig::Matrix Adj_l_sole =    Jac_l_sole_body.submatrix(0,5,0,5) *    yarp::math::luinv(Jac_l_sole_mix.submatrix(0,5,0,5)) ; 

    //std::cout <<"Virtual_Pose_sole_mix_body = " <<  std::endl<<  AdjToPose( Adj_l_sole ).toString()<<  std::endl;

   // std::cout <<" Transforming L_Sole MIX into BODY " <<   std::endl;

    yarp::sig::Matrix  R_l_sole_w =  iHomogeneous(T_w_l_sole) ;
    //std::cout << " R_l_sole_w = " <<  std::endl << R_l_sole_w.toString() << std::endl; 
    
    R_l_sole_w[0][3] = 0 ;
    R_l_sole_w[1][3] = 0 ;
    R_l_sole_w[2][3] = 0 ;
    
    //std::cout << " R_l_sole_w = " <<  std::endl << R_l_sole_w.toString() << std::endl; 

    yarp::sig::Matrix Jac_l_sole_body_computed = Adjoint(R_l_sole_w) * Jac_l_sole_mix.submatrix(0,5,0,5) ;
   // std::cout << " Jac_l_sole_body_computed = " <<  std::endl << Jac_l_sole_body_computed.toString() << std::endl; 

    //yarp::sig::Matrix Jac_l_sole_body_computed_2 = Adjoint( model.iDyn3_model.getPosition(l_sole_index, waist_index) )* Jac_waist_body.submatrix(0,5,0,5) ;
    //std::cout << " Jac_l_sole_body_computed_2 = " <<  std::endl << Jac_l_sole_body_computed_2.toString() << std::endl; 
     
   // R_SOLE  
   // std::cout << " -----------------------------------------------" <<   std::endl;     
    int r_sole_index = model.iDyn3_model.getLinkIndex("r_sole") ;
    
    yarp::sig::Matrix Jac_r_sole_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( r_sole_index, Jac_r_sole_mix, false ) ;  
   // std::cout << " Jac_r_sole_mix = " <<  std::endl << Jac_r_sole_mix.submatrix(0,5,0,5).toString() << std::endl; 

    yarp::sig::Matrix  T_w_r_sole = model.iDyn3_model.getPosition( r_sole_index) ;
   // std::cout << " T_w_r_sole = " <<  std::endl << T_w_r_sole.toString() << std::endl; 

    yarp::sig::Vector d_w_r_sole(3) ;
    d_w_r_sole[0] = T_w_r_sole[0][3] ;
    d_w_r_sole[1] = T_w_r_sole[1][3] ;
    d_w_r_sole[2] = T_w_r_sole[2][3] ;
    
    yarp::sig::Vector d_w_r_sole_waist =  d_w_waist - d_w_r_sole ;
  //  std::cout << " d_w_r_sole_waist = " <<  std::endl << d_w_r_sole_waist.toString() << std::endl; 
    
    //std::cout << " -----------------------------------------------" <<   std::endl;     
   
    yarp::sig::Matrix Jac_r_sole_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    model.iDyn3_model.getJacobian( r_sole_index, Jac_r_sole_body, true ) ; 
   // std::cout << "Jac_r_sole_body = " <<  std::endl << Jac_r_sole_body.submatrix(0,5,0,5).toString() << std::endl; 
    
     yarp::sig::Matrix T_waist_r_sole = iHomogeneous(T_w_waist)*T_w_r_sole ;    
  //  std::cout <<"T_waist_r_sole = " <<  std::endl<<  T_waist_r_sole.toString()<<  std::endl;
    yarp::sig::Matrix T_r_sole_waist = iHomogeneous(T_waist_r_sole)  ;    
   // std::cout <<"T_r_sole_waist = " <<  std::endl<<  T_r_sole_waist.toString()<<  std::endl;    
    //yarp::sig::Matrix Temp_rocchi_2 = model.iDyn3_model.getPosition(r_sole_index, waist_index) ;
    //std::cout <<"Temp_rocchi_2 = " <<  std::endl<<  Temp_rocchi_2.toString()<<  std::endl;
    
    yarp::sig::Matrix  p_r_w_a =  -1.0* T_w_r_sole.submatrix(0,2,3,3) +  T_w_waist.submatrix(0,2,3,3) ;
  //  std::cout <<"p_r_w_a = " <<  std::endl<<  p_r_w_a.toString()<<  std::endl;    
 
   // std::cout << "  ---------------------------------------- "  << std::endl ;     
   
  //  std::cout <<" Transforming R_Sole MIX into BODY " <<   std::endl;

    yarp::sig::Matrix  R_r_sole_w =  iHomogeneous(T_w_r_sole) ;
    //std::cout << " R_r_sole_w = " <<  std::endl << R_r_sole_w.toString() << std::endl; 
    
    R_r_sole_w[0][3] = 0 ;
    R_r_sole_w[1][3] = 0 ;
    R_r_sole_w[2][3] = 0 ;
    
    yarp::sig::Matrix Jac_r_sole_body_computed = Adjoint(R_r_sole_w)* Jac_r_sole_mix.submatrix(0,5,0,5) ;
 //   std::cout << " Jac_r_sole_body_computed = " <<  std::endl << Jac_r_sole_body_computed.toString() << std::endl; 
    
    // yarp::sig::Matrix Jac_r_sole_body_computed_2 = Adjoint( model.iDyn3_model.getPosition( r_sole_index, waist_index) )* Jac_waist_body.submatrix(0,5,0,5) ;
    // std::cout << " Jac_r_sole_body_computed_2 = " <<  std::endl << Jac_r_sole_body_computed_2.toString() << std::endl; 
 

 // L_HAND_UPPER_RIGHT_LINK  
 //  std::cout << " -----------------------------------------------" <<   std::endl; 
    int l_hand_upper_right_index = model.iDyn3_model.getLinkIndex("l_hand_upper_right_link") ;
    yarp::sig::Matrix  T_w_l_hand_upper_right = model.iDyn3_model.getPosition(l_hand_upper_right_index) ;
 //   std::cout << " T_w_l_hand_upper_right = " <<  std::endl << T_w_l_hand_upper_right.toString() << std::endl; 
    yarp::sig::Matrix  T_l_hand_upper_right_w =  iHomogeneous(T_w_l_hand_upper_right) ;
 //   std::cout << " T_l_hand_upper_right_w = " <<  std::endl << T_l_hand_upper_right_w.toString() << std::endl; 
    
    yarp::sig::Matrix Jac_l_hand_upper_right_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( l_hand_upper_right_index, Jac_l_hand_upper_right_mix, false ) ;  
 //   std::cout << " Jac_l_hand_upper_right_mix = " <<  std::endl << Jac_l_hand_upper_right_mix.submatrix(0,5,0,5).toString() << std::endl; 
   
    yarp::sig::Matrix T_waist_l_hand_upper_right = iHomogeneous(T_w_waist)*T_w_l_hand_upper_right ;    
//    std::cout <<"T_waist_l_hand_upper_right = " <<  std::endl<<  T_waist_l_hand_upper_right.toString()<<  std::endl;
    yarp::sig::Matrix T_l_hand_upper_right_waist = iHomogeneous(T_waist_l_hand_upper_right)  ;    
//    std::cout <<"T_l_hand_upper_right_waist = " <<  std::endl<<  T_l_hand_upper_right_waist.toString()<<  std::endl; 
    
    yarp::sig::Matrix  p_l_hand_w_a =  -1.0* T_w_l_hand_upper_right.submatrix(0,2,3,3) +  T_w_waist.submatrix(0,2,3,3) ;
//    std::cout <<"p_l_hand_w_a = " <<  std::endl<<  p_l_hand_w_a.toString()<<  std::endl;    
    
    yarp::sig::Matrix Jac_l_hand_upper_right_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( l_hand_upper_right_index, Jac_l_hand_upper_right_body, true ) ;  
//    std::cout << " Jac_l_hand_upper_right_body = " <<  std::endl << Jac_l_hand_upper_right_body.submatrix(0,5,0,5).toString() << std::endl; 
    
    // R_HAND_UPPER_RIGHT_LINK  
//    std::cout << " -----------------------------------------------" <<   std::endl; 
    int r_hand_upper_right_index = model.iDyn3_model.getLinkIndex("r_hand_upper_right_link") ;
    yarp::sig::Matrix  T_w_r_hand_upper_right = model.iDyn3_model.getPosition(r_hand_upper_right_index) ;
 //   std::cout << " T_w_r_hand_upper_right = " <<  std::endl << T_w_r_hand_upper_right.toString() << std::endl; 
    yarp::sig::Matrix  T_r_hand_upper_right_w =  iHomogeneous(T_w_r_hand_upper_right) ;
 //   std::cout << " T_r_hand_upper_right_w = " <<  std::endl << T_r_hand_upper_right_w.toString() << std::endl; 

       
    yarp::sig::Matrix Jac_r_hand_upper_right_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( r_hand_upper_right_index, Jac_r_hand_upper_right_mix, false ) ;  
 //   std::cout << " Jac_l_hand_upper_right_mix = " <<  std::endl << Jac_r_hand_upper_right_mix.submatrix(0,5,0,5).toString() << std::endl; 
   
    yarp::sig::Matrix T_waist_r_hand_upper_right = iHomogeneous(T_w_waist)*T_w_r_hand_upper_right ;    
 //   std::cout <<"T_waist_r_hand_upper_right = " <<  std::endl<<  T_waist_r_hand_upper_right.toString()<<  std::endl;
    yarp::sig::Matrix T_r_hand_upper_right_waist = iHomogeneous(T_waist_r_hand_upper_right)  ;    
 //   std::cout <<"T_r_hand_upper_right_waist = " <<  std::endl<<  T_r_hand_upper_right_waist.toString()<<  std::endl; 
    
    yarp::sig::Matrix  p_r_hand_w_a =  -1.0* T_w_r_hand_upper_right.submatrix(0,2,3,3) +  T_w_waist.submatrix(0,2,3,3) ;
 //   std::cout <<"p_r_hand_w_a = " <<  std::endl<<  p_r_hand_w_a.toString()<<  std::endl; 
    
    yarp::sig::Matrix Jac_r_hand_upper_right_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( r_hand_upper_right_index, Jac_r_hand_upper_right_body, true ) ;  
//    std::cout << " Jac_r_hand_upper_right_body = " <<  std::endl << Jac_r_hand_upper_right_body.submatrix(0,5,0,5).toString() << std::endl; 
    
    
  /*  // yarp::sig::Matrix Temp_rocchi_4 = model.iDyn3_model.getPosition(r_hand_upper_right_index, waist_index) ;
    //std::cout <<"Temp_rocchi_4 = " <<  std::endl<<  Temp_rocchi_4.toString()<<  std::endl;                          
    */
    
   // IMU_LINK  
  //  std::cout << " -----------------------------------------------" <<   std::endl;     
    int imu_link_index = model.iDyn3_model.getLinkIndex("imu_link") ;
    
    yarp::sig::Matrix Jac_imu_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( imu_link_index, Jac_imu_mix, false ) ;  
   // std::cout << " Jac_imu_mix = " <<  std::endl << Jac_imu_mix.submatrix(0,5,0,5).toString() << std::endl; 

    yarp::sig::Matrix  T_w_imu  = model.iDyn3_model.getPosition( imu_link_index) ;
   // std::cout << " T_w_imu = " <<  std::endl << T_w_imu.toString() << std::endl; 
    
    yarp::sig::Matrix  T_imu_w  = iHomogeneous(T_w_imu) ;  
    
    //RobotUtils::ftPtrMap fts = robot.getftSensors();

    int a = 0 ; 
    if( robot.hasIMU()) a=1 ;
    
   // std::cout << " hasIMU " <<  std::endl << a << std::endl; 
    
    RobotUtils::IMUPtr IMU_ptr = robot.getIMU()  ;
    yarp::sig::Vector IMU_sense = IMU_ptr->sense(); ;
   // yarp::sig::Vector IMU_sense_RPY(3) ; 
    yarp::sig::Vector IMU_sense_lin_acc(3) ; 
   // yarp::sig::Vector IMU_sense_ang_vel(3) ; 
        
    /* IMU_sense_RPY[0] = IMU_sense[0] ;
    IMU_sense_RPY[1] = IMU_sense[1] ;
    IMU_sense_RPY[2] = IMU_sense[2] ; */
  
    IMU_sense_lin_acc[0] = IMU_sense[3] ;    
    IMU_sense_lin_acc[1] = IMU_sense[4] ;    
    IMU_sense_lin_acc[2] = IMU_sense[5] ;    
 
    /*  IMU_sense_ang_vel[0] = IMU_sense[6] ;    
    IMU_sense_ang_vel[1] = IMU_sense[7] ;    
    IMU_sense_ang_vel[2] = IMU_sense[8] ;    */
    
    double norm_imu = norm(IMU_sense_lin_acc)     ;
    yarp::sig::Vector z_imu_aw =  IMU_sense_lin_acc/norm_imu ;
    double norm_z = norm(z_imu_aw)  ;
   
    yarp::sig::Matrix z_imu_aw_matr(3,1);
    z_imu_aw_matr[0][0] = z_imu_aw[0] ;
    z_imu_aw_matr[1][0] = z_imu_aw[1] ;
    z_imu_aw_matr[2][0] = z_imu_aw[2] ;   
    
    yarp::sig::Vector x_imu_w(3) ;
    x_imu_w[0] =  T_imu_w[0][0];
    x_imu_w[1] =  T_imu_w[1][0];
    x_imu_w[2] =  T_imu_w[2][0];

    yarp::sig::Matrix Null_z_tr =  nullspaceProjection(z_imu_aw_matr.transposed()) ;
    yarp::sig::Vector x_imu_aw = Null_z_tr*x_imu_w  ;
  //  std::cout << " z_imu_aw_matr.transposed()  " <<  std::endl << z_imu_aw_matr.transposed().toString() << std::endl; 

    //std::cout << " Null_z_tr  " <<  std::endl << Null_z_tr.toString() << std::endl; 

    yarp::sig::Vector y_imu_aw(3) ;
    yarp::sig::Matrix R_imu_aw(3,3) ;    
    
    if (norm(x_imu_aw)>0.01)
    {
   // std::cout << " norm(z_imu_aw)  " <<  std::endl << norm(z_imu_aw)  << std::endl; 

    x_imu_aw = x_imu_aw/norm(x_imu_aw) ;   
    y_imu_aw = cross(z_imu_aw, x_imu_aw  );   

    // std::cout << " norm(x_imu_aw)  " <<  std::endl << norm(x_imu_aw)  << std::endl; 
    // std::cout << " norm(y_imu_aw)  " <<  std::endl << norm(y_imu_aw)  << std::endl; 
    // std::cout << " norm(z_imu_aw)  " <<  std::endl << norm(z_imu_aw)  << std::endl; 
    
    R_imu_aw[0][0] = x_imu_aw[0] ;
    R_imu_aw[1][0] = x_imu_aw[1] ;
    R_imu_aw[2][0] = x_imu_aw[2] ;

    R_imu_aw[0][1] = y_imu_aw[0] ;
    R_imu_aw[1][1] = y_imu_aw[1] ;
    R_imu_aw[2][1] = y_imu_aw[2] ;

    R_imu_aw[0][2] = z_imu_aw[0] ;
    R_imu_aw[1][2] = z_imu_aw[1] ;
    R_imu_aw[2][2] = z_imu_aw[2] ;
    }
    else   {   
    x_imu_aw[0] = Null_z_tr[0][0] ; 
    x_imu_aw[1] = Null_z_tr[1][0] ; 
    x_imu_aw[2] = Null_z_tr[2][0] ; 
         
    x_imu_aw = x_imu_aw/norm(x_imu_aw) ; 
    
    y_imu_aw = cross(z_imu_aw, x_imu_aw  );   

   // yarp::sig::Matrix R_imu_aw(3,3) ;    
    
    R_imu_aw[0][0] = x_imu_aw[0] ;
    R_imu_aw[1][0] = x_imu_aw[1] ;
    R_imu_aw[2][0] = x_imu_aw[2] ;

    R_imu_aw[0][1] = y_imu_aw[0] ;
    R_imu_aw[1][1] = y_imu_aw[1] ;
    R_imu_aw[2][1] = y_imu_aw[2] ;

    R_imu_aw[0][2] = z_imu_aw[0] ;
    R_imu_aw[1][2] = z_imu_aw[1] ;
    R_imu_aw[2][2] = z_imu_aw[2] ;
    }  

    
    yarp::sig::Vector d_imu_aw(3, 0.0) ;     
    
    yarp::sig::Matrix T_imu_aw = Homogeneous( R_imu_aw, d_imu_aw) ;
    yarp::sig::Matrix T_aw_imu = iHomogeneous(T_imu_aw) ;    

    yarp::sig::Matrix T_w_aw = T_w_imu * T_imu_aw ;
    yarp::sig::Matrix T_aw_w = iHomogeneous(T_w_aw) ;    
    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//    
    // Computing Jacobian respect to the the Auxiliary World {AW}
   
    
    yarp::sig::Matrix T_R_imu_w = T_imu_w;
    
    T_R_imu_w[0][3] = 0;
    T_R_imu_w[1][3] = 0;
    T_R_imu_w[2][3] = 0;
    
    yarp::sig::Matrix Jac_imu_body = Adjoint( T_R_imu_w ) * Jac_imu_mix ; 
    yarp::sig::Matrix Jac_spa_temp = Adjoint( T_aw_imu ) * Jac_imu_body  ; 

    yarp::sig::Matrix Eye_6(6,6) ;
    Eye_6.eye() ;
    
    Jac_spa_temp.setSubmatrix(Eye_6,0,0) ;
    yarp::sig::Matrix Jac_aw_imu =  Jac_spa_temp ; // Spatial Jacobian of the IMU link
    //
    // Computing spatial Jacobian ( i.e. in frame {AW} ) for all the contacts, setting also the first block to identity
    
    yarp::sig::Matrix Jac_l_foot_upper_left_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_upper_right_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_left_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_right_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;

    yarp::sig::Matrix Jac_r_foot_upper_left_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_upper_right_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_left_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_right_link_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;
    
    model.iDyn3_model.getJacobian( l_foot_upper_left_link_index  , Jac_l_foot_upper_left_link_mix  , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_upper_right_link_index , Jac_l_foot_upper_right_link_mix , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_left_link_index  , Jac_l_foot_lower_left_link_mix  , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_foot_lower_right_link_index , Jac_l_foot_lower_right_link_mix , false  ) ; //false= mixed version jacobian //true= body jacobian

    model.iDyn3_model.getJacobian( r_foot_upper_left_link_index  , Jac_r_foot_upper_left_link_mix  , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_upper_right_link_index , Jac_r_foot_upper_right_link_mix , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_lower_left_link_index  , Jac_r_foot_lower_left_link_mix  , false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_foot_lower_right_link_index , Jac_r_foot_lower_right_link_mix , false  ) ; //false= mixed version jacobian //true= body jacobian
   
    yarp::sig::Matrix  T_w_l_foot_upper_left_link_index  = model.iDyn3_model.getPosition(  l_foot_upper_left_link_index   ) ;
    yarp::sig::Matrix  T_w_l_foot_upper_right_link_index = model.iDyn3_model.getPosition(  l_foot_upper_right_link_index  ) ;
    yarp::sig::Matrix  T_w_l_foot_lower_left_link_index  = model.iDyn3_model.getPosition(  l_foot_lower_left_link_index   ) ;
    yarp::sig::Matrix  T_w_l_foot_lower_right_link_index = model.iDyn3_model.getPosition(  l_foot_lower_right_link_index  ) ;
    
    yarp::sig::Matrix  T_w_r_foot_upper_left_link_index  = model.iDyn3_model.getPosition(  r_foot_upper_left_link_index   ) ;
    yarp::sig::Matrix  T_w_r_foot_upper_right_link_index = model.iDyn3_model.getPosition(  r_foot_upper_right_link_index  ) ;
    yarp::sig::Matrix  T_w_r_foot_lower_left_link_index  = model.iDyn3_model.getPosition(  r_foot_lower_left_link_index   ) ;
    yarp::sig::Matrix  T_w_r_foot_lower_right_link_index = model.iDyn3_model.getPosition(  r_foot_lower_right_link_index  ) ;
       
    yarp::sig::Matrix  T_l_foot_upper_left_link_index_w  =  iHomogeneous(T_w_l_foot_upper_left_link_index  ) ;
    yarp::sig::Matrix  T_l_foot_upper_right_link_index_w =  iHomogeneous(T_w_l_foot_upper_right_link_index ) ;
    yarp::sig::Matrix  T_l_foot_lower_left_link_index_w  =  iHomogeneous(T_w_l_foot_lower_left_link_index  ) ;
    yarp::sig::Matrix  T_l_foot_lower_right_link_index_w =  iHomogeneous(T_w_l_foot_lower_right_link_index ) ;

    yarp::sig::Matrix  T_r_foot_upper_left_link_index_w  =  iHomogeneous(T_w_r_foot_upper_left_link_index  ) ;
    yarp::sig::Matrix  T_r_foot_upper_right_link_index_w =  iHomogeneous(T_w_r_foot_upper_right_link_index ) ;
    yarp::sig::Matrix  T_r_foot_lower_left_link_index_w  =  iHomogeneous(T_w_r_foot_lower_left_link_index  ) ;
    yarp::sig::Matrix  T_r_foot_lower_right_link_index_w =  iHomogeneous(T_w_r_foot_lower_right_link_index ) ;      
      
    yarp::sig::Matrix T_R_l_foot_upper_left_link_index_w  = T_l_foot_upper_left_link_index_w  ;
    yarp::sig::Matrix T_R_l_foot_upper_right_link_index_w = T_l_foot_upper_right_link_index_w ;
    yarp::sig::Matrix T_R_l_foot_lower_left_link_index_w  = T_l_foot_lower_left_link_index_w  ;
    yarp::sig::Matrix T_R_l_foot_lower_right_link_index_w = T_l_foot_lower_right_link_index_w ;

    yarp::sig::Matrix T_R_r_foot_upper_left_link_index_w  = T_r_foot_upper_left_link_index_w  ;
    yarp::sig::Matrix T_R_r_foot_upper_right_link_index_w = T_r_foot_upper_right_link_index_w ;
    yarp::sig::Matrix T_R_r_foot_lower_left_link_index_w  = T_r_foot_lower_left_link_index_w  ;
    yarp::sig::Matrix T_R_r_foot_lower_right_link_index_w = T_r_foot_lower_right_link_index_w ;

    yarp::sig::Matrix d_zero_31(3,1) ;
    
    d_zero_31[0][0] = 0  ;
    d_zero_31[1][0] = 0  ;
    d_zero_31[2][0] = 0  ;
    
    T_R_l_foot_upper_left_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_l_foot_upper_right_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_l_foot_lower_left_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_l_foot_lower_right_link_index_w.setSubmatrix(d_zero_31,0,3) ;

    T_R_r_foot_upper_left_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_r_foot_upper_right_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_r_foot_lower_left_link_index_w.setSubmatrix(d_zero_31,0,3) ;
    T_R_r_foot_lower_right_link_index_w.setSubmatrix(d_zero_31,0,3) ;

    yarp::sig::Matrix Jac_l_foot_upper_left_link_body  = Adjoint( T_R_l_foot_upper_left_link_index_w  ) * Jac_l_foot_upper_left_link_mix  ; 
    yarp::sig::Matrix Jac_l_foot_upper_right_link_body = Adjoint( T_R_l_foot_upper_right_link_index_w ) * Jac_l_foot_upper_right_link_mix ; 
    yarp::sig::Matrix Jac_l_foot_lower_left_link_body  = Adjoint( T_R_l_foot_lower_left_link_index_w  ) * Jac_l_foot_lower_left_link_mix  ; 
    yarp::sig::Matrix Jac_l_foot_lower_right_link_body = Adjoint( T_R_l_foot_lower_right_link_index_w ) * Jac_l_foot_lower_right_link_mix ;

    yarp::sig::Matrix Jac_r_foot_upper_left_link_body  = Adjoint( T_R_r_foot_upper_left_link_index_w  ) * Jac_r_foot_upper_left_link_mix  ; 
    yarp::sig::Matrix Jac_r_foot_upper_right_link_body = Adjoint( T_R_r_foot_upper_right_link_index_w ) * Jac_r_foot_upper_right_link_mix ; 
    yarp::sig::Matrix Jac_r_foot_lower_left_link_body  = Adjoint( T_R_r_foot_lower_left_link_index_w  ) * Jac_r_foot_lower_left_link_mix  ; 
    yarp::sig::Matrix Jac_r_foot_lower_right_link_body = Adjoint( T_R_r_foot_lower_right_link_index_w ) * Jac_r_foot_lower_right_link_mix ;
    
    //yarp::sig::Matrix Jac_spa_temp = Adjoint( T_aw_imu ) * Jac_imu_body  ; 
    
    yarp::sig::Matrix Jac_aw_l_foot_upper_left_link_temp  = Adjoint(T_aw_w*T_w_l_foot_upper_left_link_index  ) * Jac_l_foot_upper_left_link_body ;
    yarp::sig::Matrix Jac_aw_l_foot_upper_right_link_temp = Adjoint(T_aw_w*T_w_l_foot_upper_right_link_index ) * Jac_l_foot_upper_right_link_body ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_left_link_temp  = Adjoint(T_aw_w*T_w_l_foot_lower_left_link_index  ) * Jac_l_foot_lower_left_link_body ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_right_link_temp = Adjoint(T_aw_w*T_w_l_foot_lower_right_link_index ) * Jac_l_foot_lower_right_link_body ;

    yarp::sig::Matrix Jac_aw_r_foot_upper_left_link_temp  = Adjoint(T_aw_w*T_w_r_foot_upper_left_link_index  ) * Jac_r_foot_upper_left_link_body ;
    yarp::sig::Matrix Jac_aw_r_foot_upper_right_link_temp = Adjoint(T_aw_w*T_w_r_foot_upper_right_link_index ) * Jac_r_foot_upper_right_link_body ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_left_link_temp  = Adjoint(T_aw_w*T_w_r_foot_lower_left_link_index  ) * Jac_r_foot_lower_left_link_body ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_right_link_temp = Adjoint(T_aw_w*T_w_r_foot_lower_right_link_index ) * Jac_r_foot_lower_right_link_body ;
    
    Jac_aw_l_foot_upper_left_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_l_foot_upper_right_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_l_foot_lower_left_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_l_foot_lower_right_link_temp.setSubmatrix(Eye_6,0,0) ;

    Jac_aw_r_foot_upper_left_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_r_foot_upper_right_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_r_foot_lower_left_link_temp.setSubmatrix(Eye_6,0,0) ;
    Jac_aw_r_foot_lower_right_link_temp.setSubmatrix(Eye_6,0,0) ;
    
    yarp::sig::Matrix Jac_aw_l_foot_upper_left_link  =  Jac_aw_l_foot_upper_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_l_foot_upper_right_link =  Jac_aw_l_foot_upper_right_link_temp ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_left_link  =  Jac_aw_l_foot_lower_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_right_link =  Jac_aw_l_foot_lower_right_link_temp ;

    yarp::sig::Matrix Jac_aw_r_foot_upper_left_link  =  Jac_aw_r_foot_upper_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_r_foot_upper_right_link =  Jac_aw_r_foot_upper_right_link_temp ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_left_link  =  Jac_aw_r_foot_lower_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_right_link =  Jac_aw_r_foot_lower_right_link_temp ;

    
   yarp::sig::Matrix Jac_aw_l_foot_upper_left_link_body   = Adjoint( iHomogeneous( T_aw_w*T_w_l_foot_upper_left_link_index ) ) * Jac_aw_l_foot_upper_left_link ;
   yarp::sig::Matrix Jac_aw_l_foot_upper_right_link_body  = Adjoint( iHomogeneous( T_aw_w*T_w_l_foot_upper_right_link_index ) ) * Jac_aw_l_foot_upper_right_link ;
   yarp::sig::Matrix Jac_aw_l_foot_lower_left_link_body   = Adjoint( iHomogeneous( T_aw_w*T_w_l_foot_lower_left_link_index ) ) * Jac_aw_l_foot_lower_left_link ;
   yarp::sig::Matrix Jac_aw_l_foot_lower_right_link_body  = Adjoint( iHomogeneous( T_aw_w*T_w_l_foot_lower_right_link_index ) ) * Jac_aw_l_foot_lower_right_link ;
   
   yarp::sig::Matrix Jac_aw_r_foot_upper_left_link_body   = Adjoint( iHomogeneous( T_aw_w*T_w_r_foot_upper_left_link_index ) ) * Jac_aw_r_foot_upper_left_link ;
   yarp::sig::Matrix Jac_aw_r_foot_upper_right_link_body  = Adjoint( iHomogeneous( T_aw_w*T_w_r_foot_upper_right_link_index ) ) * Jac_aw_r_foot_upper_right_link ;
   yarp::sig::Matrix Jac_aw_r_foot_lower_left_link_body   = Adjoint( iHomogeneous( T_aw_w*T_w_r_foot_lower_left_link_index ) ) * Jac_aw_r_foot_lower_left_link ;
   yarp::sig::Matrix Jac_aw_r_foot_lower_right_link_body  = Adjoint( iHomogeneous( T_aw_w*T_w_r_foot_lower_right_link_index ) ) * Jac_aw_r_foot_lower_right_link ;

   yarp::sig::Matrix B_select( 3 , 6 ) ;
   yarp::sig::Matrix Eye_3( 3 , 3 ) ;
   Eye_3.eye() ;
   B_select.setSubmatrix( Eye_3 , 0 , 0 ) ;
    
   yarp::sig::Matrix Jac_aw_l_foot_upper_left_sel   = B_select * Jac_aw_l_foot_upper_left_link_body  ; // Selected body Jacobians at the contact
   yarp::sig::Matrix Jac_aw_l_foot_upper_right_sel  = B_select * Jac_aw_l_foot_upper_right_link_body ;
   yarp::sig::Matrix Jac_aw_l_foot_lower_left_sel   = B_select * Jac_aw_l_foot_lower_left_link_body  ;
   yarp::sig::Matrix Jac_aw_l_foot_lower_right_sel  = B_select * Jac_aw_l_foot_lower_right_link_body ;
    
   yarp::sig::Matrix Jac_aw_r_foot_upper_left_sel   = B_select * Jac_aw_r_foot_upper_left_link_body  ; // Selected body Jacobians at the contact
   yarp::sig::Matrix Jac_aw_r_foot_upper_right_sel  = B_select * Jac_aw_r_foot_upper_right_link_body ;
   yarp::sig::Matrix Jac_aw_r_foot_lower_left_sel   = B_select * Jac_aw_r_foot_lower_left_link_body  ;
   yarp::sig::Matrix Jac_aw_r_foot_lower_right_sel  = B_select * Jac_aw_r_foot_lower_right_link_body ;    
    
    
    
    
   // Jac_aw_l_foot_upper_left_sel.rows() ; 
    yarp::sig::Matrix Jac_complete( 8*Jac_aw_l_foot_upper_left_sel.rows(), ( robot.getNumberOfJoints() + 6 )  ) ;
    
    Jac_complete.setSubmatrix( Jac_aw_l_foot_upper_left_sel  , 0 , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_l_foot_upper_right_sel , Jac_aw_l_foot_upper_left_sel.rows()    , 0 ) ;    
    Jac_complete.setSubmatrix( Jac_aw_l_foot_lower_left_sel  , 2*Jac_aw_l_foot_upper_left_sel.rows()  , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_l_foot_lower_right_sel , 3*Jac_aw_l_foot_upper_left_sel.rows()  , 0 ) ;
    
    Jac_complete.setSubmatrix( Jac_aw_r_foot_upper_left_sel  , 4*Jac_aw_l_foot_upper_left_sel.rows()   , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_r_foot_upper_right_sel , 5*Jac_aw_l_foot_upper_left_sel.rows()   , 0 ) ;    
    Jac_complete.setSubmatrix( Jac_aw_r_foot_lower_left_sel  , 6*Jac_aw_l_foot_upper_left_sel.rows()   , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_r_foot_lower_right_sel , 7*Jac_aw_l_foot_upper_left_sel.rows()   , 0 ) ;



    
    

   
   
   

    
    


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//     
 
/*     std::cout << " Jac_waist_mix " <<  std::endl << Jac_waist_mix.toString() << std::endl; 
     std::cout << " Jac_l_sole_mix " <<  std::endl << Jac_l_sole_mix.toString() << std::endl; 
     std::cout << " Jac_r_sole_mix " <<  std::endl << Jac_r_sole_mix.toString() << std::endl; 
     std::cout << " Jac_l_hand_upper_right_mix " <<  std::endl << Jac_l_hand_upper_right_mix.toString() << std::endl; 
     std::cout << " Jac_r_hand_upper_right_mix " <<  std::endl << Jac_r_hand_upper_right_mix.toString() << std::endl;   */

  /*yarp::sig::Vector temp   = z_imu_aw_matr.transposed()* x_imu_aw  ;
    yarp::sig::Vector temp_2 = z_imu_aw_matr.transposed()* y_imu_aw  ;    
    yarp::sig::Vector temp_3 = cross( x_imu_aw , y_imu_aw );    
    yarp::sig::Vector temp_4 = -1.0*cross( z_imu_aw , y_imu_aw );    
    
    yarp::sig::Matrix test = R_imu_aw.transposed()*R_imu_aw + R_imu_aw * R_imu_aw.transposed();
    
    std::cout << " IMU_sense_lin_acc " <<  std::endl << IMU_sense_lin_acc.toString() << std::endl; 

    std::cout << " x_imu_aw " <<  std::endl << x_imu_aw.toString() << std::endl; 
    std::cout << " y_imu_aw " <<  std::endl << y_imu_aw.toString() << std::endl; 
    std::cout << " z_imu_aw " <<  std::endl << z_imu_aw.toString() << std::endl; 

    std::cout << " temp " <<  std::endl << temp.toString() << std::endl; 
    
    std::cout << " temp_2 " <<  std::endl << temp_2.toString() << std::endl; 

    std::cout << " temp_3 " <<  std::endl << temp_3.toString() << std::endl; 

    std::cout << " temp_4 " <<  std::endl << temp_4.toString() << std::endl; 
   
    std::cout << " norm_x " <<  std::endl << norm(x_imu_aw)  << std::endl; 
    std::cout << " norm_y " <<  std::endl << norm(y_imu_aw)  << std::endl; 
    std::cout << " norm_z " <<  std::endl << norm(z_imu_aw)  << std::endl; 
     
    std::cout << " R_imu_aw " <<  std::endl << R_imu_aw.toString()  << std::endl; 
       
    std::cout << " test " <<   std::endl << test.toString() << std::endl; 
    
    std::cout << " d_imu_aw " <<  std::endl << d_imu_aw.toString()  << std::endl;    
    std::cout << " T_imu_aw " <<  std::endl << T_imu_aw.toString()  << std::endl; 
    std::cout << " T_aw_imu " <<  std::endl << T_aw_imu.toString()  << std::endl; 
    std::cout << " T_w_aw "   <<  std::endl << T_w_aw.toString()  << std::endl; 
    std::cout << " T_aw_w "   <<  std::endl << T_aw_w.toString()  << std::endl; */
    
   /*   std::cout << "  Jac_l_foot_upper_left_link_mix " <<  std::endl << Jac_l_foot_upper_left_link_mix.toString()  << std::endl; 
    std::cout << "  Jac_l_foot_upper_left_link_body " <<  std::endl << Jac_l_foot_upper_left_link_body.toString()  << std::endl; 
    std::cout << "  Jac_aw_l_foot_upper_left_link_temp " <<  std::endl << Jac_aw_l_foot_upper_left_link_temp.toString()  << std::endl; 
    std::cout << "  Jac_aw_l_foot_upper_left_link " <<  std::endl << Jac_aw_l_foot_upper_left_link.toString()  << std::endl;  
    std::cout << "  Jac_aw_l_foot_upper_left_link_body " <<  std::endl << Jac_aw_l_foot_upper_left_link_body.toString()  << std::endl;  
    std::cout << "  Jac_aw_l_foot_upper_left_sel " <<  std::endl << Jac_aw_l_foot_upper_left_sel.toString()  << std::endl;  
    std::cout << "  Jac_aw_l_foot_upper_right_sel " <<  std::endl << Jac_aw_l_foot_upper_right_sel.toString()  << std::endl;  
    std::cout << "  Jac_complete " <<  std::endl << Jac_complete.toString()  << std::endl;  */

    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//  
   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    // std::cout << "  ---------------------------------------- "  << std::endl ;     
    //---------------------------------------------------------------------------//
    //---------------------------------------------------------------------------//
    // Start Seting FLMM   
  
       
    int size_fc =   fc_l_contacts.length() +  fc_r_contacts.length() ; 
    int size_q = robot.getNumberOfJoints() ;
    
    

    // std::cout << " size_q = " <<  std::endl << size_q  << std::endl; 

    yarp::sig::Matrix Eye_fc(size_fc, size_fc) ;
    Eye_fc.eye() ;
    yarp::sig::Matrix Eye_q(size_q, size_q) ;
    Eye_q.eye() ;
    yarp::sig::Matrix Eye_tau = Eye_q ;
    
    //std::cout << " Eye_fc = " <<  std::endl << Eye_fc.toString()  << std::endl; 
    //std::cout << " Eye_q = " <<  std::endl << Eye_q.toString()  << std::endl; 
    yarp::sig::Matrix Zeros_fc_q(size_fc, size_q) ;
    yarp::sig::Matrix Zeros_q_fc(size_q, size_fc) ;
    yarp::sig::Matrix Zeros_q_q(size_q, size_q) ;
    yarp::sig::Matrix Zeros_6_q( 6 , size_q) ;
    yarp::sig::Matrix Zeros_q_6(size_q, 6 ) ;
    
    // std::cout << " Zeros_q_6 = " <<  std::endl << Zeros_q_6.toString()  << std::endl; 
    
    yarp::sig::Matrix Kc(size_fc, size_fc) ;
    Kc.eye() ;
    Kc = 1E7*Kc ;
    //std::cout << " Kc = " <<  std::endl << Kc.toString()  << std::endl; 
    yarp::sig::Matrix Kq = getKq() ;
    
    yarp::sig::Matrix Stance_c_tranposed =  Jac_complete.submatrix(0, Jac_complete.rows()-1 , 0, 5) ;
    yarp::sig::Matrix Stance_c( 6 , size_fc) ;
    Stance_c  =  Stance_c_tranposed.transposed()  ;
    
    yarp::sig::Matrix Jacob_c( size_fc , size_q) ;
    Jacob_c = Jac_complete.submatrix(0, Jac_complete.rows()-1 , 6, Jac_complete.cols()-1) ;
    
    /*std::cout << " Jac_complete " <<  std::endl << Jac_complete.toString() << std::endl; 
    //std::cout << " Stance_c_tranposed " <<  std::endl << Stance_c_tranposed.toString() << std::endl; 
    //std::cout << " Stance_c " <<  std::endl << Stance_c.toString() << std::endl; 
    std::cout << " Jacob_c " <<  std::endl << Jacob_c.toString() << std::endl; 

    std::cout << " Jac_complete.rows() " <<  std::endl << Jac_complete.rows() << std::endl; 
    std::cout << " Jac_complete.cols() " <<  std::endl << Jac_complete.cols() << std::endl; 
    std::cout << " Stance_c.rows() " <<  std::endl << Stance_c.rows() << std::endl; 
    std::cout << " Stance_c.cols() " <<  std::endl << Stance_c.cols() << std::endl; 
    
    std::cout << " Jacob_c.rows() " <<  std::endl << Jacob_c.rows() << std::endl; 
    std::cout << " Jacob_c.cols() " <<  std::endl << Jacob_c.cols() << std::endl; */
    
    

    yarp::sig::Matrix U_j( size_fc , size_q) ;
    yarp::sig::Matrix U_s( 6 , size_fc) ;  

    yarp::sig::Matrix Q_j( size_fc , size_q) ;
    yarp::sig::Matrix Q_s( 6 , size_fc) ;     
    
    //
    int r_FLMM = size_fc + 6 + 2*size_q;
    int c_FLMM = size_fc + 6 + 3*size_q;
    yarp::sig::Matrix FLMM(r_FLMM, c_FLMM) ;
    
    // Setting the first block-row of the FLMM
    FLMM.setSubmatrix( Eye_fc                           , 0 , 0                   ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc             ) ;
    FLMM.setSubmatrix( -1.0 * Kc*Stance_c.transposed()  , 0 , size_fc+size_q      ) ;
    FLMM.setSubmatrix( -1.0 * Kc*Jacob_c                , 0 , size_fc+size_q+6    ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc+2*size_q+6  ) ;

    // Setting the second block-row of the FLMM
    FLMM.setSubmatrix( -1.0*Jacob_c.transposed()    , size_fc , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau                      , size_fc , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_j                   , size_fc , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_j                   , size_fc , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_q_q                    , size_fc , size_fc+2*size_q+6 ) ;

    // Setting the third block-row of the FLMM
    FLMM.setSubmatrix( -1.0*Stance_c     , size_fc +size_q , 0                  ) ;
    FLMM.setSubmatrix( Zeros_6_q         , size_fc +size_q , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_s        , size_fc +size_q , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_s        , size_fc +size_q , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_6_q         , size_fc +size_q , size_fc+2*size_q+6 ) ;

    // Setting the fourth block-row of the FLMM
    FLMM.setSubmatrix( Zeros_q_fc  , size_fc +size_q +6  , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau     , size_fc +size_q +6  , size_fc            ) ;
    // std::cout << " Eye_tau[ 0 ][ 0 ]  = " <<  std::endl << Eye_tau[ 0 ][ 0 ]   << std::endl; 

    FLMM.setSubmatrix( Zeros_q_6   , size_fc +size_q +6  , size_fc+size_q     ) ;
    FLMM.setSubmatrix( Kq          , size_fc +size_q +6  , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( -1.0*Kq     , size_fc +size_q +6  , size_fc+2*size_q+6 ) ;

  /*  std::cout << "  ---------------------------------------- "  << std::endl ;     
 
    std::cout << " FLMM[ 0 ][ 0 ]  = " <<  std::endl << FLMM[ 0 ][ 0  ]   << std::endl; 
    std::cout << " FLMM[ 0 ][ size_fc ]  = " <<  std::endl << FLMM[ 0  ][ size_fc    ]   << std::endl; 
    std::cout << " FLMM[ 0 ][ size_fc + size_q ]  = " <<  std::endl << FLMM[ 0  ][ size_fc+size_q  ]   << std::endl; 
    std::cout << " FLMM[ 0 ][ size_fc + size_q + 6 ]  = " <<  std::endl << FLMM[ 0  ][ size_fc+size_q+6  ]   << std::endl; 
    std::cout << " FLMM[ 0 ][ size_fc + 2*size_q + 6 ]  = " <<  std::endl << FLMM[ 0  ][ size_fc + 2*size_q + 6 ]   << std::endl; 

    std::cout << "  ---------------------------------------- "  << std::endl ;     
     
    std::cout << " FLMM[ size_fc ][ 0 ]  = " <<  std::endl << FLMM[ size_fc  ][ 0  ]   << std::endl; 
    std::cout << " FLMM[ size_fc ][ size_fc ]  = " <<  std::endl << FLMM[ size_fc   ][ size_fc    ]   << std::endl; 
    std::cout << " FLMM[ size_fc ][ size_fc + size_q ]  = " <<  std::endl << FLMM[ size_fc   ][ size_fc+size_q  ]   << std::endl; 
    std::cout << " FLMM[ size_fc ][ size_fc + size_q + 6 ]  = " <<  std::endl << FLMM[ size_fc   ][ size_fc+size_q+6  ]   << std::endl; 
    std::cout << " FLMM[ size_fc ][ size_fc + 2*size_q + 6 ]  = " <<  std::endl << FLMM[ size_fc   ][ size_fc + 2*size_q + 6 ]   << std::endl; 

    std::cout << " FLMM[ size_fc+size_q+6 ][ 0 ]  = " <<  std::endl << FLMM[ size_fc+size_q+6 ][ 0  ]   << std::endl; 
    std::cout << " FLMM[ size_fc+size_q+6 ][ size_fc ]  = " <<  std::endl << FLMM[ size_fc+size_q+6  ][ size_fc    ]   << std::endl; 
    std::cout << " FLMM[ size_fc+size_q+6 ][ size_fc + size_q ]  = " <<  std::endl << FLMM[ size_fc+size_q+6  ][ size_fc+size_q  ]   << std::endl; 
    std::cout << " FLMM[ size_fc+size_q+6 ][ size_fc + size_q + 6 ]  = " <<  std::endl << FLMM[ size_fc+size_q+6  ][ size_fc+size_q+6  ]   << std::endl; 
    std::cout << " FLMM[ size_fc+size_q+6 ][ size_fc + 2*size_q + 6 ]  = " <<  std::endl << FLMM[ size_fc+size_q+6  ][ size_fc + 2*size_q + 6 ]   << std::endl;  */
    
  /*std::cout << " FLMM[ FLMM.rows() -1 ] [  FLMM.cols() -1 ]  = " <<  std::endl << FLMM[ FLMM.rows() -1  ][ FLMM.cols()-1 ]   << std::endl; 
    std::cout << " FLMM[ size_fc+ 2*size_q+6 -1 ][ size_fc + 3*size_q + 6 -1 ]  = " <<  std::endl << FLMM[ size_fc+2*size_q+6 -1][ size_fc+3*size_q+6-1]   << std::endl; */ 
       
    
    /*std::cout << " FLMM[ size_fc+ size_q+6 ][ size_fc + size_q + 6  ]  = " <<  std::endl << FLMM[ size_fc+ size_q+6 ][ size_fc+  size_q+6 ]   << std::endl; 
    std::cout << " Kq[ 0 ][ 0 ] = " <<  std::endl << Kq[ 0 ][ 0 ]  << std::endl;  */
    
    
    yarp::sig::Matrix Phi_star_i = FLMM.submatrix(0, FLMM.rows()-1, 0,   size_fc + 2*size_q + 6-1     )  ;
   
    // std::cout << " Phi_star_i.rows  = " <<  std::endl << Phi_star_i.rows()  << std::endl; 
    // std::cout << " Phi_star_i.cols  = " <<  std::endl << Phi_star_i.cols()  << std::endl; 

    yarp::sig::Matrix cFLMM =  yarp::math::luinv(Phi_star_i)*FLMM  ;
       
    yarp::sig::Matrix Phi_i = cFLMM.submatrix(0, cFLMM.rows()-1,     0    ,             size_fc + 2*size_q + 6-1     )  ;    
    yarp::sig::Matrix Phi_d = cFLMM.submatrix(0, cFLMM.rows()-1, size_fc + 2*size_q + 6 ,   cFLMM.cols()-1     )  ;

    yarp::sig::Matrix R_f = Phi_d.submatrix(0, size_fc-1 ,0  ,Phi_d.cols()-1 ) ;    
    
    
    //------------------------------------------------------------------------------//    
    //------------------------------------------------------------------------------//
    // Testing R_f

  //  std::cout << " Phi_i  = " <<  std::endl << Phi_i.toString()  << std::endl;    
  //  std::cout << " Phi_d  = " <<  std::endl << Phi_d.toString()  << std::endl;
  //  std::cout << " R_f  = " <<  std::endl << R_f.toString()  << std::endl;

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    //------------------------------------------------------------------------------//
    yarp::sig::Vector fc_actual( fc_l_contacts.length() + fc_r_contacts.length())  ;      
    
    fc_actual.setSubvector( 0 , fc_l_contacts ) ;
    fc_actual.setSubvector( fc_l_contacts.length() , fc_r_contacts ) ;
    
    // desired contact force definition
    yarp::sig::Vector fc_desired( fc_l_contacts.length() + fc_r_contacts.length())  ;

    int weight =  300 ; // [N]
    
    fc_desired[0] = 0  ;
    fc_desired[1] = 0  ;
    fc_desired[2] = 300/8 ;
    fc_desired[3] = 0  ;
    fc_desired[4] = 0  ;
    fc_desired[5] = 300/8  ;
    fc_desired[6] = 0 ;
    fc_desired[7] = 0 ;
    fc_desired[8] = 300/8  ;
    fc_desired[9] =  0 ;
    fc_desired[10] = 0 ;
    fc_desired[11] = 300/8  ;

    fc_desired[12] = 0  ;
    fc_desired[13] = 0  ;
    fc_desired[14] = 300/8 ;
    fc_desired[15] = 0  ;
    fc_desired[16] = 0  ;
    fc_desired[17] = 300/8  ;
    fc_desired[18] = 0 ;
    fc_desired[19] = 0 ;
    fc_desired[20] = 300/8  ;
    fc_desired[21] =  0 ;
    fc_desired[22] = 0 ;
    fc_desired[23] = 300/8  ;


/*  std::cout << " fc_l_contacts  = " <<  std::endl << fc_l_contacts.toString()  << std::endl; 
    std::cout << " fc_r_contacts  = " <<  std::endl << fc_r_contacts.toString()  << std::endl; 
    std::cout << " fc_actual  = " <<  std::endl << fc_actual.toString()  << std::endl; 
    
    std::cout << " fc_l_contacts_x  = " <<  std::endl << fc_l_contacts[0] +
							 fc_l_contacts[3] + 
							 fc_l_contacts[6] + 
							 fc_l_contacts[9]<< std::endl; 
    std::cout << " fc_r_contacts_x  = " <<  std::endl << fc_r_contacts[0] +
							 fc_r_contacts[3] + 
							 fc_r_contacts[6] + 
							 fc_r_contacts[9]<< std::endl;     
    
    std::cout << " fc_l_contacts_y  = " <<  std::endl << fc_l_contacts[1] +
							 fc_l_contacts[4] + 
							 fc_l_contacts[7] + 
							 fc_l_contacts[10]<< std::endl; 
    std::cout << " fc_r_contacts_y  = " <<  std::endl << fc_r_contacts[1] +
							 fc_r_contacts[4] + 
							 fc_r_contacts[7] + 
							 fc_r_contacts[10]<< std::endl;    

    std::cout << " fc_l_contacts_z  = " <<  std::endl << fc_l_contacts[2] +
							 fc_l_contacts[5] + 
							 fc_l_contacts[8] + 
							 fc_l_contacts[11]<< std::endl; 
    std::cout << " fc_r_contacts_z  = " <<  std::endl << fc_r_contacts[2] +
							 fc_r_contacts[5] + 
							 fc_r_contacts[8] + 
							 fc_r_contacts[11]<< std::endl;    	*/						 
							 

 /* std::cout << " ft_l_ankle[0]  = " <<  std::endl << ft_l_ankle[0] << std::endl; 
    std::cout << " ft_l_ankle[1]  = " <<  std::endl << ft_l_ankle[1] << std::endl; 
    std::cout << " ft_l_ankle[2]  = " <<  std::endl << ft_l_ankle[2] << std::endl; 
    std::cout << " ft_l_ankle[3]  = " <<  std::endl << ft_l_ankle[3] << std::endl; 
    std::cout << " ft_l_ankle[4]  = " <<  std::endl << ft_l_ankle[4] << std::endl; 
    std::cout << " ft_l_ankle[5]  = " <<  std::endl << ft_l_ankle[5] << std::endl; 

    std::cout << " ft_r_ankle[0]  = " <<  std::endl << ft_r_ankle[0] << std::endl; 
    std::cout << " ft_r_ankle[1]  = " <<  std::endl << ft_r_ankle[1] << std::endl; 
    std::cout << " ft_r_ankle[2]  = " <<  std::endl << ft_r_ankle[2] << std::endl; 
    std::cout << " ft_r_ankle[3]  = " <<  std::endl << ft_r_ankle[3] << std::endl; 
    std::cout << " ft_r_ankle[4]  = " <<  std::endl << ft_r_ankle[4] << std::endl; 
    std::cout << " ft_r_ankle[5]  = " <<  std::endl << ft_r_ankle[5] << std::endl; 

    std::cout << " R_x  = " <<  std::endl << ft_l_ankle[0] + ft_r_ankle[0] << std::endl; 
    std::cout << " R_y  = " <<  std::endl << ft_l_ankle[1] + ft_r_ankle[1] << std::endl;     
    std::cout << " R_z  = " <<  std::endl << ft_l_ankle[2] + ft_r_ankle[2] << std::endl;  */
    
  
 
 // Force applied by the robot on the environment
    
    fc_actual  = - 1.0*fc_actual ;
    fc_desired = - 1.0*fc_desired ;
 
    // std::cout << " fc_actual  = " <<  std::endl << fc_actual.toString() << std::endl; 
    // std::cout << " fc_desired  = " <<  std::endl << fc_desired.toString() << std::endl; 

    yarp::sig::Vector d_fc_desired = fc_desired - fc_actual ;
    
    yarp::sig::Vector d_q_motor_desired = -1.0* pinv( R_f , 1E-5 ) *d_fc_desired ; 

  //  std::cout << " d_fc_desired = " <<  std::endl << d_fc_desired.toString() << std::endl;    
   // std::cout << " d_q_motor_desired = " <<  std::endl << d_q_motor_desired.toString() << std::endl; 

    yarp::sig::Vector temp = -1.0* R_f* d_q_motor_desired ; 
  //  std::cout << " - R_f* d_q_motor_desired = " <<  std::endl << temp.toString() << std::endl; 
    
 
    
    // yarp::sig::Vector dq_ref_computed = yarp::math::pinv(R_f)*fc_desired  ;
    
    
    // + dq_projected for postural task (to avoid devergences in time)
    
   
     //---------------------------------------------------------------------------//
    //---------------------------------------------------------------------------//   

    /*std::cout << " Phi_d  = " <<  std::endl << Phi_d.toString()  << std::endl;
    std::cout << " Phi_d.rows  = " <<  std::endl << Phi_d.rows()  << std::endl; 
    std::cout << " Phi_d.cols  = " <<  std::endl << Phi_d.cols()  << std::endl; 
    std::cout << " R_f = " <<  std::endl << R_f.toString()  << std::endl; 
    std::cout << " R_f.rows  = " <<  std::endl << R_f.rows()  << std::endl; 
    std::cout << " R_f.cols  = " <<  std::endl << R_f.cols()  << std::endl;   */
    
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
    q_ref_ToMove_right_arm[0] += .00 ;  
    
    robot.fromRobotToIdyn( q_ref_ToMove_right_arm ,
                           q_ref_ToMove_left_arm  ,
                           q_ref_ToMove_torso  ,
                           q_ref_ToMove_right_leg ,
                           q_ref_ToMove_left_leg  ,
                           q_ref_ToMove );    

   // d_q_motor_desired
//     std::cout << " d_q_motor_desired.length() " << d_q_motor_desired.length() << std::endl;
//     std::cout << " q_ref_ToMove.length() " << q_ref_ToMove.length() << std::endl;

     
     q_ref_ToMove = q_ref_ToMove ; // to stabilize... q_ref_ToMove  + d_q_motor_desired ;
//     std::cout << " q_ref_ToMove_left_arm  " << q_ref_ToMove_left_arm.toString() << std::endl;
//     std::cout << " q_ref_ToMove_right_arm  " << q_ref_ToMove_right_arm.toString() << std::endl;
//     std::cout << " q_ref_ToMove_torso  " << q_ref_ToMove_torso.toString() << std::endl;

//     std::cout << " left_arm_configuration  " <<  left_arm_configuration.toString()  << std::endl;
    
     
     robot.move(q_ref_ToMove);  // q_ref_ToMove
 
     // robot.left_arm.move(q_ref_ToMove_left_arm);
    
    
    
    
    
 
    
 

    //---------------------------------------------------------------------------//
    
    //---------------------------------------------------------------------------// 
    // Default Stuff form the Generic Module -- Command Interface
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


