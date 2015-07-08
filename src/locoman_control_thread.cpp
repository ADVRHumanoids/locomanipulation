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

using namespace yarp::math;

locoman_control_thread::locoman_control_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph ):
    control_thread( module_prefix, rf, ph ),
    command_interface(module_prefix), 
    loop_counter(0) ,
    FC_size(24)  ,
    WINDOW_size(50) ,  // 15
    FC_DES(FC_size, 0.0) , 
    FC_DES_LEFT_sensor(6, 0.0) ,
    FC_DES_RIGHT_sensor(6,0.0),
    FC_FILTERED(FC_size),
    FC_WINDOW(FC_size, WINDOW_size ) 
{
    left_arm_joints = robot.left_arm.getNumberOfJoints();
    omega = 0.1;
    phi = 10;
    tick = 0;
    max_vel= 0.1; //  0.35;  // 
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
    //
    //  Simulator-To-Robot Switch
    flag_robot = 0 ;
    flag_simulator = 1-flag_robot ;
    //
    //-----------------------------------------------
    // Both equals
  /*  FC_DES[2] = - mg/8 ;
    FC_DES[5] = - mg/8  ;
    FC_DES[8] = - mg/8  ;
    FC_DES[11] = - mg/8  ;
    FC_DES[14] = - mg/8 ;
    FC_DES[17] = - mg/8  ;
    FC_DES[20] = - mg/8  ;
    FC_DES[23] = - mg/8  ;  */
    //
    double part = -6.0/10.0 ;
    // On the left foot
    FC_DES[2] = - (mg/8.0 + part*(mg/8.0) ) ;
    FC_DES[5] = - (mg/8.0 + part*(mg/8.0) )   ;
    FC_DES[8] = - (mg/8.0 + part*(mg/8.0) )   ;
    FC_DES[11] = - (mg/8.0 + part*(mg/8.0) )   ;
    // On the right foot
    FC_DES[14] = - (mg/8.0 - part*(mg/8.0) )  ;
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
    
    //robot.setPositionDirectMode();    
    
    robot.setPositionMode() ;
    robot.setReferenceSpeed(0.3) ;
    //-----------------------------------------------------------

    
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
    q_left_leg_des  = left_leg_configuration ;
    q_right_leg_des = right_leg_configuration ;
    
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
    
    d_q_des = (q_des - q_motor_0)*10 ;  // /10  /100
    
    robot.move(q_des) ; 
//     
    q_motor_act = senseMotorPosition() ;
    double err_0 = norm(q_motor_act- q_des) +0.1 ;
    double err_1 = norm(q_motor_act- q_des)  ;
  
    while ( norm(q_motor_act- q_des)>0.01 && fabs(err_0-err_1)>0.000001 ) //( ( (norm(q_motor_act- q_des)>0.01)  )) // &&  ( abs(err_0-err_1)>0.000001 )) )
    {
    err_0 = err_1 ;
    usleep(100*1000) ;
    q_motor_act = senseMotorPosition() ;
    err_1 = norm(q_motor_act- q_des)  ;
    std::cout << " err_1  =  " <<  err_1 << std::endl; 
     continue ; 
    }
    std::cout << " final error =  " <<  norm(q_motor_act- q_des) << std::endl; 
    std::cout << " final fabs(err_0-err_1) =  " <<  fabs(err_0-err_1) << std::endl; 

    //std::cout << " left_arm  =  " <<  left_arm_configuration.toString() << std::endl; 

    usleep(3000*1000) ; // usleep(milliseconds*1000)
    // robot.left_arm.move(q_ref_ToMove_left_arm);
    return true;
    //
    //----------------------------------------------
    //
    

    
    
    
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
   yarp::sig::Vector q_motor( robot.getNumberOfJoints()  )  ; 
   if(flag_robot)
   {
    yarp::sig::Vector q_link = robot.sensePosition() ;
    return q_motor ; 
   } 
    yarp::sig::Vector q_link = robot.sensePosition() ;
    yarp::sig::Vector tau    = robot.senseTorque() ;
    //     
    yarp::sig::Matrix Kq_matrix = getKq() ;
    yarp::sig::Matrix Cq_matrix = yarp::math::luinv(Kq_matrix) ;
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
    yarp::sig::Matrix Eye_3( 3 , 3 )  ;
    Eye_3.eye() ;
    // 
    B_select.setSubmatrix( Eye_3 , 0 , 0 ) ;
    if(flag_robot)
    {
      yarp::sig::Matrix T_w_ankle = model.iDyn3_model.getPosition(sens_index) ;
      yarp::sig::Vector d_sens_ankle (3, 0.0) ;
      d_sens_ankle[2] = 0.07 ; //   // measured 7 cm
      yarp::sig::Matrix T_sensor_ankle = Homogeneous(Eye_3, d_sens_ankle) ;
      //
      yarp::sig::Matrix T_ankle_w = iHomogeneous(T_w_ankle) ;
      yarp::sig::Matrix T_sensor_w = T_sensor_ankle * T_ankle_w ;
      T_w_sensor =  iHomogeneous(T_sensor_w) ;
      //  
    } 
    else if(flag_simulator)
    {
      T_w_sensor = model.iDyn3_model.getPosition(sens_index) ;     
    }
    else
    {
      std::cout << "error: flags are not coherent" << std::endl ;
    }
    //
    T_w_c1  = model.iDyn3_model.getPosition(c1_index)  ;
    T_w_c2  = model.iDyn3_model.getPosition(c2_index)  ;    
    T_w_c3  = model.iDyn3_model.getPosition(c3_index)  ;    
    T_w_c4  = model.iDyn3_model.getPosition(c4_index)  ;    
     
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


yarp::sig::Matrix locoman_control_thread::xi_hat(const yarp::sig::Vector xi)
{
    yarp::sig::Vector vel = xi.subVector(0,2)   ;
    yarp::sig::Vector omega  = xi.subVector(3,5)  ;
    yarp::sig::Matrix vel_matr(3,1)  ;
    vel_matr[0][0] = vel[0] ;
    vel_matr[1][0] = vel[1] ;
    vel_matr[2][0] = vel[2] ;
    yarp::sig::Matrix xi_hat_matr(4,4) ;
    yarp::sig::Matrix omega_hat = crossProductMatrix(omega) ;
    xi_hat_matr.setSubmatrix( omega_hat, 0, 0 )   ;
    xi_hat_matr.setSubmatrix( vel_matr, 0, 3 )   ;
    return xi_hat_matr ;
}


yarp::sig::Matrix locoman_control_thread::exp_omega_theta(const yarp::sig::Vector omega, const double theta)
{
   yarp::sig::Matrix Eye_3(3,3) ;
   Eye_3.eye() ;
   yarp::sig::Matrix R(3,3) ;
   // std::cout << "qui_1_1 " << std::endl ;
   yarp::sig::Matrix omega_hat = crossProductMatrix(omega) ;
   R = Eye_3 + (omega_hat/norm(omega) )*sin(norm(omega)*theta) + omega_hat*omega_hat/(norm(omega)*norm(omega))*(1 - cos(norm(omega)*theta)) ; 
   //std::cout << "qui_3_1 " << std::endl ;
   return R ;
}


yarp::sig::Matrix locoman_control_thread::twistexp(const yarp::sig::Vector xi, const double theta)
{
   // std::cout << "qui_1 " << std::endl ;

   yarp::sig::Matrix Eye_3(3,3) ;
   Eye_3.eye() ;
   yarp::sig::Vector vel = xi.subVector(0,2)   ;
   yarp::sig::Vector omega  = xi.subVector(3,5)  ;
   yarp::sig::Matrix omega_matr(3,1)  ;
   omega_matr[0][0] = omega[0] ;
   omega_matr[1][0] = omega[1] ;
   omega_matr[2][0] = omega[2] ;
   yarp::sig::Matrix R(3,3) ;
   yarp::sig::Vector t(3) ;
   if (norm(omega)< 1E-8)
   {
     R = Eye_3 ;
     t = vel*theta ;
   }  
   else {
   //std::cout << "qui_2 " << std::endl ;
   R = exp_omega_theta(omega, theta) ;
   // std::cout << "qui_3 " << std::endl ;
   t = (Eye_3 - 1.0*R )*(crossProductMatrix(omega)*vel ) + omega_matr*omega_matr.transposed()*vel *theta  ;
   }
   /*yarp::sig::Vector temp_1 =  (Eye_3 - 1.0*R )*(crossProductMatrix(omega)*vel )  ;
   yarp::sig::Vector temp_2 =  omega_matr*omega_matr.transposed()*vel *theta ;
   yarp::sig::Matrix temp_3 = omega_matr*omega_matr.transposed() ;
   yarp::sig::Vector temp_4 = vel *theta  ;
   std::cout << "temp_1 = " << std::endl << temp_1.toString()  << std::endl ;
   std::cout << "temp_2 = " << std::endl << temp_2.toString()  << std::endl ;
   std::cout << "temp_3 = " << std::endl << temp_3.toString()  << std::endl ;
   std::cout << "temp_4 = " << std::endl << temp_4.toString()  << std::endl ; */
   // std::cout << "qui_4 " << std::endl ;
   yarp::sig::Matrix T(4,4) ;
   T = Homogeneous(R, t) ;
   return T ;
}


yarp::sig::Matrix locoman_control_thread::ad_lie(const yarp::sig::Matrix Xi)
{    
   if(!(Xi.rows() == 6)) std::cout << "ERROR DIMENSIONS OF Xi" << std::endl; 
   if(!(Xi.cols() == 1)) std::cout << "ERROR DIMENSIONS OF Xi" << std::endl; 
   yarp::sig::Vector v(3) ;
   yarp::sig::Vector omega(3) ;
   v[0] = Xi[0][0];
   v[1] = Xi[1][0];
   v[2] = Xi[2][0];
   omega[0] = Xi[3][0];
   omega[1] = Xi[4][0];
   omega[2] = Xi[5][0];
   yarp::sig::Matrix v_skew = crossProductMatrix(v) ;
   yarp::sig::Matrix omega_skew = crossProductMatrix(omega) ;
   yarp::sig::Matrix Zero_3_3(3,3) ;
   Zero_3_3.zero();
   yarp::sig::Matrix AD_LIE(6,6) ;
   AD_LIE.setSubmatrix(omega_skew, 0, 0 ) ;
   AD_LIE.setSubmatrix(v_skew,     0, 3 ) ;
   AD_LIE.setSubmatrix(Zero_3_3 ,  3, 0 ) ;
   AD_LIE.setSubmatrix(omega_skew ,3, 3 ) ;
   return AD_LIE ;
}


yarp::sig::Matrix locoman_control_thread::ad_lie(const yarp::sig::Vector Xi)
{
   if(!(Xi.length() == 6)) std::cout << "ERROR DIMENSIONS OF Xi" << std::endl; 
 //  if(!(Xi.cols() == 1)) std::cout << "ERROR DIMENSIONS OF Xi" << std::endl; 
   yarp::sig::Vector v(3) ;
   yarp::sig::Vector omega(3) ;
   v[0] = Xi[0] ;
   v[1] = Xi[1] ;
   v[2] = Xi[2] ;
   omega[0] = Xi[3] ;
   omega[1] = Xi[4] ;
   omega[2] = Xi[5] ;
   yarp::sig::Matrix v_skew = crossProductMatrix(v) ;
   yarp::sig::Matrix omega_skew = crossProductMatrix(omega) ;
   yarp::sig::Matrix Zero_3_3(3,3) ;
   Zero_3_3.zero();
   yarp::sig::Matrix AD_LIE(6,6) ;
   AD_LIE.setSubmatrix(omega_skew, 0, 0 ) ;
   AD_LIE.setSubmatrix(v_skew,     0, 3 ) ;
   AD_LIE.setSubmatrix(Zero_3_3 ,  3, 0 ) ;
   AD_LIE.setSubmatrix(omega_skew ,3, 3 ) ;
   return AD_LIE ;
}



yarp::sig::Matrix locoman_control_thread::D_Jacob_spa_i(const yarp::sig::Matrix J_s, const int i)
{
     
   int r_J = J_s.rows() ;
   int c_J = J_s.cols() ;
   yarp::sig::Matrix D_J_i(r_J,c_J) ;
   yarp::sig::Vector zero_6(6, 0.0) ;
   D_J_i.zero();
   if(!(r_J==6)) 
   {
     std::cout << "ERROR DIMENSIONS OF Xi" << std::endl ;
   }  
   if(i>=c_J )
   {
    return D_J_i;
   }
  else{
   for ( int k = 0  ; k<i ; k++ )  
   {
     D_J_i.setCol(k, zero_6) ;
  };
   for(int k = i  ; k<c_J ; k++  )
   {    
   yarp::sig::Matrix temp = ad_lie(J_s.submatrix( 0, 5, i-1, i-1 ))*J_s.submatrix(0,5, k , k )  ;
   yarp::sig::Vector temp_2(6) ;
   temp_2[0] =  temp[0][0] ;
   temp_2[1] =  temp[1][0] ;
   temp_2[2] =  temp[2][0] ;
   temp_2[3] =  temp[3][0] ;
   temp_2[4] =  temp[4][0] ;
   temp_2[5] =  temp[5][0] ;
   D_J_i.setCol(k,  temp_2) ;
  };
  return D_J_i;
  }
}

yarp::sig::Matrix locoman_control_thread::Q_ci(const yarp::sig::Matrix J_spa_i, const yarp::sig::Matrix T_a_ci, const yarp::sig::Vector f_ci)
{
  yarp::sig::Matrix Q_c_i(J_spa_i.cols(), J_spa_i.cols());
  yarp::sig::Vector Q_ci_col_i( J_spa_i.cols() )  ; 
  yarp::sig::Matrix B(6,3) ;
  B[0][0] = 1 ;
  B[1][1] = 1 ;
  B[2][2] = 1 ;
    for ( int i = 0  ; i<(J_spa_i.cols()) ; i++ )  // i<(J_waist_l_c1_spa_0.cols()-1)
     {
       Q_ci_col_i =  ( D_Jacob_spa_i(J_spa_i, (i+1)) ).transposed()*
                     ( Adjoint( iHomogeneous(T_a_ci))).transposed()*B *f_ci  +
                     J_spa_i.transposed()* 
                     ( Adjoint( iHomogeneous(T_a_ci) )*
                      ad_lie( -1.0*J_spa_i.getCol(i))).transposed()*  B *f_ci;
       Q_c_i.setCol(i, Q_ci_col_i ) ;       
     }   
  return Q_c_i ;
}



yarp::sig::Matrix locoman_control_thread::RoundMatrix(const yarp::sig::Matrix M, const int k)
{
  yarp::sig::Matrix Round_M = M ;
  for( int i=0 ; i<Round_M.cols(); i++) {
      for( int j=0 ; j<Round_M.rows(); j++){
      Round_M[j][i]  = Round_M[j][i]  * std::pow(10,k) ;
      Round_M[j][i]  = floor( Round_M[j][i] +0.5) ;
      Round_M[j][i]  = Round_M[j][i]* std::pow(10,-k) ;
      } ;
  } ;
return Round_M ;
}




yarp::sig::Matrix locoman_control_thread::FLMM_ext(const yarp::sig::Matrix J_c, const yarp::sig::Matrix S_c, const yarp::sig::Matrix Q_j, const yarp::sig::Matrix Q_s, const yarp::sig::Matrix U_j, const yarp::sig::Matrix U_s, const yarp::sig::Matrix K_c, const yarp::sig::Matrix K_q)
{
    int size_fc  = J_c.rows();
    int size_q  = J_c.cols() ;
    int r_FLMM = size_fc + 6 + 2*size_q;
    int c_FLMM = size_fc + 6 + 3*size_q;
    yarp::sig::Matrix FLMM(r_FLMM, c_FLMM) ;    
    yarp::sig::Matrix Eye_fc(size_fc, size_fc) ;
    yarp::sig::Matrix Eye_q(size_q, size_q) ;
    Eye_fc.eye() ;
    Eye_q.eye() ;
    yarp::sig::Matrix Eye_tau = Eye_q ;
    yarp::sig::Matrix Zeros_fc_q(size_fc, size_q) ;
    yarp::sig::Matrix Zeros_q_fc(size_q, size_fc) ;
    yarp::sig::Matrix Zeros_q_q(size_q, size_q) ;
    yarp::sig::Matrix Zeros_6_q( 6 , size_q) ;
    yarp::sig::Matrix Zeros_q_6(size_q, 6 ) ;
    Zeros_fc_q.zero();
    Zeros_q_fc.zero();
    Zeros_q_q.zero();
    Zeros_6_q.zero();    
    Zeros_q_6.zero();  
    // Setting the first block-row of the FLMM
    FLMM.setSubmatrix( Eye_fc                           , 0 , 0                   ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc             ) ;
    FLMM.setSubmatrix( -1.0 * K_c*S_c.transposed()       , 0 , size_fc+size_q      ) ;
    FLMM.setSubmatrix( -1.0 * K_c*J_c                    , 0 , size_fc+size_q+6    ) ;
    FLMM.setSubmatrix( Zeros_fc_q                       , 0 , size_fc+2*size_q+6  ) ;
    // Setting the second block-row of the FLMM
    FLMM.setSubmatrix( -1.0*J_c.transposed()        , size_fc , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau                      , size_fc , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_j                   , size_fc , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_j                   , size_fc , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_q_q                    , size_fc , size_fc+2*size_q+6 ) ;
    // Setting the third block-row of the FLMM
    FLMM.setSubmatrix( -1.0*S_c      , size_fc +size_q , 0                  ) ;
    FLMM.setSubmatrix( Zeros_6_q     , size_fc +size_q , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * U_s    , size_fc +size_q , size_fc+size_q     ) ;
    FLMM.setSubmatrix( -1.0 * Q_s    , size_fc +size_q , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( Zeros_6_q     , size_fc +size_q , size_fc+2*size_q+6 ) ;

    // Setting the fourth block-row of the FLMM
    FLMM.setSubmatrix( Zeros_q_fc  , size_fc +size_q +6  , 0                  ) ;
    FLMM.setSubmatrix( Eye_tau     , size_fc +size_q +6  , size_fc            ) ;
    FLMM.setSubmatrix( Zeros_q_6   , size_fc +size_q +6  , size_fc+size_q     ) ;
    FLMM.setSubmatrix( K_q          , size_fc +size_q +6  , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( -1.0*K_q     , size_fc +size_q +6  , size_fc+2*size_q+6 ) ;
    return FLMM ;
}

yarp::sig::Matrix locoman_control_thread::Rf_ext(const yarp::sig::Matrix J_c, const yarp::sig::Matrix S_c, const yarp::sig::Matrix Q_j, const yarp::sig::Matrix Q_s, const yarp::sig::Matrix U_j, const yarp::sig::Matrix U_s, const yarp::sig::Matrix K_c, const yarp::sig::Matrix K_q)
{
   yarp::sig::Matrix Q_j_1 = -1.0*Q_j - 1.0* J_c.transposed()*K_c*J_c  ;
   yarp::sig::Matrix U_j_1 = -1.0*U_j -1.0*J_c.transposed() *K_c*S_c.transposed() ;    
   yarp::sig::Matrix Q_s_1 =  -1.0*Q_s-1.0*S_c*K_c*J_c  ;
   yarp::sig::Matrix U_s_1 = -1.0*U_s-1.0*S_c*K_c*S_c.transposed() ;    
   yarp::sig::Matrix L = yarp::math::luinv(U_s_1)* Q_s_1 ;
   yarp::sig::Matrix M = Q_j_1-U_j_1*L ;    
   yarp::sig::Matrix H = K_q-M ;
   yarp::sig::Matrix F = -1.0*yarp::math::luinv(H)*K_q ;    
   yarp::sig::Matrix E = -1.0*K_c* S_c.transposed()* L *F ;
   yarp::sig::Matrix R_f_1 = E+K_c*J_c*F  ;
   return R_f_1 ;
}


yarp::sig::Matrix locoman_control_thread::FLMM_redu(const yarp::sig::Matrix J_c, const yarp::sig::Matrix S_c, const yarp::sig::Matrix Q_s, const yarp::sig::Matrix U_s, const yarp::sig::Matrix K_c)
{
    int size_fc  = J_c.rows();
    int size_q  = J_c.cols() ;
    int r_FLMM = size_fc + 6  ;
    int c_FLMM = size_fc + 6 + size_q;
    yarp::sig::Matrix FLMM(r_FLMM, c_FLMM) ;    
    yarp::sig::Matrix Eye_fc(size_fc, size_fc) ;
    Eye_fc.eye() ;
    // Setting the first block-row of the FLMM
    FLMM.setSubmatrix( Eye_fc                      , 0 , 0                   ) ;
    FLMM.setSubmatrix( -1.0 * K_c*S_c.transposed() , 0 , size_fc             ) ;
    FLMM.setSubmatrix( -1.0 * K_c*J_c              , 0 , size_fc+6      ) ;
    // Setting the second block-row of the FLMM
    FLMM.setSubmatrix( -1.0 * S_c    , size_fc , 0                  ) ;
    FLMM.setSubmatrix( -1.0 * U_s    , size_fc , size_fc            ) ;
    FLMM.setSubmatrix( -1.0 * Q_s    , size_fc , size_fc+6     ) ;    
    return FLMM ;
}


yarp::sig::Matrix locoman_control_thread::Rf_redu(const yarp::sig::Matrix J_c, const yarp::sig::Matrix S_c, const yarp::sig::Matrix Q_s, const yarp::sig::Matrix U_s, const yarp::sig::Matrix K_c)
{
 //    
   yarp::sig::Matrix Q_s_1 =  Q_s +  S_c*K_c*J_c  ;
   yarp::sig::Matrix U_s_1 =  U_s + S_c*K_c*S_c.transposed() ;    
   yarp::sig::Matrix L = yarp::math::luinv(U_s_1)* Q_s_1 ;
   yarp::sig::Matrix R_f_1 = - 1.0*K_c*J_c + K_c*S_c.transposed()* L  ;
   return R_f_1 ;
}

yarp::sig::Matrix locoman_control_thread::Ru_redu(const yarp::sig::Matrix J_c, const yarp::sig::Matrix S_c, const yarp::sig::Matrix Q_s, const yarp::sig::Matrix U_s, const yarp::sig::Matrix K_c)
{
   yarp::sig::Matrix Q_s_1 =  -1.0*Q_s-1.0*S_c*K_c*J_c  ;
   yarp::sig::Matrix U_s_1 = -1.0*U_s-1.0*S_c*K_c*S_c.transposed() ;    
   yarp::sig::Matrix L = yarp::math::luinv(U_s_1)* Q_s_1 ;
   return L ;
}


yarp::sig::Matrix locoman_control_thread::Pinv_trunc_SVD(const yarp::sig::Matrix A, const double k)
{
  int r_A = A.rows() ;
  int c_A = A.cols() ;
  int c_U = c_A;
  if(c_U>r_A ) c_U = r_A ;
  yarp::sig::Matrix U( r_A, c_U )  ;
  yarp::sig::Matrix V( c_A, c_U )  ;
  yarp::sig::Vector S( c_U ) ;  
  
  yarp::sig::Matrix S_1( c_U , c_U ) ;  
  S_1.zero();
  yarp::math::SVD(A, U, S, V );
  
  double k_norm = S(0)*k ;
        for(int i = 0 ;  i < c_U ; i++)
    {
      if( S(i)<k_norm) 
      {
	S_1(i,i)=0.0; 
      }
      else
      {
	S_1(i,i) = 1/S(i) ;
      }
    }  
 /*   std::cout << " k = " << std::endl << k  << std::endl   ;
    std::cout << " S = " << std::endl << S.toString()  << std::endl   ;    
    std::cout << " S_1 = " << std::endl << S_1.toString()  << std::endl   ; */
    yarp::sig::Matrix pinv_A = V * S_1 *U.transposed() ;
  return pinv_A ;
}


yarp::sig::Matrix locoman_control_thread::Pinv_Regularized(const yarp::sig::Matrix A, const double k)
{
    yarp::sig::Matrix At_A = A.transposed()*A ;
    yarp::sig::Matrix Eye_temp = eye(At_A.rows(), At_A.cols() ) ;
    yarp::sig::Matrix pinv_A = yarp::math::luinv(At_A + k*Eye_temp ) *A.transposed() ;
    return pinv_A ;
}

yarp::sig::Vector locoman_control_thread::x_Pinv_Iter(const yarp::sig::Matrix A, const yarp::sig::Vector b,  double n)
{
  int r_A = A.rows() ;
  int c_A = A.cols() ;
  if(n>r_A ) n = r_A ;
  if(n>c_A ) n = c_A ;
  yarp::sig::Matrix At_A = A.transposed()*A ;
  yarp::sig::Matrix Eye_temp = eye(At_A.rows(), At_A.cols() ) ;
  yarp::sig::Vector At_b = A.transposed()*b ;
  yarp::sig::Vector x_k(c_A , 0.0) ;
  yarp::sig::Vector x_k1(c_A , 0.0) ;

      for(int i = 0 ;  i < n ; i++)
    {
      x_k1 = x_k + A.transposed()*b - A.transposed()*A*x_k ;
     //  x_k1 = (Eye_temp - At_A) * x_k + At_b ;
       x_k = x_k1 ;
    } ;
    return x_k1 ;
}

yarp::sig::Matrix locoman_control_thread::orth_SVD(const yarp::sig::Matrix A, const double k)
{
  int r_A = A.rows() ;
  int c_A = A.cols() ;
  int c_U = c_A;
  if(c_U>r_A ) c_U = r_A ;
  yarp::sig::Matrix U( r_A, c_U )  ;
  yarp::sig::Matrix V( c_A, c_U )  ;
  yarp::sig::Vector S( c_U ) ;  
  yarp::sig::Matrix S_1( c_U , c_U ) ;  
  S_1.zero();
  yarp::math::SVD(A, U, S, V );
 /*        std::cout << " U = " << std::endl <<U.toString()  << std::endl   ;
       std::cout << " S = " << std::endl <<S.toString()  << std::endl   ;
       std::cout << " V = " << std::endl <<V.toString()  << std::endl   ;  */
  int cont = 0;
  double k_norm = S(0)*k ;
        for(int i = 0 ;  i < c_U ; i++)
    {
      if( S(i)<k_norm) 
      {
	S_1(i,i)=0.0; 
      }
      else
      {//span_A.resize();
       //bool a = span_A.setCol(i, U.getCol(i) ) ;//S_1(i,i) = 1/S(i) ;
       cont += 1 ;
      // std::cout << " span_A = " << std::endl <<span_A.toString()  << std::endl   ;
    //   std::cout << " U.getCol(i)  = " << std::endl <<  (U.getCol(i)).toString()  << std::endl   ;
     //  std::cout << " a = " << std::endl << a << std::endl   ;
      }
    }  
      yarp::sig::Matrix span_A = U.submatrix(0, U.rows()-1, 0, (cont-1) );
 //   std::cout << " span_A = " << std::endl <<span_A.toString()  << std::endl   ;

 /* std::cout << " k = " << std::endl << k  << std::endl   ;
    std::cout << " S = " << std::endl << S.toString()  << std::endl   ;    
    std::cout << " S_1 = " << std::endl << S_1.toString()  << std::endl   ; */
  //  yarp::sig::Matrix span_A ;  
  return span_A ;
}

yarp::sig::Matrix locoman_control_thread::null_SVD(const yarp::sig::Matrix A, const double k)
{
  int r_A = A.rows() ;
  int c_A = A.cols() ;
  int c_U = c_A;
  if(c_U>r_A ) c_U = r_A ;
  yarp::sig::Matrix U( r_A, c_U )  ;
  yarp::sig::Matrix V( c_A, c_U )  ;
  yarp::sig::Vector S( c_U ) ;  
  yarp::sig::Matrix S_1( c_U , c_U ) ;  
  S_1.zero();
  yarp::math::SVD(A, U, S, V );
  double k_norm = S(0)*k ;
        for(int i = 0 ;  i < c_U ; i++)
    {
      if( S(i)<k_norm) 
      {
	S_1(i,i)=0.0; 
      }
      else
      {
	S_1(i,i) = S(i) ;
      }
    }  
  yarp::sig::Matrix A_filter = U * S_1 * V.transposed() ;
  yarp::sig::Matrix null_project_A = nullspaceProjection(A_filter) ;
  yarp::sig::Matrix null_A = orth_SVD(null_project_A,k) ;
  return null_A ;
}


yarp::sig::Matrix locoman_control_thread::filter_SVD(const yarp::sig::Matrix A, const double k)
{
  int r_A = A.rows() ;
  int c_A = A.cols() ;
  int c_U = c_A;
  if(c_U>r_A ) c_U = r_A ;
  yarp::sig::Matrix U( r_A, c_U )  ;
  yarp::sig::Matrix V( c_A, c_U )  ;
  yarp::sig::Vector S( c_U ) ;  
  yarp::sig::Matrix S_1( c_U , c_U ) ;  
  S_1.zero();
  yarp::math::SVD(A, U, S, V );
  double k_norm = S(0)*k ;
        for(int i = 0 ;  i < c_U ; i++)
    {
      if( S(i)<k_norm) 
      {
	S_1(i,i)=0.0; 
      }
      else
      {
	S_1(i,i) = S(i) ;
      }
    }  
  yarp::sig::Matrix A_filter = U * S_1 * V.transposed() ;
  return A_filter ;
}

double locoman_control_thread::sigma_frict(const yarp::sig::Vector fc, const double mu)
{
  yarp::sig::Vector fc_aux = fc ;
  if(fc_aux(2)<0){
     fc_aux= -1.0*fc; 
  } ;
  yarp::sig::Vector normal;
  normal(0) = 0 ;
  normal(1) = 0 ;
  normal(2) = 1 ;
  //
  double alpha_frict = 1/(sqrt( 1+ pow(mu,2))) ;
  double sigma_frict = alpha_frict*norm(fc_aux) - yarp::math::dot(fc_aux,normal) ;
  return sigma_frict ;
}


double locoman_control_thread::sigma_max(const yarp::sig::Vector fc, const double f_max)
{
  yarp::sig::Vector fc_aux = fc ;
  if(fc_aux(2)<0){
     fc_aux= -1.0*fc; 
  } ;
  yarp::sig::Vector normal;
  normal(0) = 0 ;
  normal(1) = 0 ;
  normal(2) = 1 ;
  //
  double sigma_max = - f_max +   yarp::math::dot(fc_aux,normal) ;
  return sigma_max ;
}

double locoman_control_thread::sigma_min(const yarp::sig::Vector fc, const double f_min)
{
  yarp::sig::Vector fc_aux = fc ;
  if(fc_aux(2)<0){
     fc_aux= -1.0*fc; 
  } ;
  yarp::sig::Vector normal;
  normal(0) = 0 ;
  normal(1) = 0 ;
  normal(2) = 1 ;
  //
  double sigma_min =  f_min - yarp::math::dot(fc_aux,normal) ;
  return sigma_min;
}

double locoman_control_thread::V_ij(const double sigma, const double toll)
{
  double V_ij ;
if(sigma<toll){
  V_ij = 1/(2*pow(sigma,2)) ;
}
else{
  double a = 3/(2*pow(toll,4)) ;
  double b = 4/(  pow(toll,3)) ;
  double c = 3/(  pow(toll,2)) ;
  V_ij = a*pow(sigma,2) + b*sigma + c ;
}
  return V_ij ;
}




void locoman_control_thread::run()
{     
    robot.setReferenceSpeed(max_vel) ;
    yarp::sig::Vector q_current = robot.sensePosition() ;
    robot.idynutils.updateiDyn3Model( q_current, true ); //update model first
    
    yarp::sig::Vector tau_current = robot.senseTorque() ;
        
    yarp::sig::Vector q_motor_side(robot.getNumberOfJoints() ) ;		    
    q_motor_side = senseMotorPosition() ;

    //--------------------------------------------------------------------------//
    // Getting Contact Forces
    RobotUtils::ftPtrMap fts = robot.getftSensors();
    
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
    }
  
  //--------------------------------------------------------------------//
    //Getting Sensor Measures

    yarp::sig::Vector ft_r_ankle(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;     
    yarp::sig::Vector ft_l_ankle(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 

    //  NO DELETE
//---------------------------------------------------------------------------------------------------------//  
    //  NO DELETE
    //      writing data on a file
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
    
    // 
    int l_ankle_index = model.iDyn3_model.getLinkIndex("l_ankle") ; // sensors are placed in _ankle in the model
    int l_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_upper_left_link");
    int l_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_upper_right_link");
    int l_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("l_foot_lower_left_link");
    int l_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("l_foot_lower_right_link");

    int r_ankle_index = model.iDyn3_model.getLinkIndex("r_ankle") ;
    int r_foot_upper_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_upper_left_link");
    int r_foot_upper_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_upper_right_link");
    int r_foot_lower_left_link_index   = model.iDyn3_model.getLinkIndex("r_foot_lower_left_link");
    int r_foot_lower_right_link_index  = model.iDyn3_model.getLinkIndex("r_foot_lower_right_link");

//-------------------------------------------------------------------------------------------------------------// 


    int l_c1_index = l_foot_upper_left_link_index  ;
    int l_c2_index = l_foot_upper_right_link_index ;
    int l_c3_index = l_foot_lower_left_link_index  ;
    int l_c4_index = l_foot_lower_right_link_index ;

    int r_c1_index = r_foot_upper_left_link_index  ;
    int r_c2_index = r_foot_upper_right_link_index ;
    int r_c3_index = r_foot_lower_left_link_index  ;
    int r_c4_index = r_foot_lower_right_link_index ;
    
    yarp::sig::Vector zero_3(3, 0.0) ;
    
    yarp::sig::Matrix Eye_6(6,6) ;
    Eye_6.eye() ;

    yarp::sig::Matrix Eye_3(3,3) ;
    Eye_3.eye() ;

    yarp::sig::Matrix B( 6 , 3 ) ;
    B.setSubmatrix( Eye_3 , 0 , 0 ) ;
    
    int size_q = robot.getNumberOfJoints() ;
    int size_u = 6 ;
    int size_fc = 24 ;
    
    yarp::sig::Matrix Kq = getKq() ;
   yarp::sig::Matrix Kc(size_fc, size_fc) ;
    Kc.eye() ;
    Kc =  1E6*Kc ; //   ;// 1E6*Kc ;    1E8*Kc ;  
    
   // desired contact force definition
    yarp::sig::Vector fc_des_to_world( size_fc, 0.0)  ;

  /*  fc_des_to_world[2] = - mg/8 ;
    fc_des_to_world[5] = - mg/8  ;
    fc_des_to_world[8] = - mg/8  ;
    fc_des_to_world[11] = - mg/8  ;
    fc_des_to_world[14] = - mg/8 ;
    fc_des_to_world[17] = - mg/8  ;
    fc_des_to_world[20] = - mg/8  ;
    fc_des_to_world[23] = - mg/8  ; */
    
    fc_des_to_world = FC_DES ;
    
//-------------------------------------------------------------------------------------------------------------    
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
    
    yarp::sig::Matrix T_l_ankle_w_0 = iHomogeneous(T_w_l_ankle_0) ;
    yarp::sig::Matrix T_l_c1_w_0    = iHomogeneous(T_w_l_c1_0) ;    
    yarp::sig::Matrix T_l_c2_w_0    = iHomogeneous(T_w_l_c2_0) ;  
    yarp::sig::Matrix T_l_c3_w_0    = iHomogeneous(T_w_l_c3_0) ;
    yarp::sig::Matrix T_l_c4_w_0    = iHomogeneous(T_w_l_c4_0) ;    
    
    yarp::sig::Matrix T_r_ankle_w_0 = iHomogeneous(T_w_r_ankle_0) ;
    yarp::sig::Matrix T_r_c1_w_0    = iHomogeneous(T_w_r_c1_0) ;    
    yarp::sig::Matrix T_r_c2_w_0    = iHomogeneous(T_w_r_c2_0) ;  
    yarp::sig::Matrix T_r_c3_w_0    = iHomogeneous(T_w_r_c3_0) ;
    yarp::sig::Matrix T_r_c4_w_0    = iHomogeneous(T_w_r_c4_0) ;    
    
    yarp::sig::Matrix map_l_fcToSens =   fConToSens( l_ankle_index, 
						      l_c1_index, 
					              l_c2_index,						      
						      l_c3_index, 
						      l_c4_index ) ;
						      
     yarp::sig::Matrix map_r_fcToSens =   fConToSens( r_ankle_index, 
					              r_c1_index, 
						      r_c2_index,
						      r_c3_index, 
						      r_c4_index ) ;	
						      
    yarp::sig::Vector fc_l_c_to_robot =  yarp::math::pinv( map_l_fcToSens, 1E-6)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_c_to_robot =  yarp::math::pinv( map_r_fcToSens, 1E-6)  *  ft_r_ankle     ;
    yarp::sig::Vector fc_l_c_to_world =  - 1.0 * fc_l_c_to_robot     ;
    yarp::sig::Vector fc_r_c_to_world =  - 1.0 * fc_r_c_to_robot     ;
    
    yarp::sig::Vector fc_to_world_0(size_fc) ;
    fc_to_world_0.setSubvector(0, fc_l_c_to_world ) ;
    fc_to_world_0.setSubvector(fc_l_c_to_world.length(), fc_r_c_to_world ) ;    
    
    //------------------------------------------------------------------------
        //
    FC_DES_LEFT_sensor = map_l_fcToSens * FC_DES.subVector(0, 11)  ;
    //
    FC_DES_RIGHT_sensor = map_l_fcToSens* FC_DES.subVector(12,23)  ;
    //
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
    //
    yarp::sig:: Vector FC_FILTERED_RIGHT_sensor = map_l_fcToSens* FC_FILTERED.subVector(12,23) ;
     

   std::cout << " loop_counter = " <<  loop_counter << std::endl; 
   std::cout << " counter_window = " <<  counter_window << std::endl; 
 /*  std::cout << " FC_WINDOW[0][counter_window+ 1] = " <<  FC_WINDOW[0][(counter_window+ 1)%WINDOW_size]  << std::endl;  */
   std::cout << " FC_WINDOW = " <<  std::endl << (FC_WINDOW.submatrix(0,FC_WINDOW.rows()-1,(FC_WINDOW.cols()-5),(FC_WINDOW.cols()-1) )).toString() << std::endl;
   std::cout << " FC_DES = " <<  std::endl << FC_DES.toString() << std::endl; 
   std::cout << " FC_DES_LEFT_sensor = " <<  std::endl << FC_DES_LEFT_sensor.toString() << std::endl; 
   std::cout << " FC_DES_RIGHT_sensor = " <<  std::endl << FC_DES_RIGHT_sensor.toString() << std::endl; 
   std::cout << " FC_FILTERED_LEFT_sensor  =  "  << std::endl << FC_FILTERED_LEFT_sensor.toString() << std::endl  ; 
   std::cout << " FC_FILTERED_RIGHT_sensor  =  "  << std::endl << FC_FILTERED_RIGHT_sensor.toString() << std::endl  ;     
 
   std::cout << " FC_FILTERED = " <<  std::endl << FC_FILTERED.toString() << std::endl; 
   
   
   std::cout << " norm(FC_DES_sens - FC_sens)  =  "  << std::endl <<   
   norm(FC_DES_LEFT_sensor- FC_FILTERED_LEFT_sensor) + norm(FC_DES_RIGHT_sensor- FC_FILTERED_RIGHT_sensor) << std::endl  ; 
   
   loop_counter++ ;

   
    // End of Fc Filtering
    //-----------------------------------------------------------------------------------------
    yarp::sig::Vector fc_l_c1_filt = FC_FILTERED.subVector(0,2)  ;  //
    yarp::sig::Vector fc_l_c2_filt = FC_FILTERED.subVector(3,5)  ;
    yarp::sig::Vector fc_l_c3_filt = FC_FILTERED.subVector(6,8)  ;
    yarp::sig::Vector fc_l_c4_filt = FC_FILTERED.subVector(9,11)  ;

    yarp::sig::Vector fc_r_c1_filt = FC_FILTERED.subVector(12,14)  ; 
    yarp::sig::Vector fc_r_c2_filt = FC_FILTERED.subVector(15,17)  ; 
    yarp::sig::Vector fc_r_c3_filt = FC_FILTERED.subVector(18,20)  ; 
    yarp::sig::Vector fc_r_c4_filt = FC_FILTERED.subVector(21,23)  ; 
    
    
/*   std::cout << " fc_l_c_to_world = " <<  std::endl << fc_l_c_to_world.toString() << std::endl; 
   std::cout << " fc_r_c_to_world = " <<  std::endl << fc_r_c_to_world.toString() << std::endl; 
   std::cout << " fc_to_world_0 = " <<  std::endl << fc_to_world_0.toString() << std::endl; 
   std::cout << " fc_des_to_world = " <<  std::endl << fc_des_to_world.toString() << std::endl;   */

    yarp::sig::Vector fc_l_c1 = fc_l_c_to_world.subVector(0,2)  ;  //
    yarp::sig::Vector fc_l_c2 = fc_l_c_to_world.subVector(3,5)  ;
    yarp::sig::Vector fc_l_c3 = fc_l_c_to_world.subVector(6,8)  ;
    yarp::sig::Vector fc_l_c4 = fc_l_c_to_world.subVector(9,11)  ;

    yarp::sig::Vector fc_r_c1 = fc_r_c_to_world.subVector(0,2)  ; 
    yarp::sig::Vector fc_r_c2 = fc_r_c_to_world.subVector(3,5)  ; 
    yarp::sig::Vector fc_r_c3 = fc_r_c_to_world.subVector(6,8)  ; 
    yarp::sig::Vector fc_r_c4 = fc_r_c_to_world.subVector(9,11)  ; 
    
    int imu_link_index = model.iDyn3_model.getLinkIndex("imu_link") ; 
    yarp::sig::Matrix T_w_imu_0 = model.iDyn3_model.getPosition( imu_link_index) ;    
    yarp::sig::Matrix T_imu_w_0 = iHomogeneous(T_w_imu_0) ; 
    RobotUtils::IMUPtr IMU_ptr = robot.getIMU()  ;
    yarp::sig::Vector IMU_sense = IMU_ptr->sense(); ;
    yarp::sig::Vector IMU_sense_lin_acc(3) ; 
        
    IMU_sense_lin_acc[0] = IMU_sense[3] ;    
    IMU_sense_lin_acc[1] = IMU_sense[4] ;    
    IMU_sense_lin_acc[2] = IMU_sense[5] ;    
    
    double norm_imu = norm(IMU_sense_lin_acc)     ;
    yarp::sig::Vector z_imu_aw =  IMU_sense_lin_acc/norm_imu ;
    double norm_z = norm(z_imu_aw)  ;
   
    yarp::sig::Matrix z_imu_aw_matr(3,1);
    z_imu_aw_matr[0][0] = z_imu_aw[0] ;
    z_imu_aw_matr[1][0] = z_imu_aw[1] ;
    z_imu_aw_matr[2][0] = z_imu_aw[2] ;   

    int waist_index = model.iDyn3_model.getLinkIndex("Waist") ;
    
    yarp::sig::Matrix T_w_waist_0 = model.iDyn3_model.getPosition( waist_index )  ;   
    yarp::sig::Matrix T_waist_w_0 = iHomogeneous(T_w_waist_0) ;
    yarp::sig::Matrix T_imu_waist_0 = T_imu_w_0* T_w_waist_0  ;
    
    yarp::sig::Vector x_imu_waist(3) ;
    x_imu_waist[0] =  T_imu_waist_0[0][0];
    x_imu_waist[1] =  T_imu_waist_0[1][0];
    x_imu_waist[2] =  T_imu_waist_0[2][0];

    yarp::sig::Matrix Null_z_tr =  nullspaceProjection(z_imu_aw_matr.transposed()) ;
    yarp::sig::Vector x_imu_aw = Null_z_tr*x_imu_waist  ;

    yarp::sig::Vector y_imu_aw(3) ;
    yarp::sig::Matrix R_imu_aw_0(3,3) ;    
    
    if (norm(x_imu_aw)>0.01)
    {
    x_imu_aw = x_imu_aw/norm(x_imu_aw) ;   
    R_imu_aw_0[1][0] = x_imu_aw[1] ;
    R_imu_aw_0[2][0] = x_imu_aw[2] ;

    R_imu_aw_0[0][1] = y_imu_aw[0] ;
    R_imu_aw_0[1][1] = y_imu_aw[1] ;
    R_imu_aw_0[2][1] = y_imu_aw[2] ;

    R_imu_aw_0[0][2] = z_imu_aw[0] ;
    R_imu_aw_0[1][2] = z_imu_aw[1] ;
    R_imu_aw_0[2][2] = z_imu_aw[2] ; 
    }
    else   {   
    x_imu_aw[0] = Null_z_tr[0][0] ; 
    x_imu_aw[1] = Null_z_tr[1][0] ; 
    x_imu_aw[2] = Null_z_tr[2][0] ;        
    x_imu_aw = x_imu_aw/norm(x_imu_aw) ;     
    }  
    y_imu_aw = cross(z_imu_aw, x_imu_aw  );   
    
    R_imu_aw_0[0][0] = x_imu_aw[0] ;
    R_imu_aw_0[1][0] = x_imu_aw[1] ;
    R_imu_aw_0[2][0] = x_imu_aw[2] ;

    R_imu_aw_0[0][1] = y_imu_aw[0] ;
    R_imu_aw_0[1][1] = y_imu_aw[1] ;
    R_imu_aw_0[2][1] = y_imu_aw[2] ;

    R_imu_aw_0[0][2] = z_imu_aw[0] ;
    R_imu_aw_0[1][2] = z_imu_aw[1] ;
    R_imu_aw_0[2][2] = z_imu_aw[2] ;
    
    yarp::sig::Matrix T_imu_aw_0 = Homogeneous( R_imu_aw_0, zero_3 ) ;
    yarp::sig::Matrix T_aw_imu_0 = iHomogeneous(T_imu_aw_0) ;    

    yarp::sig::Matrix T_w_aw_0 = T_w_imu_0 * T_imu_aw_0 ;
    yarp::sig::Matrix T_aw_w_0 = iHomogeneous(T_w_aw_0) ;    
//----------------------------------------------
    
    yarp::sig::Matrix T_waist_l_c1_0 = T_waist_w_0 * T_w_l_c1_0 ;  // the waist is the floating base
    yarp::sig::Matrix T_waist_l_c2_0 = T_waist_w_0 * T_w_l_c2_0 ;
    yarp::sig::Matrix T_waist_l_c3_0 = T_waist_w_0 * T_w_l_c3_0 ;
    yarp::sig::Matrix T_waist_l_c4_0 = T_waist_w_0 * T_w_l_c4_0 ;
    
    
    
        yarp::sig::Matrix test_matrix_c1_c2 = iHomogeneous( T_w_l_c1_0) *T_w_l_c2_0 ;
       std::cout << " test_matrix_c1_c2 = " <<  std::endl << test_matrix_c1_c2.toString() << std::endl;

    
    
    
    
    
    
    
    
    
    
    

    yarp::sig::Matrix T_waist_r_c1_0 = T_waist_w_0 * T_w_r_c1_0 ;
    yarp::sig::Matrix T_waist_r_c2_0 = T_waist_w_0 * T_w_r_c2_0 ;
    yarp::sig::Matrix T_waist_r_c3_0 = T_waist_w_0 * T_w_r_c3_0 ;
    yarp::sig::Matrix T_waist_r_c4_0 = T_waist_w_0 * T_w_r_c4_0 ;

    yarp::sig::Matrix T_b_l_c1_0 = T_waist_l_c1_0 ;  // the waist is the floating base {Waist} = {B}
    yarp::sig::Matrix T_b_l_c2_0 = T_waist_l_c2_0 ; 
    yarp::sig::Matrix T_b_l_c3_0 = T_waist_l_c3_0 ; 
    yarp::sig::Matrix T_b_l_c4_0 = T_waist_l_c4_0 ; 

    yarp::sig::Matrix T_b_r_c1_0 = T_waist_r_c1_0 ; 
    yarp::sig::Matrix T_b_r_c2_0 = T_waist_r_c2_0 ; 
    yarp::sig::Matrix T_b_r_c3_0 = T_waist_r_c3_0 ; 
    yarp::sig::Matrix T_b_r_c4_0 = T_waist_r_c4_0 ; 
 
    //---------------------------------------------------------------------------------------
    yarp::sig::Matrix J_l_c1_mix_0(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_l_c2_mix_0( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_l_c3_mix_0(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_l_c4_mix_0( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;

    yarp::sig::Matrix J_r_c1_mix_0(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_r_c2_mix_0( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_r_c3_mix_0(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix J_r_c4_mix_0( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;
    
    model.iDyn3_model.getJacobian( l_c1_index, J_l_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_c2_index, J_l_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_c3_index, J_l_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( l_c4_index, J_l_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian

    model.iDyn3_model.getJacobian( r_c1_index, J_r_c1_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_c2_index, J_r_c2_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_c3_index, J_r_c3_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
    model.iDyn3_model.getJacobian( r_c4_index, J_r_c4_mix_0, false  ) ; //false= mixed version jacobian //true= body jacobian
   
    yarp::sig::Matrix J_l_c1_body_0 = Adjoint( Homogeneous(getRot(T_l_c1_w_0), zero_3 ))* J_l_c1_mix_0 ;
    yarp::sig::Matrix J_l_c2_body_0 = Adjoint( Homogeneous(getRot(T_l_c2_w_0), zero_3 ))* J_l_c2_mix_0 ;
    yarp::sig::Matrix J_l_c3_body_0 = Adjoint( Homogeneous(getRot(T_l_c3_w_0), zero_3 ))* J_l_c3_mix_0 ;
    yarp::sig::Matrix J_l_c4_body_0 = Adjoint( Homogeneous(getRot(T_l_c4_w_0), zero_3 ))* J_l_c4_mix_0 ;

    yarp::sig::Matrix J_r_c1_body_0 = Adjoint( Homogeneous(getRot(T_r_c1_w_0), zero_3 ))* J_r_c1_mix_0 ;
    yarp::sig::Matrix J_r_c2_body_0 = Adjoint( Homogeneous(getRot(T_r_c2_w_0), zero_3 ))* J_r_c2_mix_0 ;
    yarp::sig::Matrix J_r_c3_body_0 = Adjoint( Homogeneous(getRot(T_r_c3_w_0), zero_3 ))* J_r_c3_mix_0 ;
    yarp::sig::Matrix J_r_c4_body_0 = Adjoint( Homogeneous(getRot(T_r_c4_w_0), zero_3 ))* J_r_c4_mix_0 ;

    yarp::sig::Matrix J_waist_l_c1_spa_0 = Adjoint( T_waist_l_c1_0 )* J_l_c1_body_0 ;
    yarp::sig::Matrix J_waist_l_c2_spa_0 = Adjoint( T_waist_l_c2_0 )* J_l_c2_body_0 ;
    yarp::sig::Matrix J_waist_l_c3_spa_0 = Adjoint( T_waist_l_c3_0 )* J_l_c3_body_0 ;
    yarp::sig::Matrix J_waist_l_c4_spa_0 = Adjoint( T_waist_l_c4_0 )* J_l_c4_body_0 ;

    yarp::sig::Matrix J_waist_r_c1_spa_0 = Adjoint( T_waist_r_c1_0 )* J_r_c1_body_0 ;
    yarp::sig::Matrix J_waist_r_c2_spa_0 = Adjoint( T_waist_r_c2_0 )* J_r_c2_body_0 ;
    yarp::sig::Matrix J_waist_r_c3_spa_0 = Adjoint( T_waist_r_c3_0 )* J_r_c3_body_0 ;
    yarp::sig::Matrix J_waist_r_c4_spa_0 = Adjoint( T_waist_r_c4_0 )* J_r_c4_body_0 ;
        
    J_waist_l_c1_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_l_c2_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_l_c3_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_l_c4_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;

    J_waist_r_c1_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_r_c2_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_r_c3_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
    J_waist_r_c4_spa_0.setSubmatrix(Eye_6, 0 , 0 )  ;
 
//------------------------------------------------------------------------------------------------------------
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

    yarp::sig::Matrix J_c = Complete_Jac.submatrix(0,  Complete_Jac.rows()-1 , 6, Complete_Jac.cols()-1 ) ;
    yarp::sig::Matrix S_trasp = Complete_Jac.submatrix(0,  Complete_Jac.rows()-1 , 0,5 ) ;
    yarp::sig::Matrix S_c = S_trasp.transposed() ;
    
//----------------------------------------------------------------------------------------------------------------
//   
    yarp::sig::Vector d_q_motor_deriv(size_q) ;
    yarp::sig::Vector d_q_motor_no_deriv(size_q) ;
    yarp::sig::Vector fc_teor(size_fc) ;
    yarp::sig::Vector d_fc_des_to_world(size_fc)  ;
   
    yarp::sig::Matrix Q_l_c1(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_l_c2(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_l_c3(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_l_c4(size_q+6, size_q+6 ) ;
    
    yarp::sig::Matrix Q_r_c1(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_r_c2(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_r_c3(size_q+6, size_q+6 ) ;
    yarp::sig::Matrix Q_r_c4(size_q+6, size_q+6 ) ;
    
    yarp::sig::Matrix U_j( size_q , 6 ) ;
    yarp::sig::Matrix U_s( 6 , 6) ;  
    yarp::sig::Matrix Q_j( size_q , size_q) ;
    yarp::sig::Matrix Q_s( 6 , size_q) ;   

  //--------------------------------------------------------------------------------------
  // Without Derivative terms
    
  // Kc.eye();
  //  Kc = 1E7*Kc ;
    U_j.zero();
    U_s.zero();
    Q_j.zero();
    Q_s.zero();

   yarp::sig::Matrix R_f_no_deriv = Rf_redu( J_c,  S_c,  Q_s,  U_s, Kc) ;// 
   yarp::sig::Matrix R_u_no_deriv = Rf_redu( J_c, S_c , Q_s,  U_s, Kc) ;
   d_fc_des_to_world = fc_des_to_world - FC_FILTERED ; // -fc_to_world_0 ;
   d_q_motor_no_deriv = -1.0* pinv( R_f_no_deriv , 1E-6 ) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_motor_no_deriv_1 = -1.0*  Pinv_trunc_SVD(R_f_no_deriv , 1E-1) * d_fc_des_to_world;
   yarp::sig::Vector d_q_motor_no_deriv_2 = -1.0*  Pinv_trunc_SVD(R_f_no_deriv , 1E-2) * d_fc_des_to_world;

   
   //---------------------------------------------------------------------------------------------------------
   
   
   yarp::sig::Matrix F_z(d_fc_des_to_world.length(), d_fc_des_to_world.length() )  ;
   F_z.zero(); 
   F_z[2][2] = 1 ;
   F_z[5][5] = 1 ;
   F_z[8][8] = 1 ;
   F_z[11][11] = 1 ;
   F_z[14][14] = 1 ;
   F_z[17][17] = 1 ;
   F_z[20][20] = 1 ;
   F_z[23][23] = 1 ; 
   
   yarp::sig::Vector d_q_motor_no_deriv_2_1 = -1.0*  Pinv_trunc_SVD(F_z*R_f_no_deriv , 1E-1) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_motor_no_deriv_2_2 = -1.0*  Pinv_trunc_SVD(F_z*R_f_no_deriv , 1E-2) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_motor_no_deriv_2_3 = -1.0*  Pinv_trunc_SVD(F_z*R_f_no_deriv , 1E-3) * (F_z* d_fc_des_to_world) ;

//   std::cout << " d_q_motor_no_deriv_2_1 = " <<  std::endl << d_q_motor_no_deriv_2_1.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_2_2 = " <<  std::endl << d_q_motor_no_deriv_2_2.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_2_3 = " <<  std::endl << d_q_motor_no_deriv_2_3.toString() << std::endl;

   //-----------------------------------------------------------------------------------------------------
 
   fc_teor   = fc_to_world_0 - 1.0*R_f_no_deriv *d_q_motor_no_deriv ;
   
   
   yarp::sig::Vector d_q_motor_no_deri_no_u_2 =   nullspaceProjection(R_u_no_deriv)* d_q_motor_no_deriv_2;
   
   
  
 /*  std::cout << " ----------------------------------------------------- "   << std::endl;    
   std::cout << " Without derivative terms "   << std::endl; 
 //  std::cout << " fc_teor = " <<  std::endl << fc_teor.toString() << std::endl;   
   std::cout << " d_q_motor_no_deriv = " <<  std::endl << d_q_motor_no_deriv.toString() << std::endl; 
   std::cout << " d_q_motor_no_deriv_1 = " <<  std::endl << d_q_motor_no_deriv_1.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_2 = " <<  std::endl << d_q_motor_no_deriv_2.toString() << std::endl;
   std::cout << " d_q_motor_no_deri_no_u_2 = " <<  std::endl << d_q_motor_no_deri_no_u_2.toString() << std::endl;  */

   /*  std::cout << " d_q_motor_no_deriv_3 = " <<  std::endl << d_q_motor_no_deriv_3.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_4 = " <<  std::endl << d_q_motor_no_deriv_4.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_5 = " <<  std::endl << d_q_motor_no_deriv_5.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_6 = " <<  std::endl << d_q_motor_no_deriv_6.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_7 = " <<  std::endl << d_q_motor_no_deriv_7.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_8 = " <<  std::endl << d_q_motor_no_deriv_8.toString() << std::endl;
   std::cout << " d_q_motor_no_deriv_9 = " <<  std::endl << d_q_motor_no_deriv_9.toString() << std::endl;*/
   
  // std::cout << " norm(d_q_motor_no_deriv_1) = " <<  std::endl << norm(d_q_motor_no_deriv_1 )<< std::endl;
 //  std::cout << " norm(d_q_motor_no_deri_no_u_2) = " <<  std::endl << norm(d_q_motor_no_deri_no_u_2 )<< std::endl;

   /*  std::cout << " norm(d_q_motor_no_deriv_3) = " <<  std::endl << norm(d_q_motor_no_deriv_3 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_4) = " <<  std::endl << norm(d_q_motor_no_deriv_4 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_5) = " <<  std::endl << norm(d_q_motor_no_deriv_5 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_6) = " <<  std::endl << norm(d_q_motor_no_deriv_6 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_7) = " <<  std::endl << norm(d_q_motor_no_deriv_7 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_8) = " <<  std::endl << norm(d_q_motor_no_deriv_8 )<< std::endl;
   std::cout << " norm(d_q_motor_no_deriv_9) = " <<  std::endl << norm(d_q_motor_no_deriv_9 )<< std::endl;*/

 //  std::cout << " norm(d_q_motor_no_deriv) = " <<  std::endl << norm(d_q_motor_no_deriv )<< std::endl;
   std::cout << " norm( d_fc_des_to_world ) = " <<  std::endl << norm( d_fc_des_to_world ) << std::endl; 
   std::cout << " norm( F_z*d_fc_des_to_world ) = " <<  std::endl << norm(F_z* d_fc_des_to_world ) << std::endl; 
   
   std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   std::cout << " norm(d_q_motor_no_deriv_2_3) = " <<  std::endl << norm(d_q_motor_no_deriv_2 )<< std::endl;

//----------------------------------------------------------------------------------------------------------------
    // Derivatives
   
     // Introducing derivative terms
    Q_l_c1 = Q_ci(J_waist_l_c1_spa_0, T_waist_l_c1_0, fc_l_c1 ) ;
    Q_l_c2 = Q_ci(J_waist_l_c2_spa_0, T_waist_l_c2_0, fc_l_c2 ) ; // (size_q+ 6, size_q + 6) ;
    Q_l_c3 = Q_ci(J_waist_l_c3_spa_0, T_waist_l_c3_0, fc_l_c3 ) ; //(size_q+ 6, size_q + 6) ; 
    Q_l_c4 = Q_ci(J_waist_l_c4_spa_0, T_waist_l_c4_0, fc_l_c4 ) ; //(size_q+ 6, size_q + 6) ;

    Q_r_c1 = Q_ci(J_waist_r_c1_spa_0, T_waist_r_c1_0, fc_r_c1 ) ; //(size_q+ 6, size_q + 6) ;
    Q_r_c2 = Q_ci(J_waist_r_c2_spa_0, T_waist_r_c2_0, fc_r_c2 ) ; //(size_q+ 6, size_q + 6) ;
    Q_r_c3 = Q_ci(J_waist_r_c3_spa_0, T_waist_r_c3_0, fc_r_c3 ) ; //(size_q+ 6, size_q + 6) ; 
    Q_r_c4 = Q_ci(J_waist_r_c4_spa_0, T_waist_r_c4_0, fc_r_c4 ) ; //(size_q+ 6, size_q + 6) ;
    
    yarp::sig::Matrix Q_l_tot = Q_l_c1 + Q_l_c2 + Q_l_c3 + Q_l_c4;
    yarp::sig::Matrix Q_r_tot = Q_r_c1 + Q_r_c2 + Q_r_c3 + Q_r_c4;
    yarp::sig::Matrix Q_c =  Q_l_tot + Q_r_tot ;  

   yarp::sig::Matrix U_s_der = Q_c.submatrix(0,5 , 0, 5) ;
   yarp::sig::Matrix U_j_der = U_j ;
   U_j_der.zero(); //Q_c.submatrix(6, (Q_c.rows()-1) , 0, 5) ;
     
   yarp::sig::Matrix Q_s_der = Q_c.submatrix(0,5,  6, (Q_c.cols()-1)  ) ;
   yarp::sig::Matrix Q_j_der = Q_c.submatrix( 6 , (Q_c.rows()-1)  ,  6, (Q_c.cols()-1)  ) ;

 //  yarp::sig::Matrix Rf_ext_der = Rf_ext( J_c , S_c ,Q_j_der, Q_s_der,  U_j_der,  U_s_der,  Kc, Kq)  ;
   yarp::sig::Matrix Rf_redu_der = Rf_redu( J_c,  S_c,  Q_s_der,  U_s_der, Kc) ;
 //  yarp::sig::Vector d_q_trunc_1 = -1.0*  Pinv_trunc_SVD(Rf_ext_der , 1E-1) * d_fc_des_to_world ; 
//   yarp::sig::Vector d_q_der_ext = -1.0*  Pinv_trunc_SVD(Rf_ext_der , 1E-1) * d_fc_des_to_world ; 
  // yarp::sig::Vector d_q_der_redu_1 = -1.0*  Pinv_trunc_SVD(Rf_redu_der , 1E-1) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_der_redu_2 = -1.0*  Pinv_trunc_SVD(Rf_redu_der , 1E-2) * d_fc_des_to_world ; 
 //----------------------------------------------------------------------------     
  
   std::cout << " ----------------------------------------------------- "   << std::endl;    
 //  std::cout << " With derivative terms "   << std::endl; 
 //  std::cout << " d_q_der_ext = "    <<  std::endl << d_q_der_ext.toString() << std::endl; 
 //  std::cout << " d_q_der_redu_1 = " <<  std::endl << d_q_der_redu_1.toString() << std::endl; 
 //  std::cout << " d_q_der_redu_2 = " <<  std::endl << (d_q_der_redu_2).toString() << std::endl; 
 //  std::cout << "---------------------------------------------------------------------------" <<  std::endl ; 
   
   
      
   yarp::sig::Vector d_q_der_redu_2_1 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der , 1E-1) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_der_redu_2_2 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der , 1E-2) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_der_redu_2_3 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der , 1E-3) * (F_z* d_fc_des_to_world) ;

 /*  std::cout << " d_q_der_redu_2_1 = " <<  std::endl << d_q_motor_no_deriv_2_1.toString() << std::endl;
   std::cout << " d_q_der_redu_2_2 = " <<  std::endl << d_q_motor_no_deriv_2_2.toString() << std::endl;
   std::cout << " d_q_der_redu_2_3 = " <<  std::endl << d_q_motor_no_deriv_2_3.toString() << std::endl;  */

   
   
   
 //----------------------------------------------------------------------------     
  /* // Computing derivatives of the COM part
   
     yarp::sig::Matrix Jac_w_COM_0_temp( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_body_0_temp( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_body_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_spa_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix zero_3_6q(3,size_q+6);
     zero_3_6q.zero();
     
     yarp::sig::Vector d_w_COM_0(3) ;
     yarp::sig::Matrix T_w_COM_0_temp(4,4) ;
     yarp::sig::Matrix T_w_COM_0(4,4) ;
     yarp::sig::Matrix T_COM_w_0(4,4) ;     
     yarp::sig::Matrix R_w_COM_0(3,3) ;
     yarp::sig::Matrix R_COM_w_0(3,3) ;

     yarp::sig::Matrix T_aw_COM_0(4,4)  ;
     yarp::sig::Matrix T_waist_COM_0(4,4);
     yarp::sig::Matrix T_COM_waist_0(4,4) ;
     
    
     model.iDyn3_model.getCOMJacobian( Jac_w_COM_0_temp ) ;  
     d_w_COM_0 = model.iDyn3_model.getCOM()  ;  
     T_w_COM_0_temp = Homogeneous(Eye_3, d_w_COM_0 ) ;
     T_aw_COM_0 =  T_aw_w_0  *T_w_COM_0_temp ;   
     T_aw_COM_0.setSubmatrix(Eye_3,0,0) ;
     T_waist_COM_0 =  T_waist_w_0 * T_w_aw_0*T_aw_COM_0 ;
     T_COM_waist_0 = iHomogeneous(T_waist_COM_0)   ;
     T_w_COM_0 = T_w_waist_0*  T_waist_COM_0;
     T_COM_w_0 = iHomogeneous(T_w_COM_0 ) ;
     R_w_COM_0 = getRot( T_w_COM_0 ) ;
     R_COM_w_0 = getRot( T_COM_w_0 ) ;
     Jac_COM_body_0_temp    =  Adjoint(Homogeneous(R_COM_w_0, zero_3))   *  Jac_w_COM_0_temp ;
     Jac_COM_spa_0 =  Adjoint( T_waist_COM_0   )  * Jac_COM_body_0_temp ;
     Jac_COM_spa_0.setSubmatrix(Eye_6,0,0) ;  // spatial on the waist with VKC     
     yarp::sig::Matrix Jac_COM_waist_0 = Adjoint(Homogeneous(Eye_3 , getTrasl(T_waist_COM_0) ) ) * Jac_COM_spa_0 ; // moving the pole on the COM
     //
     yarp::sig::Matrix Jac_COM_waist_0_red = Jac_COM_waist_0.submatrix(0,2, 0, (Jac_COM_waist_0.cols()-1)  )  ;
     yarp::sig::Vector mg_COM_0(3, 0.0) ;
     mg_COM_0[2] = - mg ;
     yarp::sig::Vector mg_waist_0= getRot( T_waist_COM_0 )*mg_COM_0 ;
     yarp::sig::Matrix U_mg_2 = -1.0*crossProductMatrix(mg_waist_0)*Jac_COM_waist_0_red  ;
     
   // Introducing derivative terms for COM
    yarp::sig::Matrix U_mg( 6, size_q+6 ) ;
    yarp::sig::Matrix U_s_mg( 6 , 6) ;  
    yarp::sig::Matrix Q_s_mg( 6 , size_q) ;
    
    U_mg.setSubmatrix( zero_3_6q, 0 , 0) ;
    U_mg.setSubmatrix( U_mg_2   , 3 , 0)  ; 
    U_s_mg = U_mg.submatrix(0,5 , 0, 5) ;
    Q_s_mg = U_mg.submatrix(0,5,  6, (U_mg.cols()-1)  ) ;

    yarp::sig::Matrix U_s_der_mg = U_s_der + U_s_mg ;             
    yarp::sig::Matrix Q_s_der_mg = Q_s_der + Q_s_mg ;
    
   yarp::sig::Matrix Rf_redu_der_mg = Rf_redu( J_c,  S_c,  Q_s_der_mg,  U_s_der_mg , Kc) ;
   
   //
   yarp::sig::Vector d_q_der_mg_1 = -1.0*  Pinv_trunc_SVD(Rf_redu_der_mg , 1E-1) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_der_mg_2 = -1.0*  Pinv_trunc_SVD(Rf_redu_der_mg , 1E-2) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_der_mg_3 = -1.0*  Pinv_trunc_SVD(Rf_redu_der_mg , 1E-3) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_der_mg_4 = -1.0*  Pinv_trunc_SVD(Rf_redu_der_mg , 1E-4) * d_fc_des_to_world ;    
   yarp::sig::Vector d_q_der_mg_5 = -1.0*  Pinv_trunc_SVD(Rf_redu_der_mg , 1E-5) * d_fc_des_to_world ; 
 //----------------------------------------------------------------------------     
 /*  yarp::sig::Matrix U_rf ;  
   yarp::sig::Vector S_rf ;
   yarp::sig::Matrix V_rf ;
   yarp::math::SVD(Rf_redu_der_mg, U_rf, S_rf, V_rf);

   std::cout << " S_rf = " <<  std::endl << S_rf.toString() << std::endl; 

   std::cout << " ----------------------------------------------------- "   << std::endl;   */ 
 /*   std::cout << " With COM derivative terms "   << std::endl; 
//   std::cout << " d_q_der_mg_1 = " <<  std::endl << d_q_der_mg_1.toString() << std::endl;   
   std::cout << " d_q_der_mg_2 = " <<  std::endl << (d_q_der_mg_2).toString() << std::endl; 
//  std::cout << " d_q_der_mg_3 = " <<  std::endl << d_q_der_mg_3.toString() << std::endl; 
//   std::cout << " d_q_der_mg_4 = " <<  std::endl << d_q_der_mg_4.toString() << std::endl; 
//   std::cout << " d_q_der_mg_5 = " <<  std::endl << d_q_der_mg_5.toString() << std::endl; 
 /*   std::cout << "---------------------------------------------------------------------------" <<  std::endl ;  */
    
    
      
 /*   yarp::sig::Vector d_q_der_mg_2_1 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der_mg , 1E-1) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_der_mg_2_2 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der_mg , 1E-2) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_der_mg_2_3 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_der_mg , 1E-3) * (F_z* d_fc_des_to_world) ;

  std::cout << " d_q_der_mg_2_1 = " <<  std::endl << d_q_der_mg_2_1.toString() << std::endl;
   std::cout << " d_q_der_mg_2_2 = " <<  std::endl << d_q_der_mg_2_2.toString() << std::endl;
   std::cout << " d_q_der_mg_2_3 = " <<  std::endl << d_q_der_mg_2_3.toString() << std::endl;  */

   //----------------------------------------------------------------------------------------------------------------
   
   
   
   
 //----------------------------------------------------------------------------     
   // Computing derivatives of the COM part
   
     yarp::sig::Matrix Jac_w_COM_0_temp( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_body_0_temp( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_body_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix Jac_COM_spa_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     yarp::sig::Matrix zero_3_6q(3,size_q+6);
     zero_3_6q.zero();
     
     yarp::sig::Vector d_w_COM_0(3) ;
     yarp::sig::Matrix T_w_COM_0_temp(4,4) ;
     yarp::sig::Matrix T_w_COM_0(4,4) ;
     yarp::sig::Matrix T_COM_w_0(4,4) ;     
     yarp::sig::Matrix R_w_COM_0(3,3) ;
     yarp::sig::Matrix R_COM_w_0(3,3) ;

     yarp::sig::Matrix T_aw_COM_0(4,4)  ;
     yarp::sig::Matrix T_waist_COM_0(4,4);
     yarp::sig::Matrix T_COM_waist_0(4,4) ;
     
    
     model.iDyn3_model.getCOMJacobian( Jac_w_COM_0_temp ) ;  
     d_w_COM_0 = model.iDyn3_model.getCOM()  ;  
     T_w_COM_0_temp = Homogeneous(Eye_3, d_w_COM_0 ) ;
     T_aw_COM_0 =  T_aw_w_0  *T_w_COM_0_temp ;   
     T_aw_COM_0.setSubmatrix(Eye_3,0,0) ;
     T_waist_COM_0 =  T_waist_w_0 * T_w_aw_0*T_aw_COM_0 ;
     T_COM_waist_0 = iHomogeneous(T_waist_COM_0)   ;
     T_w_COM_0 = T_w_waist_0*  T_waist_COM_0;
     T_COM_w_0 = iHomogeneous(T_w_COM_0 ) ;
     
     
     R_w_COM_0 = getRot( T_w_COM_0 ) ;
     R_COM_w_0 = getRot( T_COM_w_0 ) ;
     
     
     Jac_COM_body_0_temp    =  Adjoint(Homogeneous(R_COM_w_0, zero_3))   *  Jac_w_COM_0_temp ;
     Jac_COM_spa_0 =  Adjoint( T_waist_COM_0   )  * Jac_COM_body_0_temp ;
     Jac_COM_spa_0.setSubmatrix(Eye_6,0,0) ;  // spatial on the waist with VKC     
     // moving the pole on the COM  
     yarp::sig::Matrix Jac_COM_waist_0 = Adjoint( Homogeneous( Eye_3, getTrasl(T_COM_waist_0) )  ) *  Jac_COM_spa_0 ;
     //
     yarp::sig::Matrix Jac_COM_waist_0_trasl = Jac_COM_waist_0.submatrix(0,2, 0, (Jac_COM_waist_0.cols()-1)  )  ;
     yarp::sig::Vector mg_COM_0(3, 0.0) ;
     mg_COM_0[2] =  mg ; // positive because is exterted from the robot to the environment
     yarp::sig::Vector mg_waist_0= getRot( T_waist_COM_0 )*mg_COM_0 ;
     yarp::sig::Matrix D_mg_rot = (-1.0*crossProductMatrix(mg_waist_0))*Jac_COM_waist_0_trasl  ;
     
   // Introducing derivative terms for COM
    yarp::sig::Matrix U_mg( 6, size_q+6 ) ;
    yarp::sig::Matrix U_s_mg( 6 , 6) ;  
    yarp::sig::Matrix Q_s_mg( 6 , size_q) ;
    
    U_mg.setSubmatrix( zero_3_6q, 0 , 0 ) ;
    U_mg.setSubmatrix( D_mg_rot,  3 , 0 )  ; 
    U_s_mg = U_mg.submatrix(0,5 , 0, 5) ;
    Q_s_mg = U_mg.submatrix(0,5,  6, (U_mg.cols()-1)  ) ;

  //  yarp::sig::Matrix U_s_der_mg = U_s_der + U_s_mg ;             
  //  yarp::sig::Matrix Q_s_der_mg = Q_s_der + Q_s_mg ;
    
   yarp::sig::Matrix Rf_redu_mg = Rf_redu( J_c,  S_c,  Q_s_mg,  U_s_mg , Kc) ;
   
   //
   yarp::sig::Vector d_q_mg_1_1 = -1.0*  Pinv_trunc_SVD(Rf_redu_mg , 1E-1) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_mg_1_2 = -1.0*  Pinv_trunc_SVD(Rf_redu_mg , 1E-2) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_mg_1_3 = -1.0*  Pinv_trunc_SVD(Rf_redu_mg , 1E-3) * d_fc_des_to_world ; 

 //----------------------------------------------------------------------------     

 /*   std::cout << "---------------------------------------------------------------------------" <<  std::endl ;  */
 
   yarp::sig::Vector d_q_mg_2_1 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_mg , 1E-1) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_2_2 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_mg , 1E-2) * (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_2_3 = -1.0*  Pinv_trunc_SVD(F_z*Rf_redu_mg , 1E-3) * (F_z* d_fc_des_to_world) ;
        
   std::cout << " ----------------------------------------------------- "   << std::endl;  
   std::cout << " With COM derivative terms, no nullspace Project "   << std::endl; 
 /*  std::cout << " d_q_mg_1_1 = " <<  std::endl << d_q_mg_1_1.toString() << std::endl;   
   std::cout << " d_q_mg_1_2 = " <<  std::endl << d_q_mg_1_2.toString() << std::endl; 
   std::cout << " d_q_mg_1_3 = " <<  std::endl << d_q_mg_1_3.toString() << std::endl; */

 //  std::cout << " d_q_mg_2_1 = " <<  std::endl << d_q_mg_2_1.toString() << std::endl;
   std::cout << " d_q_mg_2_2 = " <<  std::endl << d_q_mg_2_2.toString() << std::endl;
 //  std::cout << " d_q_mg_2_3 = " <<  std::endl << d_q_mg_2_3.toString() << std::endl;
   
   
   
   
   
   yarp::sig::Vector d_q_mg_3_1 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-1) *  S_c* ( d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_3_2 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-2) *  S_c* ( d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_3_3 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-3) *  S_c* ( d_fc_des_to_world) ;

   yarp::sig::Vector d_q_mg_4_1 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-1) *  S_c* (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_4_2 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-2) *  S_c* (F_z* d_fc_des_to_world) ;
   yarp::sig::Vector d_q_mg_4_3 = -1.0*  Pinv_trunc_SVD( U_mg , 1E-3) *  S_c* (F_z* d_fc_des_to_world) ;
   
 
/*   std::cout << " d_q_mg_3_1 = " <<  std::endl << d_q_mg_3_1.toString() << std::endl;   
   std::cout << " d_q_mg_3_2 = " <<  std::endl << d_q_mg_3_2.toString() << std::endl; 
   std::cout << " d_q_mg_3_3 = " <<  std::endl << d_q_mg_3_3.toString() << std::endl; 

   std::cout << " d_q_mg_4_1 = " <<  std::endl << d_q_mg_4_1.toString() << std::endl;
   std::cout << " d_q_mg_4_2 = " <<  std::endl << d_q_mg_4_2.toString() << std::endl;
   std::cout << " d_q_mg_4_3 = " <<  std::endl << d_q_mg_4_3.toString() << std::endl;   */
   
   d_q_mg_3_1 = nullspaceProjection( Complete_Jac, 1E-5  ) *d_q_mg_3_1 ;
   d_q_mg_3_2 = nullspaceProjection( Complete_Jac, 1E-5  ) *d_q_mg_3_2 ;
   d_q_mg_3_3 = nullspaceProjection( Complete_Jac, 1E-5  ) *d_q_mg_3_3 ;
 
   
   d_q_mg_4_1 = nullspaceProjection( Complete_Jac, 1E-5 )  *d_q_mg_4_1 ;
   d_q_mg_4_2 = nullspaceProjection( Complete_Jac, 1E-5 )  *d_q_mg_4_2 ;
   d_q_mg_4_3 = nullspaceProjection( Complete_Jac, 1E-5 )  *d_q_mg_4_3 ;

 
   std::cout << " ----------------------------------------------------- "   << std::endl;  
   std::cout << " With COM derivative terms, with nullspace Project "   << std::endl;  
   
 /*  std::cout << " d_q_mg_3_1 = " <<  std::endl << (d_q_mg_3_1).toString() << std::endl;
   std::cout << " d_q_mg_3_2 = " <<  std::endl << (d_q_mg_3_2).toString() << std::endl;
   std::cout << " d_q_mg_3_3 = " <<  std::endl << (d_q_mg_3_3).toString() << std::endl;
  
   std::cout << " d_q_mg_4_1 = " <<  std::endl << (d_q_mg_4_1).toString() << std::endl;
   std::cout << " d_q_mg_4_2 = " <<  std::endl << (d_q_mg_4_2).toString() << std::endl;
   std::cout << " d_q_mg_4_3 = " <<  std::endl << (d_q_mg_4_3).toString() << std::endl;  */

   
   d_q_mg_3_1 = d_q_mg_3_1.subVector(6, d_q_mg_3_1.length()-1 ) ;
   d_q_mg_3_2 = d_q_mg_3_2.subVector(6, d_q_mg_3_2.length()-1 ) ; 
   d_q_mg_3_3 = d_q_mg_3_3.subVector(6, d_q_mg_3_3.length()-1 ) ;
 
   d_q_mg_4_1 = d_q_mg_4_1.subVector(6, d_q_mg_4_1.length()-1 ) ; 
   d_q_mg_4_2 = d_q_mg_4_2.subVector(6, d_q_mg_4_2.length()-1 ) ; 
   d_q_mg_4_3 = d_q_mg_4_3.subVector(6, d_q_mg_4_3.length()-1 ) ;  
   
    
   std::cout << " ----------------------------------------------------- "   << std::endl;  
   std::cout << " With COM derivative terms, short, with nullspace Project "   << std::endl;  
   
 /*  std::cout << " d_q_mg_3_1 = " <<  std::endl << (d_q_mg_3_1).toString() << std::endl;
   std::cout << " d_q_mg_3_2 = " <<  std::endl << (d_q_mg_3_2).toString() << std::endl;
   std::cout << " d_q_mg_3_3 = " <<  std::endl << (d_q_mg_3_3).toString() << std::endl;  */
  
   std::cout << " d_q_mg_4_1 = " <<  std::endl << (d_q_mg_4_1).toString() << std::endl;
   std::cout << " d_q_mg_4_2 = " <<  std::endl << (d_q_mg_4_2).toString() << std::endl;
   std::cout << " d_q_mg_4_3 = " <<  std::endl << (d_q_mg_4_3).toString() << std::endl;
   

   
   
   

   
   //----------------------------------------------------------------------------------------------------------------
   // Filtering d_q_motor
    yarp::sig::Vector d_q_motor_filter(size_q) ;
    d_q_motor_filter = d_q_motor_no_deriv_2_2 ;  //  d_q_der_mg_2 ;  //
    double     max;
    double     min ;
    double min_abs;
    double d_q_max ;
    max=findMax( d_q_motor_filter ) ;
    min=findMin( d_q_motor_filter ) ;    
    max = std::abs(max) ;
    min_abs = std::abs(min) ;
    if(min_abs>max) max = min_abs;
    d_q_max = 0.08722*2.0 ;  // 0.08722 rad = 5 deg  // 0.08722/100 <--> 0.08722/500  seems fine
    if(max>d_q_max)  { d_q_motor_filter =  (d_q_motor_filter/max)*d_q_max ; } //(d_q_motor_filter/max)  *d_q_max  

    //------------------------------------------------------------------------
    // Projection of the Initial Configuration 
   
   // Initializing the vector of the initial configuration
    yarp::sig::Vector q_motor_init(robot.getNumberOfJoints() ) ;		       
    yarp::sig::Vector q_right_arm_init(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_left_arm_init(robot.left_arm.getNumberOfJoints()) ;
    yarp::sig::Vector q_torso_init(robot.torso.getNumberOfJoints()) ;
    yarp::sig::Vector q_right_leg_init(robot.right_leg.getNumberOfJoints()) ;
    yarp::sig::Vector q_left_leg_init(robot.left_leg.getNumberOfJoints()) ;    
    
    robot.fromIdynToRobot(  q_motor_init,
                            q_right_arm_init,
                            q_left_arm_init,
                            q_torso_init,
                            q_right_leg_init,
                            q_left_leg_init  ) ; 
    
    q_right_arm_init = right_arm_configuration  ; 			    
    q_left_arm_init  = left_arm_configuration   ;
    q_torso_init     = torso_configuration      ;
    q_right_leg_init = left_leg_configuration   ;
    q_left_leg_init  = right_leg_configuration  ;
    
    robot.fromRobotToIdyn( q_right_arm_init ,
                           q_left_arm_init  ,
                           q_torso_init  ,
                           q_right_leg_init ,
                           q_left_leg_init  ,
                           q_motor_init );    
    
    //Rf_redu_der_mg
    yarp::sig::Matrix  Rf_redu_der_mg_filter = filter_SVD(Rf_redu_mg, 1E-1) ;
    
    yarp::sig::Vector dq_conf = q_motor_init - q_motor_side ;
    
    yarp::sig::Vector dq_conf_proj = nullspaceProjection(Rf_redu_der_mg_filter ) * dq_conf ;
    
   //---------------------------------------------------------------------------//
     //---------------------------------------------------------------------------------------------------------//  
//   writing data on a file
    //std::ofstream r_ankle ;
    //r_ankle.open ("r_ankle.txt");
    double err = norm( F_z*d_fc_des_to_world )  ;  // d_fc_des_to_world
    std::ofstream err_cl ( "err.m", std::ios::app );
    if( err_cl.is_open() )
    err_cl <<  err << std::endl;  
 
    
    //----------------------------------------------------------------------------------//
    yarp::sig::Vector q_ref_ToMove_f(robot.right_arm.getNumberOfJoints()) ; 
    yarp::sig::Vector q_ref_ToMove_right_arm(robot.left_arm.getNumberOfJoints()) ;
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
    

    double err_min = 40; //10.0 ;
    double err_max = 80;  //40.0 ; 
    double alpha = (1.0/(err_max - err_min)) *( norm(FC_DES_LEFT_sensor- FC_FILTERED_LEFT_sensor) 
                                    + norm(FC_DES_RIGHT_sensor- FC_FILTERED_RIGHT_sensor) )  ; 
    //       ; // (norm( F_z*d_fc_des_to_world ) - err_min) ;
    std::cout << " alpha = "  << alpha << std::endl; 
    if(alpha<0.001 ){ alpha = 0.0 ; } 
    if(alpha>0.999 ){ alpha = 1.0 ; }   
    std::cout << " alpha = "  << alpha << std::endl; 
   

    q_ref_ToMove = q_ref_ToMove  + (1.0/5.0 )*alpha*d_q_mg_4_2 
                                 + (1.0/20.0 )*alpha* d_q_motor_no_deriv_2_2   ;  //   d_q_mg_4_2
                                //+ (1.0/1.0 )*alpha* d_q_motor_no_deriv_2_2    ;//+ (1.0/1.0 )*alpha* d_q_mg_6_2  ;// - (10.0/1.0 )*alpha*d_q_mg_2_2  ;  //; + (1.0/1.0 )*alpha*  d_q_motor_filter ;

   
   
   // d_q_der_mg_2_2 ;
   //   d_q_der_redu_2_2  ;
   //  d_q_motor_no_deriv_2_2 ; //+ (1.0/1.0 )*d_q_motor_no_deri_no_u_2;//    ;  // ;//
  //  + (1.0 / 1.0 ) * dq_conf_proj; 
   //d_q_motor_filter   + (1.0 / 10.0 ) * dq_conf_proj ; // d_q_motor_no_deriv  ;  
  
   //   d_q_der_mg_2   + (1.0 / 10.0 ) * dq_conf_proj ; 
   //   d_q_der_redu_2   ;  //
   //  + (1.0/1.0 )* d_q_der_mg_2    ;  //
   //  + (1.0/1.0 )*d_q_motor_filter ; // + (1.0/1.0 )*d_q_motor_filter ; // to balance... q_ref_ToMove  + d_q_motor_desired ;
       
   robot.move(q_ref_ToMove);  
     
     
     
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
 double V_l_c1 =   V_ij(sigma_frict(-1.0*fc_l_c1_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c1_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c1_filt, 0)) ;
// Contact left 2
 double V_l_c2 =   V_ij(sigma_frict(-1.0*fc_l_c2_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c2_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c2_filt, 0)) ;
  // Contact left  3
 double V_l_c3 =   V_ij(sigma_frict(-1.0*fc_l_c3_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c3_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c3_filt, 0)) ;
  // Contact left  4
 double V_l_c4 =   V_ij(sigma_frict(-1.0*fc_l_c4_filt, mu_l)) +  V_ij(sigma_max(-1.0*fc_l_c4_filt, 700)) +  V_ij(sigma_min(-1.0*fc_l_c4_filt, 0)) ;
  //
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
    
    
    
    
    
    
 

  //  yarp::sig::Matrix Rf_ext_1 = Rf_ext( J_c , S_c ,Q_j, Q_s,  U_j,  U_s,  Kc, Kq)  ;
    
  //  yarp::sig::Matrix Rf_redu_1 = Rf_redu( J_c,  S_c,  Q_s,  U_s, Kc) ;

     //  yarp::math::SVD(R_f_no_deriv, U1, S1, V1 )  ;
    
  //  yarp::math::SVD(Rf_redu_1, U1, S1, V1);
 //   yarp::sig::Vector S_redu = S1 ;
    
 //   std::cout << " S_redu = " <<  std::endl << S_redu.toString() << std::endl; 
    
//    yarp::sig::Matrix Eye_phi= Phi_d ; 
 //   Eye_phi.eye() ;

 //   yarp::sig::Vector d_q_redu_1 = -1.0* pinv( Rf_redu_1 , 1E-6 ) * d_fc_des_to_world ; 
    
    
 //   std::cout << " d_q_redu_1 = " <<  std::endl << d_q_redu_1.toString() << std::endl; 

    
   
 /*  //   writing data on a file
    double err = norm( d_fc_des_to_world )  ;
    std::ofstream err_cl ( "err.m", std::ios::app );
    if( err_cl.is_open() )
    err_cl <<  err << std::endl;  */
   
   
   

   
     
 //    q_ref_ToMove = q_ref_ToMove ; // + d_q_motor_no_deriv_damp_2 ; // + (1.0/1.0 )*d_q_motor_filter ; // to balance... q_ref_ToMove  + d_q_motor_desired ;
       
 //    robot.move(q_ref_ToMove);  // q_ref_ToMove
 
     // robot.left_arm.move(q_ref_ToMove_left_arm);
    
  
    //------------------------------------------------------------------------------------------------------------------------------------
    
  

   
   //---------------------------------------------------------------------------------------
 
   
//   std::cout << " ----------------------------------------------------- "   << std::endl;    
//   std::cout << " With derivative terms "   << std::endl; 
   
  //  Kc.eye() ;
  //  Kc = 1E7*Kc ;
 
 /*   // Alternative formulation for R_f
   Q_j_1 = -1.0*Q_j - 1.0* J_c.transposed()*Kc*J_c  ;
   U_j_1 = -1.0*U_j -1.0*J_c.transposed() *Kc*S_c.transposed() ;    
   Q_s_1 = -1.0*Q_s-1.0*S_c*Kc*J_c  ;
   U_s_1 = -1.0*U_s-1.0*S_c*Kc*S_c.transposed() ;    
   L = yarp::math::luinv(U_s_1)* Q_s_1 ;
   M = Q_j_1-U_j_1*L ;    
   H = Kq-M ;
   F = -1.0*yarp::math::luinv(H)*Kq ;    
   E = -1.0*Kc* S_c.transposed()* L *F ;
   R_f_1 = E+Kc*J_c*F  ;
   
   yarp::sig::Matrix R_f_deriv = R_f_1 ;
   
   yarp::math::SVD(R_f_deriv, U1, S1, V1 )  ;

   yarp::sig::Vector S_deriv = S1 ;
   std::cout << " S_deriv = " <<  std::endl << S_deriv.toString() << std::endl;  */

   
   
   
   /* d_fc_des_to_world = fc_des_to_world - fc_to_world_0 ;
    yarp::sig::Vector d_q_motor_deriv_no_damp = -1.0* pinv( R_f_1 , 1E-6 ) * d_fc_des_to_world ; 
    fc_teor   = fc_to_world_0 - 1.0*R_f_1 *d_q_motor_deriv ;

      yarp::sig::Vector d_q_trunc_1 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-1) * d_fc_des_to_world ;   */
    
  /*    yarp::sig::Vector d_q_trunc_2 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-2) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_trunc_3 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-3) * d_fc_des_to_world ;  
   yarp::sig::Vector d_q_trunc_4 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-4) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_trunc_5 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-5) * d_fc_des_to_world ;  
   yarp::sig::Vector d_q_trunc_6 =   -1.0*  Pinv_trunc_SVD(R_f_1 , 1E-6) * d_fc_des_to_world ;  

   yarp::sig::Vector d_q_regu_1 =    -1.0*  Pinv_Regularized( R_f_1 , 1E7 ) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_regu_2 =    -1.0*  Pinv_Regularized( R_f_1 , 1E8 ) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_regu_3 =    -1.0*  Pinv_Regularized( R_f_1 , 1E9 ) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_regu_4 =    -1.0*  Pinv_Regularized( R_f_1 , 1E10 ) * d_fc_des_to_world ; 
   yarp::sig::Vector d_q_regu_5 =    -1.0*  Pinv_Regularized( R_f_1 , 1E11 ) * d_fc_des_to_world ; 

   yarp::sig::Vector d_q_iter_3 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 1) ;
   yarp::sig::Vector d_q_iter_4 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 10) ;
   yarp::sig::Vector d_q_iter_5 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 20) ;
   yarp::sig::Vector d_q_iter_6 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 16) ;
   yarp::sig::Vector d_q_iter_7 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 17) ;
   yarp::sig::Vector d_q_iter_8 =  -1.0* x_Pinv_Iter( R_f_1 , d_fc_des_to_world, 18) ;  */
  
 //    yarp::sig::Vector temp_Atb =  -1.0* R_f_1.transposed()*d_fc_des_to_world;
  
 //  std::cout << " d_q_trunc_1 = " <<  std::endl << (d_q_trunc_1).toString() << std::endl; 
  /* std::cout << " d_q_trunc_2 = " <<  std::endl << (d_q_trunc_2).toString() << std::endl;  
   std::cout << " d_q_trunc_3 = " <<  std::endl << (d_q_trunc_3).toString() << std::endl; 
   std::cout << " d_q_trunc_4 = " <<  std::endl << (d_q_trunc_4).toString() << std::endl; 
   std::cout << " d_q_trunc_5 = " <<  std::endl << (d_q_trunc_5).toString() << std::endl;   */

   
 /*  std::cout << " d_q_regu_1 = " <<  std::endl << (d_q_regu_1).toString() << std::endl; 
   std::cout << " d_q_regu_2 = " <<  std::endl << (d_q_regu_2).toString() << std::endl; 
   std::cout << " d_q_regu_3 = " <<  std::endl << (d_q_regu_3).toString() << std::endl; 
   std::cout << " d_q_regu_4 = " <<  std::endl << (d_q_regu_4).toString() << std::endl; 
   std::cout << " d_q_regu_5 = " <<  std::endl << (d_q_regu_5).toString() << std::endl;    

   
   std::cout << " d_q_iter_3 = " <<  std::endl << (d_q_iter_3).toString()  << std::endl; 
   std::cout << " d_q_iter_4 = " <<  std::endl << (d_q_iter_4).toString()  << std::endl; 
   std::cout << " d_q_iter_5 = " <<  std::endl << (d_q_iter_5).toString()  << std::endl; 
   std::cout << " d_q_iter_6 = " <<  std::endl << (d_q_iter_6).toString()  << std::endl; 
   std::cout << " d_q_iter_7 = " <<  std::endl << (d_q_iter_7).toString()  << std::endl;  
   std::cout << " d_q_iter_8 = " <<  std::endl << (d_q_iter_8).toString()  << std::endl;     */
    
    
    
    
    
    
    
    
    
    
    
    
  

    
 
 
    //----------------------------------------------------------------------------------------------------------------
/* // Filtering
    d_q_motor_filter = d_q_motor_deriv_damp_3 ;  //d_q_motor_deriv_damp_2
    max=findMax( d_q_motor_filter ) ;
    min=findMin( d_q_motor_filter ) ;    
    max = std::abs(max) ;
    min_abs = std::abs(min) ;
    if(min_abs>max) max = min_abs;
    d_q_max = 0.08722/100 ;  // 0.08722 rad = 5 deg  // 0.08722/100  seems fine
    if(max>d_q_max)  { d_q_motor_filter =  (d_q_motor_filter/max)*d_q_max ; } //(d_q_motor_filter/max)  *d_q_max  */


  









     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
  
     
   
     
     
     
     
     
     
     
     
     
     
     
     
     
  
     
     
     
     
     
     

 
          
       
 //---------------------------------------------------------------------------------------------------------//  
//   writing data on a file
 /*   //std::ofstream r_ankle ;
    double err = norm( d_fc_desired_to_world )  ;
    //r_ankle.open ("r_ankle.txt");
    std::ofstream err_cl ( "err.m", std::ios::app );
    if( err_cl.is_open() )
    err_cl <<  err << std::endl;  */
 
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


