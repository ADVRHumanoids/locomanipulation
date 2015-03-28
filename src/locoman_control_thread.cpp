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
    control_thread( module_prefix, rf, ph ), command_interface(module_prefix)
{
    left_arm_joints = robot.left_arm.getNumberOfJoints();
    omega = 0.1;
    phi = 10;
    tick = 0;
    max_vel= 0.35;
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
    
    //robot.setPositionDirectMode();    
    
    robot.setPositionMode() ;
    robot.setReferenceSpeed(max_vel) ;
    
    
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
    
    d_q_des = (q_des - q_motor_0)/100 ;
    
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

    usleep(2000*1000) ; // usleep(milliseconds*1000)
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
    yarp::sig::Vector tau    = robot.senseTorque() ;
    //     
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



void locoman_control_thread::run()
{     
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
 /*   yarp::sig::Vector ft_r_ankle_1(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle_1 )) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    yarp::sig::Vector ft_l_ankle_1(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle_1 )) std::cout << "ERROR READING SENSOR l_ankle" << std::endl;     
    
    usleep(10*1000) ;       
    yarp::sig::Vector ft_r_ankle_2(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle_2 )) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    yarp::sig::Vector ft_l_ankle_2(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle_2 )) std::cout << "ERROR READING SENSOR l_ankle" << std::endl;     
    
    usleep(10*1000) ;       
    yarp::sig::Vector ft_r_ankle_3(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle_3 )) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    yarp::sig::Vector ft_l_ankle_3(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle_3 )) std::cout << "ERROR READING SENSOR l_ankle" << std::endl;    

    usleep(10*1000) ;       
    yarp::sig::Vector ft_r_ankle_4(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle_4 )) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    yarp::sig::Vector ft_l_ankle_4(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle_4 )) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 

    usleep(10*1000) ;       
    yarp::sig::Vector ft_r_ankle_5(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle_5 )) std::cout << "ERROR READING SENSOR r_ankle" << std::endl; 
    yarp::sig::Vector ft_l_ankle_5(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle_5 )) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    
   // yarp::sig::Vector wrench_tot_l_2 = ft_l_ankle_2 + Adjoint_MT(T_l_r) * ft_r_ankle_2 ;    
    
    yarp::sig::Vector ft_l_ankle(6,0.0);
    yarp::sig::Vector ft_r_ankle(6,0.0);
    
   // yarp::sig::Vector Wrench_mean_l(6,0.0);
    ft_l_ankle[0] = ( ft_l_ankle_1[0] + ft_l_ankle_2[0] + ft_l_ankle_3[0] + ft_l_ankle_4[0] + ft_l_ankle_5[0] )/5  ;
    ft_l_ankle[1] = ( ft_l_ankle_1[1] + ft_l_ankle_2[1] + ft_l_ankle_3[1] + ft_l_ankle_4[1] + ft_l_ankle_5[1] )/5  ;
    ft_l_ankle[2] = ( ft_l_ankle_1[2] + ft_l_ankle_2[2] + ft_l_ankle_3[2] + ft_l_ankle_4[2] + ft_l_ankle_5[2] )/5  ;
    ft_l_ankle[3] = ( ft_l_ankle_1[3] + ft_l_ankle_2[3] + ft_l_ankle_3[3] + ft_l_ankle_4[3] + ft_l_ankle_5[3] )/5  ;
    ft_l_ankle[4] = ( ft_l_ankle_1[4] + ft_l_ankle_2[4] + ft_l_ankle_3[4] + ft_l_ankle_4[4] + ft_l_ankle_5[4] )/5  ;
    ft_l_ankle[5] = ( ft_l_ankle_1[5] + ft_l_ankle_2[5] + ft_l_ankle_3[5] + ft_l_ankle_4[5] + ft_l_ankle_5[5] )/5  ;  
    
    ft_r_ankle[0] = ( ft_r_ankle_1[0] + ft_r_ankle_2[0] + ft_r_ankle_3[0] + ft_r_ankle_4[0] + ft_r_ankle_5[0] )/5  ;
    ft_r_ankle[1] = ( ft_r_ankle_1[1] + ft_r_ankle_2[1] + ft_r_ankle_3[1] + ft_r_ankle_4[1] + ft_r_ankle_5[1] )/5  ;
    ft_r_ankle[2] = ( ft_r_ankle_1[2] + ft_r_ankle_2[2] + ft_r_ankle_3[2] + ft_r_ankle_4[2] + ft_r_ankle_5[2] )/5  ;
    ft_r_ankle[3] = ( ft_r_ankle_1[3] + ft_r_ankle_2[3] + ft_r_ankle_3[3] + ft_r_ankle_4[3] + ft_r_ankle_5[3] )/5  ;
    ft_r_ankle[4] = ( ft_r_ankle_1[4] + ft_r_ankle_2[4] + ft_r_ankle_3[4] + ft_r_ankle_4[4] + ft_r_ankle_5[4] )/5  ;
    ft_r_ankle[5] = ( ft_r_ankle_1[5] + ft_r_ankle_2[5] + ft_r_ankle_3[5] + ft_r_ankle_4[5] + ft_r_ankle_5[5] )/5  ;      */   

 
    
    yarp::sig::Vector ft_r_ankle(6,0.0);
    if(!robot.senseftSensor("r_ankle", ft_r_ankle)) std::cout << "ERROR READING SENSOR r_ankle" << std::endl;     
    yarp::sig::Vector ft_l_ankle(6,0.0);
    if(!robot.senseftSensor("l_ankle", ft_l_ankle)) std::cout << "ERROR READING SENSOR l_ankle" << std::endl; 
    
    
    //  NO DELETE
//---------------------------------------------------------------------------------------------------------//  
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

    

//----------------------------------------------------------------------------------------//
// Computing contact forces at each point - Left Foot
    
    yarp::sig::Matrix T_w_l_ankle(  4 ,   4 ) ;
    yarp::sig::Matrix T_w_l_foot_upper_left_link(   4 ,   4 ) ;	
    yarp::sig::Matrix T_w_l_foot_upper_right_link(  4 ,   4 ) ;	
    yarp::sig::Matrix T_w_l_foot_lower_left_link(   4 ,   4 ) ;    
    yarp::sig::Matrix T_w_l_foot_lower_right_link(  4 ,   4 ) ;

    
    yarp::sig::Matrix T_l_l1(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l2(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l3(  4 ,   4 ) ;
    yarp::sig::Matrix T_l_l4(  4 ,   4 ) ;

    T_w_l_ankle = model.iDyn3_model.getPosition(l_ankle_index) ;
    T_w_l_foot_upper_left_link  = model.iDyn3_model.getPosition(l_foot_upper_left_link_index)  ;    
    T_w_l_foot_upper_right_link = model.iDyn3_model.getPosition(l_foot_upper_right_link_index) ;  
    T_w_l_foot_lower_left_link  = model.iDyn3_model.getPosition(l_foot_lower_left_link_index)  ;
    T_w_l_foot_lower_right_link = model.iDyn3_model.getPosition(l_foot_lower_right_link_index) ;    
  
    T_l_l1 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_upper_left_link  ;
    T_l_l2 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_upper_right_link ; 
    T_l_l3 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_lower_left_link  ;
    T_l_l4 = yarp::math::luinv(T_w_l_ankle)*T_w_l_foot_lower_right_link ;
    
    
//----------------------------------------------------------------------------------------//
// Computing contact forces at each point - Right Foot    
    
    yarp::sig::Matrix T_w_r_ankle(  4 ,   4 ) ;	
    yarp::sig::Matrix T_w_r_foot_upper_left_link(   4 ,   4 ) ;	
    yarp::sig::Matrix T_w_r_foot_upper_right_link(  4 ,   4 ) ;	
    yarp::sig::Matrix T_w_r_foot_lower_left_link(   4 ,   4 ) ;    
    yarp::sig::Matrix T_w_r_foot_lower_right_link(  4 ,   4 ) ;

	
    yarp::sig::Matrix T_r_r1(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r2(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r3(  4 ,   4 ) ;
    yarp::sig::Matrix T_r_r4(  4 ,   4 ) ;

    T_w_r_ankle = model.iDyn3_model.getPosition(r_ankle_index) ;
    T_w_r_foot_upper_left_link  = model.iDyn3_model.getPosition(r_foot_upper_left_link_index)  ;    
    T_w_r_foot_upper_right_link  = model.iDyn3_model.getPosition(r_foot_upper_right_link_index) ;  
    T_w_r_foot_lower_left_link  = model.iDyn3_model.getPosition(r_foot_lower_left_link_index)  ;
    T_w_r_foot_lower_right_link = model.iDyn3_model.getPosition(r_foot_lower_right_link_index) ;     

    T_r_r1 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_upper_left_link  ; 
    T_r_r2 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_upper_right_link ; 
    T_r_r3 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_lower_left_link  ;
    T_r_r4 = iHomogeneous(T_w_r_ankle) *T_w_r_foot_lower_right_link ;
    
    std::cout << "  ------------------------------------------------------------------------------------- "  << std::endl ;     

     yarp::sig::Matrix map_l_fcToSens =   fConToSens( l_ankle_index, 
						      l_foot_upper_left_link_index, 
					              l_foot_upper_right_link_index,						      
						      l_foot_lower_left_link_index, 
						      l_foot_lower_right_link_index ) ;
						      
     yarp::sig::Matrix map_r_fcToSens =   fConToSens( r_ankle_index, 
					              r_foot_upper_left_link_index, 
						      r_foot_upper_right_link_index,
						      r_foot_lower_left_link_index, 
						      r_foot_lower_right_link_index ) ;	
						      
    yarp::sig::Vector fc_l_contacts_to_robot =  yarp::math::pinv( map_l_fcToSens)  *  ft_l_ankle     ;
    yarp::sig::Vector fc_r_contacts_to_robot =  yarp::math::pinv( map_r_fcToSens)  *  ft_r_ankle     ;
    yarp::sig::Vector fc_l_contacts_to_world =  - 1.0 * fc_l_contacts_to_robot     ;
    yarp::sig::Vector fc_r_contacts_to_world =  - 1.0 * fc_r_contacts_to_robot     ;
    
    yarp::sig::Matrix T_l_r = iHomogeneous(T_w_l_ankle)*T_w_r_ankle ;


    
  //--------------------------------------------------------------------------------------------------------------------------//
    // Testing Jacobians
 /*   // WAIST
    int waist_index = model.iDyn3_model.getLinkIndex("Waist") ;
    yarp::sig::Matrix  T_w_waist = model.iDyn3_model.getPosition(waist_index) ;    
    yarp::sig::Matrix  T_waist_w = iHomogeneous(T_w_waist) ;
    
    yarp::sig::Matrix Jac_waist_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
    model.iDyn3_model.getJacobian( waist_index, Jac_waist_mix, false ) ;  

    yarp::sig::Matrix Jac_waist_body(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    model.iDyn3_model.getJacobian( waist_index, Jac_waist_body, true ) ; 
    

    
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
//    std::cout << " Jac_r_hand_upper_right_body = " <<  std::endl << Jac_r_hand_upper_right_body.submatrix(0,5,0,5).toString() << std::endl;    */
    
    
  /*  // yarp::sig::Matrix Temp_rocchi_4 = model.iDyn3_model.getPosition(r_hand_upper_right_index, waist_index) ;
    //std::cout <<"Temp_rocchi_4 = " <<  std::endl<<  Temp_rocchi_4.toString()<<  std::endl;                          
    */
    
   // IMU_LINK  
  //  std::cout << " -----------------------------------------------" <<   std::endl;     
    int imu_link_index = model.iDyn3_model.getLinkIndex("imu_link") ;
    
//    yarp::sig::Matrix Jac_imu_mix(robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;    
//    model.iDyn3_model.getJacobian( imu_link_index, Jac_imu_mix, false ) ;  
    yarp::sig::Matrix  T_w_imu  = model.iDyn3_model.getPosition( imu_link_index) ;    
    yarp::sig::Matrix  T_imu_w  = iHomogeneous(T_w_imu) ;  
    
    int a = 0 ; 
    if( robot.hasIMU()) a=1 ;
        
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
    
    yarp::sig::Matrix T_w_waist = model.iDyn3_model.getPosition( waist_index )  ;   
    yarp::sig::Matrix T_imu_waist = T_imu_w* T_w_waist  ;
    
    yarp::sig::Vector x_imu_waist(3) ;
    x_imu_waist[0] =  T_imu_waist[0][0];
    x_imu_waist[1] =  T_imu_waist[1][0];
    x_imu_waist[2] =  T_imu_waist[2][0];

    yarp::sig::Matrix Null_z_tr =  nullspaceProjection(z_imu_aw_matr.transposed()) ;
    yarp::sig::Vector x_imu_aw = Null_z_tr*x_imu_waist  ;

    yarp::sig::Vector y_imu_aw(3) ;
    yarp::sig::Matrix R_imu_aw(3,3) ;    
    
    if (norm(x_imu_aw)>0.01)
    {
    x_imu_aw = x_imu_aw/norm(x_imu_aw) ;   
   // y_imu_aw = cross(z_imu_aw, x_imu_aw  );   
    
 /*   R_imu_aw[0][0] = x_imu_aw[0] ;
    R_imu_aw[1][0] = x_imu_aw[1] ;
    R_imu_aw[2][0] = x_imu_aw[2] ;

    R_imu_aw[0][1] = y_imu_aw[0] ;
    R_imu_aw[1][1] = y_imu_aw[1] ;
    R_imu_aw[2][1] = y_imu_aw[2] ;

    R_imu_aw[0][2] = z_imu_aw[0] ;
    R_imu_aw[1][2] = z_imu_aw[1] ;
    R_imu_aw[2][2] = z_imu_aw[2] ; */
    }
    else   {   
    x_imu_aw[0] = Null_z_tr[0][0] ; 
    x_imu_aw[1] = Null_z_tr[1][0] ; 
    x_imu_aw[2] = Null_z_tr[2][0] ; 
         
    x_imu_aw = x_imu_aw/norm(x_imu_aw) ;     
    }  
    y_imu_aw = cross(z_imu_aw, x_imu_aw  );   
    
    R_imu_aw[0][0] = x_imu_aw[0] ;
    R_imu_aw[1][0] = x_imu_aw[1] ;
    R_imu_aw[2][0] = x_imu_aw[2] ;

    R_imu_aw[0][1] = y_imu_aw[0] ;
    R_imu_aw[1][1] = y_imu_aw[1] ;
    R_imu_aw[2][1] = y_imu_aw[2] ;

    R_imu_aw[0][2] = z_imu_aw[0] ;
    R_imu_aw[1][2] = z_imu_aw[1] ;
    R_imu_aw[2][2] = z_imu_aw[2] ;

    yarp::sig::Vector d_imu_aw(3, 0.0) ;     
    
    yarp::sig::Matrix T_imu_aw = Homogeneous( R_imu_aw, d_imu_aw) ;
    yarp::sig::Matrix T_aw_imu = iHomogeneous(T_imu_aw) ;    

    yarp::sig::Matrix T_w_aw = T_w_imu * T_imu_aw ;
    yarp::sig::Matrix T_aw_w = iHomogeneous(T_w_aw) ;    
    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//

    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//    
    // Computing Jacobian respect to the the Auxiliary World {AW}
   
    
    yarp::sig::Matrix T_R_imu_w = T_imu_w;
    
    T_R_imu_w[0][3] = 0;
    T_R_imu_w[1][3] = 0;
    T_R_imu_w[2][3] = 0;
    
 //   yarp::sig::Matrix Jac_imu_body = Adjoint( T_R_imu_w ) * Jac_imu_mix ; 
//    yarp::sig::Matrix Jac_spa_temp = Adjoint( T_aw_imu ) * Jac_imu_body  ; 

    yarp::sig::Matrix Eye_6(6,6) ;
    Eye_6.eye() ;
    
 //   Jac_spa_temp.setSubmatrix(Eye_6,0,0) ;
 //   yarp::sig::Matrix Jac_aw_imu =  Jac_spa_temp ; // Spatial Jacobian of the IMU link
    //
    // Computing spatial Jacobian ( i.e. in frame {AW} ) for all the contacts, setting also the first block to identity
    
    yarp::sig::Matrix Jac_l_foot_upper_left_link_mix(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_upper_right_link_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_left_link_mix(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_l_foot_lower_right_link_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;

    yarp::sig::Matrix Jac_r_foot_upper_left_link_mix(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_upper_right_link_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_left_link_mix(  robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ; 
    yarp::sig::Matrix Jac_r_foot_lower_right_link_mix( robot.getNumberOfJoints(), ( robot.getNumberOfJoints() + 6 ) ) ;
    
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
    
    T_R_l_foot_upper_left_link_index_w.setSubmatrix(  d_zero_31,0,3) ;
    T_R_l_foot_upper_right_link_index_w.setSubmatrix( d_zero_31,0,3) ;
    T_R_l_foot_lower_left_link_index_w.setSubmatrix(  d_zero_31,0,3) ;
    T_R_l_foot_lower_right_link_index_w.setSubmatrix( d_zero_31,0,3) ;

    T_R_r_foot_upper_left_link_index_w.setSubmatrix(  d_zero_31,0,3) ;
    T_R_r_foot_upper_right_link_index_w.setSubmatrix( d_zero_31,0,3) ;
    T_R_r_foot_lower_left_link_index_w.setSubmatrix(  d_zero_31,0,3) ;
    T_R_r_foot_lower_right_link_index_w.setSubmatrix( d_zero_31,0,3) ;

    yarp::sig::Matrix Jac_l_foot_upper_left_link_body_temp  = Adjoint( T_R_l_foot_upper_left_link_index_w  ) * Jac_l_foot_upper_left_link_mix  ; //These 'body' Jacobians are temporary because of the first six columns
    yarp::sig::Matrix Jac_l_foot_upper_right_link_body_temp = Adjoint( T_R_l_foot_upper_right_link_index_w ) * Jac_l_foot_upper_right_link_mix ; 
    yarp::sig::Matrix Jac_l_foot_lower_left_link_body_temp  = Adjoint( T_R_l_foot_lower_left_link_index_w  ) * Jac_l_foot_lower_left_link_mix  ; 
    yarp::sig::Matrix Jac_l_foot_lower_right_link_body_temp = Adjoint( T_R_l_foot_lower_right_link_index_w ) * Jac_l_foot_lower_right_link_mix ;

    yarp::sig::Matrix Jac_r_foot_upper_left_link_body_temp  = Adjoint( T_R_r_foot_upper_left_link_index_w  ) * Jac_r_foot_upper_left_link_mix  ; 
    yarp::sig::Matrix Jac_r_foot_upper_right_link_body_temp = Adjoint( T_R_r_foot_upper_right_link_index_w ) * Jac_r_foot_upper_right_link_mix ; 
    yarp::sig::Matrix Jac_r_foot_lower_left_link_body_temp  = Adjoint( T_R_r_foot_lower_left_link_index_w  ) * Jac_r_foot_lower_left_link_mix  ; 
    yarp::sig::Matrix Jac_r_foot_lower_right_link_body_temp = Adjoint( T_R_r_foot_lower_right_link_index_w ) * Jac_r_foot_lower_right_link_mix ;
        
    yarp::sig::Matrix Jac_aw_l_foot_upper_left_link  = Adjoint( T_aw_w*T_w_l_foot_upper_left_link_index  ) * Jac_l_foot_upper_left_link_body_temp ;
    yarp::sig::Matrix Jac_aw_l_foot_upper_right_link = Adjoint( T_aw_w*T_w_l_foot_upper_right_link_index ) * Jac_l_foot_upper_right_link_body_temp ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_left_link  = Adjoint( T_aw_w*T_w_l_foot_lower_left_link_index  ) * Jac_l_foot_lower_left_link_body_temp ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_right_link = Adjoint( T_aw_w*T_w_l_foot_lower_right_link_index ) * Jac_l_foot_lower_right_link_body_temp ;

    yarp::sig::Matrix Jac_aw_r_foot_upper_left_link  = Adjoint( T_aw_w*T_w_r_foot_upper_left_link_index  ) * Jac_r_foot_upper_left_link_body_temp ;
    yarp::sig::Matrix Jac_aw_r_foot_upper_right_link = Adjoint( T_aw_w*T_w_r_foot_upper_right_link_index ) * Jac_r_foot_upper_right_link_body_temp ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_left_link  = Adjoint( T_aw_w*T_w_r_foot_lower_left_link_index  ) * Jac_r_foot_lower_left_link_body_temp ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_right_link = Adjoint( T_aw_w*T_w_r_foot_lower_right_link_index ) * Jac_r_foot_lower_right_link_body_temp ;
    
    Jac_aw_l_foot_upper_left_link.setSubmatrix(  Eye_6,0,0 ) ;  // These are spatial Jacobians in {AW}
    Jac_aw_l_foot_upper_right_link.setSubmatrix( Eye_6,0,0 ) ;
    Jac_aw_l_foot_lower_left_link.setSubmatrix(  Eye_6,0,0 ) ;
    Jac_aw_l_foot_lower_right_link.setSubmatrix( Eye_6,0,0 ) ;

    Jac_aw_r_foot_upper_left_link.setSubmatrix(  Eye_6,0,0 ) ;
    Jac_aw_r_foot_upper_right_link.setSubmatrix( Eye_6,0,0 ) ;
    Jac_aw_r_foot_lower_left_link.setSubmatrix(  Eye_6,0,0 ) ;
    Jac_aw_r_foot_lower_right_link.setSubmatrix( Eye_6,0,0 ) ;
    
  /*  yarp::sig::Matrix Jac_aw_l_foot_upper_left_link  =  Jac_aw_l_foot_upper_left_link_temp  ; 
    yarp::sig::Matrix Jac_aw_l_foot_upper_right_link =  Jac_aw_l_foot_upper_right_link_temp ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_left_link  =  Jac_aw_l_foot_lower_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_l_foot_lower_right_link =  Jac_aw_l_foot_lower_right_link_temp ;

    yarp::sig::Matrix Jac_aw_r_foot_upper_left_link  =  Jac_aw_r_foot_upper_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_r_foot_upper_right_link =  Jac_aw_r_foot_upper_right_link_temp ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_left_link  =  Jac_aw_r_foot_lower_left_link_temp  ;
    yarp::sig::Matrix Jac_aw_r_foot_lower_right_link =  Jac_aw_r_foot_lower_right_link_temp ;  */
  
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

   yarp::sig::Matrix Jac_complete( 8*Jac_aw_l_foot_upper_left_sel.rows(), ( robot.getNumberOfJoints() + 6 )  ) ;
    
    Jac_complete.setSubmatrix( Jac_aw_l_foot_upper_left_sel  , 0 , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_l_foot_upper_right_sel , Jac_aw_l_foot_upper_right_sel.rows()    , 0 ) ;    
    Jac_complete.setSubmatrix( Jac_aw_l_foot_lower_left_sel  , 2*Jac_aw_l_foot_lower_left_sel.rows()  , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_l_foot_lower_right_sel , 3*Jac_aw_l_foot_lower_right_sel.rows()  , 0 ) ;
    
    Jac_complete.setSubmatrix( Jac_aw_r_foot_upper_left_sel  , 4*Jac_aw_r_foot_upper_left_sel.rows()   , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_r_foot_upper_right_sel , 5*Jac_aw_r_foot_upper_right_sel.rows()   , 0 ) ;    
    Jac_complete.setSubmatrix( Jac_aw_r_foot_lower_left_sel  , 6*Jac_aw_r_foot_lower_left_sel.rows()   , 0 ) ;
    Jac_complete.setSubmatrix( Jac_aw_r_foot_lower_right_sel , 7*Jac_aw_r_foot_lower_right_sel.rows()   , 0 ) ;

   
 //---------------------------------------------------------------------------//


    
 //---------------------------------------------------------------------------//
//---------------------------------------------------------------------------//  
   
    // std::cout << "  ---------------------------------------- "  << std::endl ;     
    //---------------------------------------------------------------------------//
    //---------------------------------------------------------------------------//
    // Start Seting FLMM     
       
    int size_fc = fc_l_contacts_to_world.length() +  fc_r_contacts_to_world  .length() ; 
    int size_q  = robot.getNumberOfJoints() ;
    
    yarp::sig::Matrix Eye_fc(size_fc, size_fc) ;
    Eye_fc.eye() ;
    yarp::sig::Matrix Eye_q(size_q, size_q) ;
    Eye_q.eye() ;
    yarp::sig::Matrix Eye_tau = Eye_q ;
    
    yarp::sig::Matrix Zeros_fc_q(size_fc, size_q) ;
    yarp::sig::Matrix Zeros_q_fc(size_q, size_fc) ;
    yarp::sig::Matrix Zeros_q_q(size_q, size_q) ;
    yarp::sig::Matrix Zeros_6_q( 6 , size_q) ;
    yarp::sig::Matrix Zeros_q_6(size_q, 6 ) ;
        
    yarp::sig::Matrix Kc(size_fc, size_fc) ;
    Kc.eye() ;
    Kc = 1E7*Kc ;
    yarp::sig::Matrix Kq = getKq() ;
    
    yarp::sig::Matrix Stance_c_tranposed =  Jac_complete.submatrix(0, Jac_complete.rows()-1 , 0, 5) ;
    yarp::sig::Matrix Stance_c( 6 , size_fc) ;
    Stance_c  =  Stance_c_tranposed.transposed()  ;
    
    yarp::sig::Matrix Jacob_c( size_fc , size_q) ;
    Jacob_c = Jac_complete.submatrix(0, Jac_complete.rows()-1 , 6, Jac_complete.cols()-1) ;

   //-------------------------------------------------------------------------------------------------------------// 
  
   //-------------------------------------------------------------------------------------------------------------// 

    /*yarp::sig::Matrix U_j( size_fc , size_q) ;
    U_j.zero();
    yarp::sig::Matrix U_s( 6 , size_fc) ;  
    U_s.zero();
    yarp::sig::Matrix Q_j( size_fc , size_q) ;
    Q_j.zero();
    yarp::sig::Matrix Q_s( 6 , size_fc) ;     
    Q_s.zero();  */
    
    yarp::sig::Matrix U_j( size_q , 6 ) ;
    U_j.zero();
    yarp::sig::Matrix U_s( 6 , 6) ;  
    U_s.zero();
    yarp::sig::Matrix Q_j( size_q , size_q) ;
    Q_j.zero();
    yarp::sig::Matrix Q_s( 6 , size_q) ;     
    Q_s.zero();
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
    FLMM.setSubmatrix( Zeros_q_6   , size_fc +size_q +6  , size_fc+size_q     ) ;
    FLMM.setSubmatrix( Kq          , size_fc +size_q +6  , size_fc+size_q+6   ) ;
    FLMM.setSubmatrix( -1.0*Kq     , size_fc +size_q +6  , size_fc+2*size_q+6 ) ;
 
    yarp::sig::Matrix Phi_star_i = FLMM.submatrix(0, FLMM.rows()-1, 0,   size_fc + 2*size_q + 6-1     )  ;

    yarp::sig::Matrix cFLMM =  yarp::math::luinv(Phi_star_i)*FLMM  ;
       
    yarp::sig::Matrix Phi_i = cFLMM.submatrix(0, cFLMM.rows()-1,     0    ,             size_fc + 2*size_q + 6-1     )  ;    
    yarp::sig::Matrix Phi_d = cFLMM.submatrix(0, cFLMM.rows()-1, size_fc + 2*size_q + 6 ,   cFLMM.cols()-1     )  ;

    yarp::sig::Matrix R_f = Phi_d.submatrix(0, size_fc-1 ,0  ,Phi_d.cols()-1 ) ;    

    //------------------------------------------------------------------------------//
    yarp::sig::Vector fc_actual_to_world( fc_l_contacts_to_world.length() + fc_r_contacts_to_world.length())  ;      
    
    fc_actual_to_world.setSubvector( 0 , fc_l_contacts_to_world ) ;
    fc_actual_to_world.setSubvector( fc_l_contacts_to_world.length() , fc_r_contacts_to_world ) ;
    
    // desired contact force definition
    yarp::sig::Vector fc_desired_to_world( fc_l_contacts_to_world.length() + fc_l_contacts_to_world.length())  ;

    int mg =  295 ; // [N]
    
    fc_desired_to_world[0] = 0  ;
    fc_desired_to_world[1] = 0  ;
    fc_desired_to_world[2] = - mg/8 ;
    fc_desired_to_world[3] = 0  ;
    fc_desired_to_world[4] = 0  ;
    fc_desired_to_world[5] = - mg/8  ;
    fc_desired_to_world[6] = 0 ;
    fc_desired_to_world[7] = 0 ;
    fc_desired_to_world[8] = - mg/8  ;
    fc_desired_to_world[9] =  0 ;
    fc_desired_to_world[10] = 0 ;
    fc_desired_to_world[11] = - mg/8  ;

    fc_desired_to_world[12] = 0  ;
    fc_desired_to_world[13] = 0  ;
    fc_desired_to_world[14] = - mg/8 ;
    fc_desired_to_world[15] = 0  ;
    fc_desired_to_world[16] = 0  ;
    fc_desired_to_world[17] = - mg/8  ;
    fc_desired_to_world[18] = 0 ;
    fc_desired_to_world[19] = 0 ;
    fc_desired_to_world[20] = - mg/8  ;
    fc_desired_to_world[21] = 0 ;
    fc_desired_to_world[22] = 0 ;
    fc_desired_to_world[23] = - mg/8  ;    
  
    yarp::sig::Vector d_fc_desired_to_world = fc_desired_to_world - fc_actual_to_world ;
    
    yarp::sig::Vector d_q_motor_desired = -1.0* pinv( R_f , 1E-6 ) * d_fc_desired_to_world ; 

    yarp::sig::Vector fc_teor   = d_fc_desired_to_world - 1.0*R_f *d_q_motor_desired ;
    yarp::sig::Vector d_fc_teor = d_fc_desired_to_world - fc_teor ;
    
    std::cout << " fc_actual_to_world  = " <<  std::endl << fc_actual_to_world.toString() << std::endl; 
    std::cout << " fc_desired_to_world  = " <<  std::endl << fc_desired_to_world.toString() << std::endl;    
    std::cout << " norm( d_fc_desired_to_world ) = " <<  std::endl << norm( d_fc_desired_to_world ) << std::endl; 
 
 //---------------------------------------------------------------------------------------------------------//  
//   writing data on a file
    //std::ofstream r_ankle ;
    double err = norm( d_fc_desired_to_world )  ;
    //r_ankle.open ("r_ankle.txt");
    std::ofstream err_cl ( "err.m", std::ios::app );
    if( err_cl.is_open() )
    err_cl <<  err << std::endl;
 
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
    q_ref_ToMove_right_arm[0] += .0 ;  
    
    robot.fromRobotToIdyn( q_ref_ToMove_right_arm ,
                           q_ref_ToMove_left_arm  ,
                           q_ref_ToMove_torso  ,
                           q_ref_ToMove_right_leg ,
                           q_ref_ToMove_left_leg  ,
                           q_ref_ToMove );    
     
     q_ref_ToMove = q_ref_ToMove   + (1.0/1.0 )*d_q_motor_desired  ; // to stabilize... q_ref_ToMove  + d_q_motor_desired ;
       
     robot.move(q_ref_ToMove);  // q_ref_ToMove
 
     // robot.left_arm.move(q_ref_ToMove_left_arm);
    
     //---------------------------------------------------------------------------------------------------
 
     double eps = std::numeric_limits<double>::epsilon();
     double h = sqrt(eps) ;
 
     yarp::sig::Matrix T_aw_l_foot_upper_left_link  = T_aw_w*T_w_l_foot_upper_left_link  ;
     yarp::sig::Matrix T_aw_l_foot_upper_right_link = T_aw_w*T_w_l_foot_upper_right_link ;
     yarp::sig::Matrix T_aw_l_foot_lower_left_link  = T_aw_w*T_w_l_foot_lower_left_link  ;
     yarp::sig::Matrix T_aw_l_foot_lower_right_link = T_aw_w*T_w_l_foot_lower_right_link ;

     yarp::sig::Matrix T_aw_r_foot_upper_left_link  = T_aw_w*T_w_r_foot_upper_left_link  ;
     yarp::sig::Matrix T_aw_r_foot_upper_right_link = T_aw_w*T_w_r_foot_upper_right_link ;
     yarp::sig::Matrix T_aw_r_foot_lower_left_link  = T_aw_w*T_w_r_foot_lower_left_link  ;
     yarp::sig::Matrix T_aw_r_foot_lower_right_link = T_aw_w*T_w_r_foot_lower_right_link ;
     
     yarp::sig::Matrix T_aw_l_c1_0 = T_aw_l_foot_upper_left_link  ;
     yarp::sig::Matrix T_aw_l_c2_0 = T_aw_l_foot_upper_right_link ;
     yarp::sig::Matrix T_aw_l_c3_0 = T_aw_l_foot_lower_left_link  ;
     yarp::sig::Matrix T_aw_l_c4_0 = T_aw_l_foot_lower_right_link ;

     yarp::sig::Matrix T_aw_r_c1_0 = T_aw_r_foot_upper_left_link  ;
     yarp::sig::Matrix T_aw_r_c2_0 = T_aw_r_foot_upper_right_link ;
     yarp::sig::Matrix T_aw_r_c3_0 = T_aw_r_foot_lower_left_link  ;
     yarp::sig::Matrix T_aw_r_c4_0 = T_aw_r_foot_lower_right_link ;
     
     yarp::sig::Matrix T_l_c1_aw_0 = iHomogeneous( T_aw_l_c1_0 )  ;
     yarp::sig::Matrix T_l_c2_aw_0 = iHomogeneous( T_aw_l_c2_0 )  ;
     yarp::sig::Matrix T_l_c3_aw_0 = iHomogeneous( T_aw_l_c3_0 )   ;
     yarp::sig::Matrix T_l_c4_aw_0 = iHomogeneous( T_aw_l_c4_0 )  ;

     yarp::sig::Matrix T_r_c1_aw_0 = iHomogeneous( T_aw_r_c1_0 )   ;
     yarp::sig::Matrix T_r_c2_aw_0 = iHomogeneous( T_aw_r_c2_0 )  ;
     yarp::sig::Matrix T_r_c3_aw_0 = iHomogeneous( T_aw_r_c3_0 )   ;
     yarp::sig::Matrix T_r_c4_aw_0 = iHomogeneous( T_aw_r_c4_0 )  ;     
     
     yarp::sig::Vector fc_l_c1(3) ;  //
     yarp::sig::Vector fc_l_c2(3) ;  // = fc_l_foot_upper_right_link ;
     yarp::sig::Vector fc_l_c3(3) ;  // = fc_l_foot_lower_left_link  ;
     yarp::sig::Vector fc_l_c4(3) ;  // = fc_l_foot_lower_right_link ;

     yarp::sig::Vector fc_r_c1(3) ;  // = fc_r_foot_upper_left_link  ;
     yarp::sig::Vector fc_r_c2(3) ;  // = fc_r_foot_upper_right_link ;
     yarp::sig::Vector fc_r_c3(3) ;  // = fc_r_foot_lower_left_link  ;
     yarp::sig::Vector fc_r_c4(3) ;  // = fc_r_foot_lower_right_link ;  
     
     fc_l_c1.setSubvector( 0, fc_l_contacts_to_world.subVector(0,2) )   ;
     fc_l_c2.setSubvector( 0, fc_l_contacts_to_world.subVector(3,5) )   ;
     fc_l_c3.setSubvector( 0, fc_l_contacts_to_world.subVector(6,8) )   ;
     fc_l_c4.setSubvector( 0, fc_l_contacts_to_world.subVector(9,11) )  ;

     fc_r_c1.setSubvector( 0, fc_r_contacts_to_world.subVector(0,2) )   ;
     fc_r_c2.setSubvector( 0, fc_r_contacts_to_world.subVector(3,5) )   ;
     fc_r_c3.setSubvector( 0, fc_r_contacts_to_world.subVector(6,8) )   ;
     fc_r_c4.setSubvector( 0, fc_r_contacts_to_world.subVector(9,11) )  ;

     //-------------------------------------------------------------------------------------------------------------------
     // REMIND NOTATION
     //  Jac_aw_l_foot_upper_left_link       -> spatial long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_aw_l_foot_upper_left_link_body  -> body long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_spa_0                      -> spatial short in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_body_0                     -> spatial short in the initial configuration (not incemented for derivatives)
  
     // Spatial Jacobian Short   
     yarp::sig::Matrix Jac_l_c1_spa_0 = Jac_aw_l_foot_upper_left_link.submatrix( 0,5 , 6, Jac_aw_l_foot_upper_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_l_c2_spa_0 = Jac_aw_l_foot_upper_right_link.submatrix(0,5 , 6, Jac_aw_l_foot_upper_right_link.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c3_spa_0 = Jac_aw_l_foot_lower_left_link.submatrix( 0,5 , 6, Jac_aw_l_foot_lower_left_link.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c4_spa_0 = Jac_aw_l_foot_lower_right_link.submatrix(0,5 , 6, Jac_aw_l_foot_lower_right_link.cols() -1  )  ;
    
     yarp::sig::Matrix Jac_r_c1_spa_0 = Jac_aw_r_foot_upper_left_link.submatrix( 0,5 , 6, Jac_aw_r_foot_upper_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c2_spa_0 = Jac_aw_r_foot_upper_right_link.submatrix(0,5 , 6, Jac_aw_r_foot_upper_right_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c3_spa_0 = Jac_aw_r_foot_lower_left_link.submatrix( 0,5 , 6, Jac_aw_r_foot_lower_left_link.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c4_spa_0 = Jac_aw_r_foot_lower_right_link.submatrix(0,5 , 6, Jac_aw_r_foot_lower_right_link.cols() -1  )  ; // Spacial Jacobians Reduced
     
     // Body Jacobian Short   
     yarp::sig::Matrix Jac_l_c1_body_0 = Jac_aw_l_foot_upper_left_link_body.submatrix( 0,5 , 6, Jac_aw_l_foot_upper_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_l_c2_body_0 = Jac_aw_l_foot_upper_right_link_body.submatrix(0,5 , 6, Jac_aw_l_foot_upper_right_link_body.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c3_body_0 = Jac_aw_l_foot_lower_left_link_body.submatrix( 0,5 , 6, Jac_aw_l_foot_lower_left_link_body.cols() -1  )  ;
     yarp::sig::Matrix Jac_l_c4_body_0 = Jac_aw_l_foot_lower_right_link_body.submatrix(0,5 , 6, Jac_aw_l_foot_lower_right_link_body.cols() -1  )  ;
    
     yarp::sig::Matrix Jac_r_c1_body_0 = Jac_aw_r_foot_upper_left_link_body.submatrix( 0,5 , 6, Jac_aw_r_foot_upper_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c2_body_0 = Jac_aw_r_foot_upper_right_link_body.submatrix(0,5 , 6, Jac_aw_r_foot_upper_right_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c3_body_0 = Jac_aw_r_foot_lower_left_link_body.submatrix( 0,5 , 6, Jac_aw_r_foot_lower_left_link_body.cols() -1  )  ; // Spacial Jacobians Reduced
     yarp::sig::Matrix Jac_r_c4_body_0 = Jac_aw_r_foot_lower_right_link_body.submatrix(0,5 , 6, Jac_aw_r_foot_lower_right_link_body.cols() -1  )  ; // Spacial Jacobians Reduced         
 
     //------------------------------------------------------------------------------------------
     // Computing U_s
     yarp::sig::Vector xi_1(6, 0.0)  ;
     yarp::sig::Vector xi_2(6, 0.0)  ;
     yarp::sig::Vector xi_3(6, 0.0)  ;
     yarp::sig::Vector xi_4(6, 0.0)  ;
     yarp::sig::Vector xi_5(6, 0.0)  ;
     yarp::sig::Vector xi_6(6, 0.0)  ;
     
     xi_1[0] = 1 ;
     xi_2[1] = 1 ;
     xi_3[2] = 1 ;
     xi_4[3] = 1 ;
     xi_5[4] = 1 ;
     xi_6[5] = 1 ;

     yarp::sig::Vector u_curr( 6, 0.0 )  ;
     yarp::sig::Vector u_incr(6) ;
     u_incr = u_curr ;
     u_incr[0] += h ;
 
 //------------------------------------------------------------------------------------------------------    
     // 'For' loop for computing  U_s 
     
     yarp::sig::Matrix U_s_l_c1(6 , 6) ;
     yarp::sig::Matrix U_s_l_c2(6 , 6) ;
     yarp::sig::Matrix U_s_l_c3(6 , 6) ;
     yarp::sig::Matrix U_s_l_c4(6 , 6) ;

     yarp::sig::Matrix U_s_r_c1(6 , 6) ;
     yarp::sig::Matrix U_s_r_c2(6 , 6) ;
     yarp::sig::Matrix U_s_r_c3(6 , 6) ;
     yarp::sig::Matrix U_s_r_c4(6 , 6) ;

     U_s_l_c1.zero();
     U_s_l_c2.zero();
     U_s_l_c3.zero();
     U_s_l_c4.zero();

     U_s_r_c1.zero();
     U_s_r_c2.zero();
     U_s_r_c3.zero();
     U_s_r_c4.zero();
     
     yarp::sig::Vector U_s_l_c1_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c2_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c3_i(6, 0.0) ;
     yarp::sig::Vector U_s_l_c4_i(6, 0.0) ;

     yarp::sig::Vector U_s_r_c1_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c2_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c3_i(6, 0.0) ;
     yarp::sig::Vector U_s_r_c4_i(6, 0.0) ;
     
     yarp::sig::Matrix d_adj_fix_aw_u_i(6,6) ;
     
     
     //--------------------------------------
     yarp::sig::Matrix Jac_COM_w_0( 6 , robot.getNumberOfJoints()  + 6 ) ; 
     model.iDyn3_model.getCOMJacobian( Jac_COM_w_0 ) ;  
     yarp::sig::Vector d_w_COM_0(3) ;
     yarp::sig::Vector zero_3(3, 0.0) ;

     robot.idynutils.updateiDyn3Model( q_current, true ); //update model first  
     
     d_w_COM_0 = model.iDyn3_model.getCOM()  ;               
     yarp::sig::Matrix T_w_COM_0 = Homogeneous(Eye_3, d_w_COM_0 ) ;
     yarp::sig::Matrix T_aw_COM_0 = T_aw_w *  T_w_COM_0 ;
     yarp::sig::Vector d_aw_COM_0 = getTrasl(T_aw_COM_0) ;
     yarp::sig::Vector g_dir_aw(3, 0.0) ; //direction of the gravity vector expressed in {AW}
     g_dir_aw(2) = -1 ;
     
     yarp::sig::Matrix  T_aw_w_0 = T_aw_w ;
           // robot.idynutils.updateiDyn3Model( q_current, true ); 
     yarp::sig::Matrix Jac_COM_aw_temp = Adjoint( Homogeneous( getRot( T_aw_w_0 ) , zero_3) )*Jac_COM_w_0 ;    
     yarp::sig::Matrix Jac_COM_aw_0 = Jac_COM_aw_temp.submatrix( 0, 2, 6, Jac_COM_aw_temp.cols()-1 ) ;      
     
     yarp::sig::Matrix T_fix_COM_incr(4,4) ; // = g_fix_aw_u_i *  T_aw_COM_0 ;
     yarp::sig::Vector d_fix_COM_incr(3)  ;  // = getTrasl(T_fix_COM_incr) ;
     yarp::sig::Vector d_com_i(3) ;
     
     yarp::sig::Matrix U_sg(6,6)  ;     
     yarp::sig::Matrix U_sg_1(3,6) ;
     U_sg_1.zero(); 
     yarp::sig::Matrix U_sg_2(3,6)  ;

     yarp::sig::Matrix T_fix_w_incr(4,4) ;
     yarp::sig::Matrix U_jg(robot.getNumberOfJoints() ,6)  ;     
  //   yarp::sig::Matrix U_jg_2(robot.getNumberOfJoints() ,6)  ;     
     yarp::sig::Vector temp_adj(robot.getNumberOfJoints() + 6) ;
     yarp::sig::Vector temp_adj_2(robot.getNumberOfJoints() ) ;

     yarp::sig::Vector U_jg_i(robot.getNumberOfJoints()) ;
     
     yarp::sig::Matrix Jac_COM_fix_temp( 6, Jac_COM_fix_temp.cols() ) ;
     yarp::sig::Matrix Jac_COM_fix_incr( 6, robot.getNumberOfJoints()) ;// = Jac_COM_aw_temp.submatrix( 0, 2, 6, Jac_COM_fix_temp.cols()-1 ) ;

     for ( int i = 0  ; i<6 ; i++ ) 
     {
      u_incr = u_curr ;
      u_incr[i] += h ;
      
      yarp::sig::Matrix g_fix_aw_u_i = twistexp(xi_1, u_incr[0]) * // this is the only point in which it is relevant the fact that
				       twistexp(xi_2, u_incr[1]) * // the frame {AW} is attached the world and can move a frame
				       twistexp(xi_3, u_incr[2]) * // {AW'} which, in the initial config., is coincident with {AW}
				       twistexp(xi_4, u_incr[3]) * // here {AW} is moving with respect to {fix}
				       twistexp(xi_5, u_incr[4]) *
				       twistexp(xi_6, u_incr[5]) ;
     d_adj_fix_aw_u_i = ( ( Adjoint_MT( g_fix_aw_u_i ) - Eye_6)/h ) ;
     U_s_l_c1_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c1_0 ) * B_select.transposed() * fc_l_c1 ;    
     U_s_l_c2_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c2_0 ) * B_select.transposed() * fc_l_c2 ;
     U_s_l_c3_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c3_0 ) * B_select.transposed() * fc_l_c3 ;
     U_s_l_c4_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_l_c4_0 ) * B_select.transposed() * fc_l_c4 ;

     U_s_r_c1_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c1_0 ) * B_select.transposed() * fc_r_c1 ;
     U_s_r_c2_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c2_0 ) * B_select.transposed() * fc_r_c2 ;
     U_s_r_c3_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c3_0 ) * B_select.transposed() * fc_r_c3 ;
     U_s_r_c4_i = d_adj_fix_aw_u_i * Adjoint_MT( T_aw_r_c4_0 ) * B_select.transposed() * fc_r_c4 ;

     U_s_l_c1.setCol( i, U_s_l_c1_i )  ;      
     U_s_l_c2.setCol( i, U_s_l_c2_i )  ;
     U_s_l_c3.setCol( i, U_s_l_c3_i )  ;
     U_s_l_c4.setCol( i, U_s_l_c4_i )  ;

     U_s_r_c1.setCol( i, U_s_r_c1_i )  ;
     U_s_r_c2.setCol( i, U_s_r_c2_i )  ;
     U_s_r_c3.setCol( i, U_s_r_c3_i )  ;
     U_s_r_c4.setCol( i, U_s_r_c4_i )  ;
     
     //------------------------------------------------
     // COM Part
     T_fix_COM_incr = g_fix_aw_u_i *  T_aw_COM_0 ;
     d_fix_COM_incr = getTrasl(T_fix_COM_incr) ;
     d_com_i = (d_fix_COM_incr - d_aw_COM_0)/h ;
     U_sg_2.setCol( i, d_com_i )  ;
     //-----------------------------------------------
     T_fix_w_incr = g_fix_aw_u_i * T_aw_w_0 ;    

     temp_adj =  ( ( (Adjoint(Homogeneous( getRot( T_fix_w_incr), zero_3) ) - 
                                         Adjoint( Homogeneous( getRot(T_aw_w_0), zero_3) ))/h )  *Jac_COM_w_0).transposed() 
					 *B_select.transposed()  *mg*(-1.0*g_dir_aw)    ;
    
     temp_adj_2 = temp_adj.subVector( 6 ,  temp_adj.length()-1  ) ;
 //    std::cout << "temp_adj =  " << std::endl << temp_adj.length() << std::endl;  
 //    std::cout << "temp_adj_2 =  " << std::endl << temp_adj_2.toString() << std::endl;  
 //     std::cout << "temp_adj_2 =  " << std::endl << temp_adj_2.length() << std::endl;  

     U_jg.setCol(i, temp_adj_2) ;
   
     } ;
           //       std::cout << "U_jg =  " << std::endl << U_jg.toString() << std::endl;  
    // std::cout << "U_jg =  " << std::endl << U_jg.toString() << std::endl;  
     
     U_sg_2 = mg*crossProductMatrix(g_dir_aw)* U_sg_2 ;
     U_sg.setSubmatrix( U_sg_1, 0, 0) ;
     U_sg.setSubmatrix( U_sg_2, 3, 0) ;
     
     yarp::sig::Matrix U_s_l_tot(6 , 6) ;
     yarp::sig::Matrix U_s_r_tot(6 , 6) ;
     yarp::sig::Matrix U_s_lr(6 , 6) ;
     yarp::sig::Matrix U_s_tot(6 , 6) ;
     
     U_s_l_tot = U_s_l_c1 + U_s_l_c2 + U_s_l_c3 + U_s_l_c4 ;     
     U_s_r_tot = U_s_r_c1 + U_s_r_c2 + U_s_r_c3 + U_s_r_c4 ;
     U_s_lr = U_s_l_tot + U_s_r_tot  ; 
     U_s_tot = U_s_l_tot + U_s_r_tot + U_sg ;     

//     std::cout << "U_sg =  " << std::endl << U_sg.toString() << std::endl;       
//     std::cout << "U_s_lr =  " << std::endl << U_s_lr.toString() << std::endl;            
//     std::cout << "U_s_tot =  " << std::endl << U_s_tot.toString() << std::endl;       
     
//-------------------------------------------------------------------------------------------------------------

//     yarp::sig::Matrix U_s_r_tot(6 , 6) ;
//     U_s_r_tot = U_s_r_c1 + U_s_r_c2 + U_s_r_c3 + U_s_r_c4 ;
  //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s

     //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s_l     

     yarp::sig::Vector q_incr = q_current ; 
     
     yarp::sig::Matrix Q_s_l_c1(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c2(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c3(6,6 ) ;
     yarp::sig::Matrix Q_s_l_c4(6,6 ) ;
     Q_s_l_c1.zero();     
     Q_s_l_c2.zero();
     Q_s_l_c3.zero();
     Q_s_l_c4.zero();
     
     yarp::sig::Vector Q_s_l_c1_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c2_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c3_i(6, 0.0) ;
     yarp::sig::Vector Q_s_l_c4_i(6, 0.0) ;

     yarp::sig::Matrix T_imu_w_incr(4,4) ;     //  = iHomogeneous(T_w_imu_incr) ;
     yarp::sig::Matrix T_aw_w_incr(4,4) ;     //  = T_aw_imu * T_imu_incr_w ;

     T_imu_w_incr.zero();
     T_aw_w_incr.zero();

     yarp::sig::Matrix T_w_l_c1_incr(4,4) ;    
     yarp::sig::Matrix T_w_l_c2_incr(4,4) ;
     yarp::sig::Matrix T_w_l_c3_incr(4,4) ;
     yarp::sig::Matrix T_w_l_c4_incr(4,4) ;
     T_w_l_c1_incr.zero();
     T_w_l_c2_incr.zero();
     T_w_l_c3_incr.zero();
     T_w_l_c4_incr.zero();
  // 
     yarp::sig::Matrix T_aw_l_c1_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c2_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c3_incr(4,4) ;
     yarp::sig::Matrix T_aw_l_c4_incr(4,4) ;
     T_aw_l_c1_incr.zero();
     T_aw_l_c2_incr.zero();
     T_aw_l_c3_incr.zero();
     T_aw_l_c4_incr.zero();   
     
     for ( int i = 0  ; i<6 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  

      T_imu_w_incr  = model.iDyn3_model.getPosition( imu_link_index ,true ) ;    
      T_aw_w_incr   = T_aw_imu * T_imu_w_incr ;
     
      T_w_l_c1_incr = model.iDyn3_model.getPosition( l_foot_upper_left_link_index  )  ;    // l_c1 = l_foot_upper_left_link_index
      T_w_l_c2_incr = model.iDyn3_model.getPosition( l_foot_upper_right_link_index )  ;    // l_c2 = l_foot_upper_right_link_index
      T_w_l_c3_incr = model.iDyn3_model.getPosition( l_foot_lower_left_link_index  )  ;    // l_c3 = l_foot_lower_left_link_index
      T_w_l_c4_incr = model.iDyn3_model.getPosition( l_foot_lower_right_link_index )  ;    // l_c4 = l_foot_lower_right_link_index
  
      T_aw_l_c1_incr  = T_aw_w_incr * T_w_l_c1_incr ;  
      T_aw_l_c2_incr  = T_aw_w_incr * T_w_l_c2_incr ;  
      T_aw_l_c3_incr  = T_aw_w_incr * T_w_l_c3_incr ;  
      T_aw_l_c4_incr  = T_aw_w_incr * T_w_l_c4_incr ;  

      Q_s_l_c1_i = ( (Adjoint_MT(T_aw_l_c1_incr) - Adjoint_MT(T_aw_l_c1_0 )) /h )*B_select.transposed()*fc_l_c1  ;
      Q_s_l_c2_i = ( (Adjoint_MT(T_aw_l_c2_incr) - Adjoint_MT(T_aw_l_c2_0 )) /h )*B_select.transposed()*fc_l_c2  ;
      Q_s_l_c3_i = ( (Adjoint_MT(T_aw_l_c3_incr) - Adjoint_MT(T_aw_l_c3_0 )) /h )*B_select.transposed()*fc_l_c3  ;
      Q_s_l_c4_i = ( (Adjoint_MT(T_aw_l_c4_incr) - Adjoint_MT(T_aw_l_c4_0 )) /h )*B_select.transposed()*fc_l_c4  ;
         
      Q_s_l_c1.setCol( i ,  Q_s_l_c1_i ) ;
      Q_s_l_c2.setCol( i ,  Q_s_l_c2_i ) ;
      Q_s_l_c3.setCol( i ,  Q_s_l_c3_i ) ;
      Q_s_l_c4.setCol( i ,  Q_s_l_c4_i ) ;   
     } ;
      
   //  std::cout << "Q_s_l_c1 =  " << std::endl << Q_s_l_c1.toString() << std::endl; 
   //   std::cout << "Q_s_l_c2 =  " << std::endl << Q_s_l_c2.toString() << std::endl; 
   //  std::cout << "Q_s_l_c3 =  " << std::endl << Q_s_l_c3.toString() << std::endl; 
   //  std::cout << "Q_s_l_c4 =  " << std::endl << Q_s_l_c4.toString() << std::endl;  

     yarp::sig::Matrix Q_s_l = Q_s_l_c1 + Q_s_l_c2 + Q_s_l_c3 + Q_s_l_c4 ;
   //  std::cout << "Q_s_l =  " << std::endl << Q_s_l.toString() << std::endl; 

     //-------------------------------------------------------------------------------------------------------------------
     //  Computing Q_s_r     
     //-------------------------------------------------------------------
    
     yarp::sig::Matrix Q_s_r_c1(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c2(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c3(6,6 ) ;
     yarp::sig::Matrix Q_s_r_c4(6,6 ) ;
     Q_s_r_c1.zero();  
     Q_s_r_c2.zero();
     Q_s_r_c3.zero();    
     Q_s_r_c4.zero();
     
     yarp::sig::Vector Q_s_r_c1_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c2_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c3_i(6, 0.0) ;
     yarp::sig::Vector Q_s_r_c4_i(6, 0.0) ; 
           
     yarp::sig::Matrix T_w_r_c1_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c2_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c3_incr(4,4) ;
     yarp::sig::Matrix T_w_r_c4_incr(4,4) ;
     T_w_r_c1_incr.zero();     
     T_w_r_c2_incr.zero();
     T_w_r_c3_incr.zero();
     T_w_r_c4_incr.zero();
  // 
     yarp::sig::Matrix T_aw_r_c1_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c2_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c3_incr(4,4) ;
     yarp::sig::Matrix T_aw_r_c4_incr(4,4) ;
     T_aw_r_c1_incr.zero();
     T_aw_r_c2_incr.zero();
     T_aw_r_c3_incr.zero();  
     T_aw_r_c4_incr.zero();
     
     for ( int i = 6  ; i<12 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  

      T_imu_w_incr  = model.iDyn3_model.getPosition( imu_link_index ,true ) ;    
      T_aw_w_incr   = T_aw_imu * T_imu_w_incr ;
     
      T_w_r_c1_incr = model.iDyn3_model.getPosition( r_foot_upper_left_link_index  )  ;    // r_c1 = r_foot_upper_left_link_index
      T_w_r_c2_incr = model.iDyn3_model.getPosition( r_foot_upper_right_link_index )  ;    // r_c2 = r_foot_upper_right_link_index
      T_w_r_c3_incr = model.iDyn3_model.getPosition( r_foot_lower_left_link_index  )  ;    // r_c3 = r_foot_lower_left_link_index
      T_w_r_c4_incr = model.iDyn3_model.getPosition( r_foot_lower_right_link_index )  ;    // r_c4 = r_foot_lower_right_link_index
  
      T_aw_r_c1_incr  = T_aw_w_incr * T_w_r_c1_incr ;  
      T_aw_r_c2_incr  = T_aw_w_incr * T_w_r_c2_incr ;  
      T_aw_r_c3_incr  = T_aw_w_incr * T_w_r_c3_incr ;  
      T_aw_r_c4_incr  = T_aw_w_incr * T_w_r_c4_incr ;  

      Q_s_r_c1_i = ( (Adjoint_MT(T_aw_r_c1_incr) - Adjoint_MT( T_aw_r_c1_0 )) /h )*B_select.transposed()*fc_r_c1  ;
      Q_s_r_c2_i = ( (Adjoint_MT(T_aw_r_c2_incr) - Adjoint_MT( T_aw_r_c2_0 )) /h )*B_select.transposed()*fc_r_c2  ;
      Q_s_r_c3_i = ( (Adjoint_MT(T_aw_r_c3_incr) - Adjoint_MT( T_aw_r_c3_0 )) /h )*B_select.transposed()*fc_r_c3  ;
      Q_s_r_c4_i = ( (Adjoint_MT(T_aw_r_c4_incr) - Adjoint_MT( T_aw_r_c4_0 )) /h )*B_select.transposed()*fc_r_c4  ;
         
      Q_s_r_c1.setCol( i-6 ,  Q_s_r_c1_i ) ;
      Q_s_r_c2.setCol( i-6 ,  Q_s_r_c2_i ) ;
      Q_s_r_c3.setCol( i-6 ,  Q_s_r_c3_i ) ;
      Q_s_r_c4.setCol( i-6 ,  Q_s_r_c4_i ) ;   // QUI
     } ;
      
   //  std::cout << "Q_s_l_c1 =  " << std::endl << Q_s_l_c1.toString() << std::endl; 
   //  std::cout << "Q_s_l_c2 =  " << std::endl << Q_s_l_c2.toString() << std::endl; 
   //  std::cout << "Q_s_l_c3 =  " << std::endl << Q_s_l_c3.toString() << std::endl; 
   //  std::cout << "Q_s_l_c4 =  " << std::endl << Q_s_l_c4.toString() << std::endl;   

     yarp::sig::Matrix  Q_s_r = Q_s_r_c1 + Q_s_r_c2 + Q_s_r_c3 + Q_s_r_c4 ;
 //    std::cout << "Q_s_r =  " << std::endl << Q_s_r.toString() << std::endl; 
      
     yarp::sig::Matrix Q_s_lr( 6 , robot.getNumberOfJoints() ) ;
     
     Q_s_lr.setSubmatrix(Q_s_l , 0 , 0) ;
     Q_s_lr.setSubmatrix(Q_s_r , 0 , 6) ;

     yarp::sig::Matrix Q_sg(6,  robot.getNumberOfJoints() ) ;
     yarp::sig::Matrix Q_sg_1(3,  robot.getNumberOfJoints() )  ;
     yarp::sig::Matrix Q_sg_2 = mg*crossProductMatrix(g_dir_aw) * Jac_COM_aw_0    ;
     Q_sg.setSubmatrix(Q_sg_1, 0 , 0) ;
     Q_sg.setSubmatrix(Q_sg_2, 3 , 0) ; 

     yarp::sig::Matrix Q_s_tot = Q_s_lr + Q_sg   ;     

//     std::cout << "Q_s_lr =  " << std::endl << Q_s_lr.toString() << std::endl; 
//     std::cout << "Q_sg =  " << std::endl << Q_sg.toString() << std::endl; 
     
//     std::cout << "Q_s_tot =  " << std::endl << Q_s_tot.toString() << std::endl; 

  //   U_s_tot = U_s_l_tot + U_s_r_tot + U_mg;
  //   Q_s_tot = Q_s_lr + Q_mg;     
    
      
//---------------------------------------------------------------------------------------------------------------------
      // Starting Q_j Computation
  
     yarp::sig::Matrix Q_j_lr(robot.getNumberOfJoints(), robot.getNumberOfJoints())  ;
     Q_j_lr.zero() ; 
     yarp::sig::Matrix Q_j_l( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c1( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c2( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c3( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_l_c4( robot.getNumberOfJoints() , 6) ;
     
     yarp::sig::Matrix Q_j_r( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c1( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c2( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c3( robot.getNumberOfJoints() , 6) ;
     yarp::sig::Matrix Q_j_r_c4( robot.getNumberOfJoints() , 6) ;

     yarp::sig::Matrix Jac_l_c1_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c2_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c3_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_l_c4_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version

     yarp::sig::Matrix Jac_r_c1_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c2_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c3_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version
     yarp::sig::Matrix Jac_r_c4_incr_long( robot.getNumberOfJoints() + 6 , 6)  ; //mixed, long, incremented version     

     yarp::sig::Matrix Jac_l_c1_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c2_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c3_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_l_c4_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version

     yarp::sig::Matrix Jac_r_c1_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c2_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c3_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     yarp::sig::Matrix Jac_r_c4_incr( robot.getNumberOfJoints(), 6) ; //spatial short, incremented version
     
     yarp::sig::Matrix T_l_c1_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c2_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c3_w_incr(4,4) ;
     yarp::sig::Matrix T_l_c4_w_incr(4,4) ;

     yarp::sig::Matrix T_r_c1_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c2_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c3_w_incr(4,4) ;
     yarp::sig::Matrix T_r_c4_w_incr(4,4) ;

     yarp::sig::Matrix R_T_l_c1_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c2_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c3_w_incr(4,4) ;
     yarp::sig::Matrix R_T_l_c4_w_incr(4,4) ;

     yarp::sig::Matrix R_T_r_c1_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c2_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c3_w_incr(4,4) ;
     yarp::sig::Matrix R_T_r_c4_w_incr(4,4) ;
     
     yarp::sig::Matrix Jac_l_c1_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c2_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c3_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_l_c4_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version

     yarp::sig::Matrix Jac_r_c1_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c2_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c3_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     yarp::sig::Matrix Jac_r_c4_incr_body(6, robot.getNumberOfJoints() ) ;  //body short, incremented version
     
     //-------------------------------------------------------------------------------------------------------------------
     // REMIND NOTATION
     //  Jac_aw_l_foot_upper_left_link       -> spatial long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_aw_l_foot_upper_left_link_body  -> body long (+6 joints) in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_spa_0                      -> spatial short in the initial configuration (not incemented for derivatives)
     //  Jac_l_c1_body_0                     -> spatial short in the initial configuration (not incemented for derivatives)
        
     yarp::sig::Vector Q_j_l_c1_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c2_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c3_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_l_c4_i( robot.getNumberOfJoints() ) ;

     yarp::sig::Vector Q_j_r_c1_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c2_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c3_i( robot.getNumberOfJoints() ) ;
     yarp::sig::Vector Q_j_r_c4_i( robot.getNumberOfJoints() ) ;   
     
     //---------------------------------------------------------------------------
     // Q_j_l computation     
     for ( int i = 0  ; i<6 ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      //
      model.iDyn3_model.getJacobian( l_foot_upper_left_link_index  , Jac_l_c1_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_upper_right_link_index , Jac_l_c2_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_lower_left_link_index  , Jac_l_c3_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( l_foot_lower_right_link_index , Jac_l_c4_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian

      Jac_l_c1_incr = Jac_l_c1_incr_long.submatrix(0,5, 6, Jac_l_c1_incr_long.cols()-1 ) ; // mixed short
      Jac_l_c2_incr = Jac_l_c2_incr_long.submatrix(0,5, 6, Jac_l_c2_incr_long.cols()-1 ) ;
      Jac_l_c3_incr = Jac_l_c3_incr_long.submatrix(0,5, 6, Jac_l_c3_incr_long.cols()-1 ) ;
      Jac_l_c4_incr = Jac_l_c4_incr_long.submatrix(0,5, 6, Jac_l_c4_incr_long.cols()-1 ) ;
      
      T_l_c1_w_incr  = model.iDyn3_model.getPosition(  l_foot_upper_left_link_index  , true ) ;
      T_l_c2_w_incr  = model.iDyn3_model.getPosition(  l_foot_upper_right_link_index , true ) ;
      T_l_c3_w_incr  = model.iDyn3_model.getPosition(  l_foot_lower_left_link_index  , true ) ;
      T_l_c4_w_incr  = model.iDyn3_model.getPosition(  l_foot_lower_right_link_index , true) ;
      
      R_T_l_c1_w_incr =  getRot( T_l_c1_w_incr  );
      R_T_l_c2_w_incr =  getRot( T_l_c2_w_incr  );
      R_T_l_c3_w_incr =  getRot( T_l_c3_w_incr  );
      R_T_l_c4_w_incr =  getRot( T_l_c4_w_incr  );
      
      Jac_l_c1_incr_body = Adjoint(  Homogeneous(R_T_l_c1_w_incr, zero_3) ) *  Jac_l_c1_incr ;  
      Jac_l_c2_incr_body = Adjoint(  Homogeneous(R_T_l_c2_w_incr, zero_3) ) *  Jac_l_c2_incr ;  
      Jac_l_c3_incr_body = Adjoint(  Homogeneous(R_T_l_c3_w_incr, zero_3) ) *  Jac_l_c3_incr ;  
      Jac_l_c4_incr_body = Adjoint(  Homogeneous(R_T_l_c4_w_incr, zero_3) ) *  Jac_l_c4_incr ;  
     
      Q_j_l_c1_i = ( ( Jac_l_c1_incr_body - Jac_l_c1_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c1 ;
      Q_j_l_c2_i = ( ( Jac_l_c2_incr_body - Jac_l_c2_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c2 ;
      Q_j_l_c3_i = ( ( Jac_l_c3_incr_body - Jac_l_c3_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c3 ;
      Q_j_l_c4_i = ( ( Jac_l_c4_incr_body - Jac_l_c4_body_0)/h  ).transposed()*B_select.transposed()*  fc_l_c4 ;
      
      Q_j_l_c1.setCol( i ,  Q_j_l_c1_i ) ;
      Q_j_l_c2.setCol( i ,  Q_j_l_c2_i ) ;
      Q_j_l_c3.setCol( i ,  Q_j_l_c3_i ) ;
      Q_j_l_c4.setCol( i ,  Q_j_l_c4_i ) ;
     } ;   

     Q_j_l = Q_j_l_c1 + Q_j_l_c2 + Q_j_l_c3 + Q_j_l_c4 ;

     
     //---------------------------------------------------------------------------
     // Q_j_r computation
     for ( int i = 6  ; i<12 ; i++ ) {
      q_incr = q_current ;      
      q_incr[i] += h ;

      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      
      model.iDyn3_model.getJacobian( r_foot_upper_left_link_index  , Jac_r_c1_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_upper_right_link_index , Jac_r_c2_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_lower_left_link_index  , Jac_r_c3_incr_long  , false  ) ; //false= mixed version jacobian //true= body jacobian
      model.iDyn3_model.getJacobian( r_foot_lower_right_link_index , Jac_r_c4_incr_long  , false  ) ;
       
      Jac_r_c1_incr = Jac_r_c1_incr_long.submatrix(0,5, 6, Jac_r_c1_incr_long.cols()-1 ) ;
      Jac_r_c2_incr = Jac_r_c2_incr_long.submatrix(0,5, 6, Jac_r_c2_incr_long.cols()-1 ) ;
      Jac_r_c3_incr = Jac_r_c3_incr_long.submatrix(0,5, 6, Jac_r_c3_incr_long.cols()-1 ) ;
      Jac_r_c4_incr = Jac_r_c4_incr_long.submatrix(0,5, 6, Jac_r_c4_incr_long.cols()-1 ) ;
       
      T_r_c1_w_incr  = model.iDyn3_model.getPosition(  r_foot_upper_left_link_index  , true ) ;
      T_r_c2_w_incr  = model.iDyn3_model.getPosition(  r_foot_upper_right_link_index , true ) ;
      T_r_c3_w_incr  = model.iDyn3_model.getPosition(  r_foot_lower_left_link_index  , true ) ;
      T_r_c4_w_incr  = model.iDyn3_model.getPosition(  r_foot_lower_right_link_index , true ) ;
       
      R_T_r_c1_w_incr =  getRot( T_r_c1_w_incr  );
      R_T_r_c2_w_incr =  getRot( T_r_c2_w_incr  );
      R_T_r_c3_w_incr =  getRot( T_r_c3_w_incr  );
      R_T_r_c4_w_incr =  getRot( T_r_c4_w_incr  );
       
      Jac_r_c1_incr_body = Adjoint(  Homogeneous(R_T_r_c1_w_incr, zero_3) ) *  Jac_r_c1_incr ;  
      Jac_r_c2_incr_body = Adjoint(  Homogeneous(R_T_r_c2_w_incr, zero_3) ) *  Jac_r_c2_incr ;  
      Jac_r_c3_incr_body = Adjoint(  Homogeneous(R_T_r_c3_w_incr, zero_3) ) *  Jac_r_c3_incr ;  
      Jac_r_c4_incr_body = Adjoint(  Homogeneous(R_T_r_c4_w_incr, zero_3) ) *  Jac_r_c4_incr ;  
       
      Q_j_r_c1_i = ( ( Jac_r_c1_incr_body - Jac_r_c1_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c1 ;
      Q_j_r_c2_i = ( ( Jac_r_c2_incr_body - Jac_r_c2_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c2 ;
      Q_j_r_c3_i = ( ( Jac_r_c3_incr_body - Jac_r_c3_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c3 ;
      Q_j_r_c4_i = ( ( Jac_r_c4_incr_body - Jac_r_c4_body_0)/h  ).transposed()*B_select.transposed()*  fc_r_c4 ;
       
      Q_j_r_c1.setCol( i-6 ,  Q_j_r_c1_i ) ;
      Q_j_r_c2.setCol( i-6 ,  Q_j_r_c2_i ) ;
      Q_j_r_c3.setCol( i-6 ,  Q_j_r_c3_i ) ;
      Q_j_r_c4.setCol( i-6 ,  Q_j_r_c4_i ) ; 
     }     
     
     Q_j_r = Q_j_r_c1 + Q_j_r_c2 + Q_j_r_c3 + Q_j_r_c4 ;
     Q_j_lr.setSubmatrix(Q_j_l, 0, 0 )  ;
     Q_j_lr.setSubmatrix(Q_j_r, 0, 6 )  ;
 
     //--------------------------------------------------------------------------------------------------
     // Computing Q_jg
     //--------------------------------------
 
     yarp::sig::Matrix Jac_COM_w_incr(6, robot.getNumberOfJoints()+6 ) ;
     yarp::sig::Matrix T_w_imu_incr(4,4)  ;     
     yarp::sig::Matrix Jac_COM_aw_long_incr(6, robot.getNumberOfJoints()+6 ) ;
     yarp::sig::Matrix Jac_COM_aw_incr( 3, robot.getNumberOfJoints() )     ;
     yarp::sig::Vector d_Jg_i(robot.getNumberOfJoints()) ;
     yarp::sig::Matrix Q_jg( robot.getNumberOfJoints() , robot.getNumberOfJoints()) ;
     
     for ( int i = 0  ; i<(robot.getNumberOfJoints()-1) ; i++ ) 
     {
      q_incr = q_current ;
      q_incr[i] += h ;
      robot.idynutils.updateiDyn3Model( q_incr, true ); //update model first  
      model.iDyn3_model.getCOMJacobian( Jac_COM_w_incr ) ;  
      T_w_imu_incr  = model.iDyn3_model.getPosition( imu_link_index  )   ;
      T_aw_w_incr = iHomogeneous( T_w_imu_incr * T_imu_aw  ) ;    
      Jac_COM_aw_long_incr = Adjoint( Homogeneous( getRot( T_aw_w_incr ) , zero_3) )*Jac_COM_w_incr ;    
      Jac_COM_aw_incr = Jac_COM_aw_long_incr.submatrix( 0, 2, 6, Jac_COM_aw_long_incr.cols()-1 ) ; 
      d_Jg_i = ((Jac_COM_aw_incr - Jac_COM_aw_0)/h).transposed()* mg*g_dir_aw;   
      Q_jg.setCol( i ,  d_Jg_i ) ;     
     }
     
    //    std::cout << "Q_jg : "  << std::endl << Q_jg.toString() << std::endl;
    //  std::cout << "Q_jg.rows : "  << std::endl << Q_jg.rows() << std::endl;
    //  std::cout << "Q_jg.cols : "  << std::endl << Q_jg.cols() << std::endl;
     
      yarp::sig::Matrix Q_j_tot = Q_j_lr + Q_jg ;
     
  //    std::cout << "Q_j_tot : "  << std::endl << Q_j_tot.toString() << std::endl;
     
      
        
       
       
        
        
       
       
       
       
       
       
       
       
   /*    std::cout << "U_s_l_tot : "  << std::endl << U_s_l_tot.toString() << std::endl;
       std::cout << "U_s_r_tot : "  << std::endl << U_s_r_tot.toString() << std::endl;
       std::cout << "U_sg : "  << std::endl << U_sg.toString() << std::endl;
       std::cout << "Q_s_l : "  << std::endl << Q_s_l.toString() << std::endl;
       std::cout << "Q_s_r : "  << std::endl << Q_s_r.toString() << std::endl;
       std::cout << "Q_s_lr : "  << std::endl << Q_s_lr.toString() << std::endl;
       std::cout << "Q_sg : "  << std::endl << Q_sg.toString() << std::endl;
       std::cout << "Q_j_l : "  << std::endl << Q_j_l.toString() << std::endl;
       std::cout << "Q_j_r : "  << std::endl << Q_j_r.toString() << std::endl;
       std::cout << "Q_jg : "  << std::endl << Q_jg.toString() << std::endl;
       
       std::cout << "Q_j_tot : "  << std::endl << Q_j_tot.toString() << std::endl;  
     
       std::cout << "U_jg : "  << std::endl << U_jg.toString() << std::endl;   */
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     

     
     
 
     
     
  
     


    
    
    
    
    
  /*  yarp::sig::Vector omega(3);
    omega[0] = 1.0 ;
    omega[1] = 2.0 ;
    omega[2] = 3.0 ;
    yarp::sig::Vector omega_1(3);
    omega_1[0] = 4.0 ;
    omega_1[1] = 5.0 ;
    omega_1[2] = 6.0 ;
    
    yarp::sig::Matrix R_omega = exp_omega_theta(omega, 1.0) ;
    yarp::sig::Matrix R_omega_1 = exp_omega_theta(omega_1, 1.0) ;
    std::cout << "R_omega =  " << std::endl << R_omega.toString() << std::endl;
    std::cout << "R_omega_1 =  " << std::endl << R_omega_1.toString() << std::endl;  */
    

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


