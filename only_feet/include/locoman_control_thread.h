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
    
    double max_vel;
public:


    int mg =  290 ; // [N]  295 // mg_coman = 290; mg_bigman = 1000 ;
    int loop_counter;
    int WINDOW_size;
    int FC_size ;  
    bool flag_robot = 1 ;
    bool flag_simulator = 1-flag_robot ;
    
    
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
