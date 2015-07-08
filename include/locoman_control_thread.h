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
    
    //Example stuff (for sin wave generation)
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
    int mg =  290 ; // [N]  295
    int loop_counter;
    int WINDOW_size;
    int FC_size ;  
    bool flag_robot = 0 ;
    bool flag_simulator = 1-flag_robot ;
    double part_hand ;
    double part_foot_left ;
    yarp::sig::Vector FC_DES ;  //     yarp::sig::Vector FC_DES( FC_size   ) ;
    yarp::sig::Vector FC_DES_LEFT_sensor ;
    yarp::sig::Vector FC_DES_RIGHT_sensor ;
    yarp::sig::Vector FC_DES_RIGHT_HAND_sensor ;
    yarp::sig::Vector FC_SUM ;
    yarp::sig::Vector FC_FILTERED ;
    yarp::sig::Matrix FC_WINDOW ;  //    yarp::sig::Matrix FC_WINDOW(FC_size, WINDOW_filter ) ;
    
  
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


     /**
     * @brief getKq returns the joint stiffness matrix (hardcoded for now)
     * @return a yarp Matrix of dimension =  robot.getNumberOfJoints() x robot.getNumberOfJoints()
     */
    yarp::sig::Matrix getKq( ) ;

    
     /**
     * @brief senseMotorPosition we use this method to obtain the motor position
     * @return a yarp vector of dimension equal to robot.getNumberOfJoints() 
     */
    yarp::sig::Vector senseMotorPosition( ) ;
    
    //-----------------------------------------------------------

     /**
     * @brief  getRot extracts the rotation matrix from a homogenous matrix
     * @param  T_ab is a 4x4 yarp matrix describing a homogenous transformation
     * @return R_ab is a 3x3 yarp matrix
     */
    yarp::sig::Matrix getRot( const yarp::sig::Matrix T_ab) ;

    //------------------------------------------------------------
    
     /**
     * @brief  getTrasl extracts the translation vector from a homogenous matrix
     * @param  T_ab is a 4x4 yarp matrix describing a homogenous transformation
     * @return d_ab  is a 3x1 yarp vector 
     */
    yarp::sig::Vector getTrasl( const yarp::sig::Matrix T_ab) ;
    
    //------------------------------------------------------------
    
     /**
     * @brief  homogeneous computes the homogenous transformation composed by:
     * @param  R_ab is a 3x3 yarp matrix describing the rotational part
     * @param  d_ab is a 3x1 yarp vector describing the translational part   
     * @return T_ab is a 4x4 yarp matrix 
     */
    yarp::sig::Matrix Homogeneous( const yarp::sig::Matrix R_ab,  const yarp::sig::Vector d_ab  ) ;
    
    
    //-----------------------------------------------------------------
     /**
     * @brief  iHomogeneous computes the inverse of an homogenous transformation
     * @param  T_ab is a 4x4 yarp matrix describing an homogenous transformation
     * @return T_ba  is a 4x4 yarp matrix, the inverse of T_ab
     */
    yarp::sig::Matrix iHomogeneous( const yarp::sig::Matrix T_ab) ;
    
     
     //-----------------------------------------------------------------
     /**
     * @brief  Adjoint computes the adjoint matrix := Ad_g from a homogenous transformation
     * @param  T_ab is a 4x4 yarp matrix describing an homogenous transformation
     * @return Ad_T_ab is a 6x6 yarp matrix, able to map twists in twists
     */
    yarp::sig::Matrix Adjoint( const yarp::sig::Matrix T_ab) ;
    
     //-----------------------------------------------------------------
     /**
     * @brief  Adjoint_MT computes the inverse transpose of the adjoint  matrix := (Ad_(g^{-1}))^{T} from a homogenous transformation
     * @param  T_ab is a 4x4 yarp matrix describing an homogenous transformation
     * @return ( Ad_(T_ab)^{-1} )^{T}  is a 6x6 yarp matrix, able to map wrenches in wrenches
     */
    yarp::sig::Matrix Adjoint_MT( const yarp::sig::Matrix T_ab) ;
    
         
    
    //-----------------------------------------------------------------
     /**
     * @brief  fConToSens maps contact forces on the sensor frame, is equivalent to a grasp matrix
     * @param  sens_index, ... c4_index are the indexes of the sensor and of the 4 contact points
     * @return is a 6x6 yarp matrix, able to map contact forces in sensor wrenches
     */
    yarp::sig::Matrix fConToSens( const int sens_index,
                                  const int c1_index,
                                  const int c2_index,
				  const int c3_index,
                                  const int c4_index  ) ;

				  
     //-----------------------------------------------------------------
     /**
     * @brief  SkewToVect transforms a cross product matrix into the original vector
     * @param  Skew is a 3x3 yarp matrix describing an cross product
     * @return 3x1 yarp vector
     */
    yarp::sig::Vector SkewToVect( const yarp::sig::Matrix Skew) ;
    
    
     //-----------------------------------------------------------------
     /**
     * @brief  xi_hat returns the homogeneous form of a tiwst
     * @param  xi is a 6 dimentional yarp vector describing a twits = [v^T, w^T]^T
     * @return 4x4 yarp matrix describing the homogenous form of a the twist xi
     */
    yarp::sig::Matrix xi_hat( const yarp::sig::Vector xi) ;

     //-----------------------------------------------------------------
     /**
     * @brief  exp_omega_theta returns the rotation matrix provided by the Rodrigues formula
     * @param  omega is the rotation axis
     * @param  theta is the rotation amount
     * @return 4x4 yarp matrix describing the homogenous form of a the twist xi
     */
    yarp::sig::Matrix exp_omega_theta( const yarp::sig::Vector omega, const double theta) ;


    
    
     //-----------------------------------------------------------------
     /**
     * @brief  exp_xi_theta returns the homogenous transformation associated to the twist
     * @param  xi is the twist = [v^T, w^T]^T 
     * @param  theta is the transformation amount
     * @return 4x4 yarp matrix describing the homogenous transformation
     */
    yarp::sig::Matrix twistexp( const yarp::sig::Vector xi, const double theta) ;

    


   
     //----------------------------------------------------------------------------
     /**
     * @brief  AdjToPose transforms an Adjoint matrix into the original homogenous matrix
     * @param  Adj is a 6x6 yarp matrix describing an Adjoint transformation
     * @return 4x4 yarp matrix describing a homogenous transformation
     */
    yarp::sig::Matrix AdjToPose( const yarp::sig::Matrix Adj) ;
    
    
    
    
     //----------------------------------------------------------------------------
     /**
     * @brief  ad_lie transforms a tiwst given as a yarp matrix into the Lie adjoint matrix
     * @param  Xi is a 6x1 yarp matrix describing a twist
     * @return 6xc yarp matrix 
     */
    yarp::sig::Matrix ad_lie( const yarp::sig::Matrix Xi) ;
    
    yarp::sig::Matrix ad_lie( const yarp::sig::Vector Xi) ;

    
    
     //----------------------------------------------------------------------------
     /**
     * @brief  D_Jacob_spa_i compute the derivative of the spatial Jacobian with respect to the i-th q
     * @param  J_s is a 6xc yarp matrix describing a spatial Jacobian
     * @param  i -th joint () with repect to the derivative is computed 
     * @return 6x6 yarp matrix describing the Lie adjoint matrix
     */
    yarp::sig::Matrix D_Jacob_spa_i( const yarp::sig::Matrix J_s, const int i ) ;


         //----------------------------------------------------------------------------
     /**
     * @brief  Q_ci compute the derivative of the spatial Jacobian 
     * @param  J_spa_i is a 6xc yarp matrix describing a spatial Jacobian
     * @param  T_a_ci is the homogeneous transformation between the floating base and the contact frame 
     * @param  f_ci contact force vector
     * @return qxq yarp matrix 
     */
    yarp::sig::Matrix Q_ci( const yarp::sig::Matrix J_spa_i, const yarp::sig::Matrix T_a_ci , const yarp::sig::Vector f_ci) ;
    
    
     //----------------------------------------------------------------------------
     /**
     * @brief  RoundMatrix computes the round of M to k decimal places 
     * @param  M yarp matrix to round
     * @param  k number of decimal places 
     * @return rounded yarp matrix 
     */
    yarp::sig::Matrix RoundMatrix( const yarp::sig::Matrix M, const int k) ;
    
    
    
    
    
     //------------------------------------------------------------------------------------
     /**
     * @brief  FLMM_ext computes the FLMM for a compliant humanoid robot 
     * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
     * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
     * @param  Q_j is a joints x joints yarp matrix about the derivative of the Jacobian 
     * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
     * @param  U_j is a joints x 6 yarp matrix about the derivative of the Jacobian 
     * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
     * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
     * @param  K_q is a joints x joints yarp matrix describing the joint stiffness matrix
     * @return FLMM_ext is the Fundamental Loco-Manipulation Matrix
     */
    yarp::sig::Matrix FLMM_ext( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_j,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_j,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c,
			    const yarp::sig::Matrix K_q
			      ) ;

   //------------------------------------------------------------------------------------
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
    yarp::sig::Matrix Rf_ext( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_j,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_j,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c,
			    const yarp::sig::Matrix K_q
			      ) ;			      
	      
     //------------------------------------------------------------------------------------
     /**
     * @brief  FLMM_redu computes a basic version of the FLMM for a rigid humanoid robot 
     * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
     * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
     * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
     * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
     * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
     * @return FLMM_ext is the Fundamental Loco-Manipulation Matrix
     */
    yarp::sig::Matrix FLMM_redu( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c
			      ) ; 			      
     //------------------------------------------------------------------------------------
     /**
     * @brief  Rf_redu computes the joints-forces map for a rigid humanoid robot 
     * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
     * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
     * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
     * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
     * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
     * @return Rf_redu is the Fundamental Loco-Manipulation Matrix
     */
    yarp::sig::Matrix Rf_redu( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c
			      ) ;			      
			      
       //------------------------------------------------------------------------------------
     /**
     * @brief  Ru_redu computes the joints-movements map for a a rigid humanoid robot 
     * @param  J_c is a contacts x joints yarp matrix describing the body Jacobian of the humanoid
     * @param  S_c is a 6 x contacts yarp matrix describing the body Stance matrix of the humanoid
     * @param  Q_s is a contacts x joints yarp matrix about the derivative of the Jacobian 
     * @param  U_s is a 6 x 6 yarp matrix about the derivative of the Jacobian 
     * @param  K_c is a contacts x contacts yarp matrix describing the contact stiffness matrix
     * @return Ru_redu is the Fundamental Loco-Manipulation Matrix
     */
    yarp::sig::Matrix Ru_redu( const yarp::sig::Matrix J_c ,
			    const yarp::sig::Matrix S_c ,
			    const yarp::sig::Matrix Q_s,
			    const yarp::sig::Matrix U_s,
			    const yarp::sig::Matrix K_c
			      ) ;		
//------------------------------------------------------------------------------------
     /**
     * @brief  Pinv_trunc_SVD computes the pseudoinverse of A via the truncated SVD method 
     * @param  A is the matrix to pseudo-inverse
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return Pinv_trunc_SVD is the pseudo-inverse of A
     */
      yarp::sig::Matrix Pinv_trunc_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ;   // 
//------------------------------------------------------------------------------------
     /**
     * @brief  Pinv_Regularized computes the Tikhonov Regularized pseudo-inverse of A 
     * @param  A is the matrix to pseudo-inverse
     * @param  k is the regularization factor
     * @return Pinv_Regularized is the pseudo-inverse of A
     */ 
      yarp::sig::Matrix Pinv_Regularized( const yarp::sig::Matrix A ,
			                  const double k  
			                ) ;     
//------------------------------------------------------------------------------------
     /**
     * @brief  x_Pinv_Iter computes the variable x: Ax=b via the Landweber iteration method
     * @param  A is the matrix to pseudo-inverse
     * @param  b is the vector of known terms
     * @param  n is the maximum number of steps to be performed (less that the minimum dimension of A)
     * @return x_Pinv_Iter is the solution vetor
     */
      yarp::sig::Vector x_Pinv_Iter( const yarp::sig::Matrix A , 
				   const yarp::sig::Vector b , 
			           double n 
			           ) ;   // 			      
     /**
     * @brief  orth_SVD computes a basis for the span of A via the truncated SVD method 
     * @param  A is the matrix of which a basis is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return orth_SVD is a basis for the span of A
     */
      yarp::sig::Matrix orth_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ;   // 			           
     /**
     * @brief  null_SVD computes a basis for the nullspace of A via the truncated SVD method 
     * @param  A is the matrix of which a basis for the nullspace is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return null_SVD is a basis for the nullspace of A
     */
      yarp::sig::Matrix null_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ;  
           
     /**
     * @brief  filter_SVD computes a "fileters" version of A via the truncated SVD  
     * @param  A is the matrix of which the filtration effect is needed
     * @param  k is the maximum ratio admitted between the max and min singular values
     * @return filter_SVD is the filtered version of A
     */
      yarp::sig::Matrix filter_SVD( const yarp::sig::Matrix A ,
			                const double k = 1E-4 
			              ) ; 				      
				      
			           
};






#endif
