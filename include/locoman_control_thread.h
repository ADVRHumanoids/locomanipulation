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
    
    
    //Example stuff for parameters setting from outside
    yarp::sig::Vector left_arm_configuration;
    double max_vel;
public:
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
     * @brief  AdjToPose transforms an Adjoint matrix into the original homogenous matrix
     * @param  Adj is a 6x6 yarp matrix describing an Adjoint transformation
     * @return 4x4 yarp matrix describing a homogenous transformation
     */
    yarp::sig::Matrix AdjToPose( const yarp::sig::Matrix Adj) ;
    
   
  
  
};

#endif
