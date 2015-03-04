#ifndef LOCOMAN_MODULE_HPP_
#define LOCOMAN_MODULE_HPP_

// include generic module header
#include <GYM/generic_module.hpp>
// include control thread and constants header files
#include "locoman_control_thread.h"
#include "locoman_constants.h"
#include <GYM/control_module.hpp>
/**
 * @brief The locoman_module class inherit from generic_module.
 */
class locoman_module : public control_module<locoman_control_thread> {
public:	
    /**
     * @brief locoman_module constructor
     * @param argc main argc
     * @param argv main argv
     * @param module_prefix prefix of the module
     * @param module_period period of the module (ex: 1000 [ms])
     * @param rf resource finder
     */
    locoman_module(    int argc, 
                        char* argv[],
                        std::string module_prefix, 
                        int module_period, 
                        yarp::os::ResourceFinder rf ) : control_module<locoman_control_thread>(    argc, 
                                                                                                    argv, 
                                                                                                    module_prefix, 
                                                                                                    module_period,
                                                                                                    rf )
    {
    }
    
    /**
     * @brief custom_get_ph_parameters inherit from generic module, we reimplement it since we have more parameters in the
     * param_help (locoman_configuration.ini file) than the default ones.
     * @return a vector of the custom parameters for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {   
        // custom param helper parameters vector
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        // insert left_arm param
        /// NOTE: "left_arm" id has to be the same of the one in the locoman_configuration.ini
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(   "left_arm", 
                                                                            PARAM_ID_LEFT_ARM, 
                                                                            PARAM_SIZE_LEFT_ARM, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "left_arm configuration in [degree]" ) );
        // insert max_vel param
        custom_params.push_back( new paramHelp::ParamProxyBasic<double>(    "max_vel", 
                                                                            PARAM_ID_MAX_VEL, 
                                                                            PARAM_SIZE_MAX_VEL, 
                                                                            paramHelp::PARAM_IN_OUT, 
                                                                            NULL, 
                                                                            "maximum velocity in [degree/second]" ) );
        
        return custom_params;
    }
};

#endif
