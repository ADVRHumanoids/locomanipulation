// include yarp and generic_module headers
#include <yarp/os/all.h>
#include <GYM/generic_module.hpp>
// include locoman module and control thread headers 
#include "locoman_module.hpp"
#include "locoman_control_thread.h"

// define representing the period of the module in [milliseconds]
#define MODULE_PERIOD_MILLISEC 1000

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create the resource finder for the locomanipulation module
    yarp::os::ResourceFinder locoman_rf;
    locoman_rf.setVerbose(true);
    locoman_rf.setDefaultConfigFile( "locoman_configuration.ini" );
    locoman_rf.setDefaultContext( "generic_locoman" );  //generic_locoman
    locoman_rf.configure(argc, argv);

    // create locomanipulation module
    locoman_module locoman_mod = locoman_module( argc, 
                                                    argv, 
                                                    "generic_locoman",   //generic_locoman
                                                    MODULE_PERIOD_MILLISEC, 
                                                    locoman_rf );
        

    
    // run the module
    locoman_mod.runModule( locoman_rf );
    
        // yarp network deinitialization
    yarp.fini();
    exit(EXIT_SUCCESS);
}
