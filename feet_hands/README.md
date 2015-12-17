% 
% Hi! 
% This is a first attempt of implementation of 
% the Whole-Body Loco-Manipulation Analysis and Control Tools
% Stay tuned for updates!
%
% How To Run the Module:
% 
% 
% Open 5 terminal:
% 
% On the simulator:
% Terminal 1. Run yarpserver
% >> yarpserver --write
% 
% Terminal 2. Run roscore
% >> roscore
% 
% Terminal 3. Run Gazebo
% >> gazebo 
% 
% Insert a Coman in the world
% 
% Terminal 4. Run the Module!!
% >> locomanipulation
%
% Terminal 5. Start (or quit) the Module!!
% >> yarp write ... /generic_locoman/switch:i
%    start
%   quit
% >> yarp write ... /generic_locoman/command:i
%   ("your command") 0
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% On the robot: % NOT TESTED
% NOT TESTED
% 1. yarpserver --write
% 2. comanInterface
% 3. roscore
% 4. roslaunch yarp_ros_joint_state_publisher coman_status.launch
% 5. >> locomanipulation
%    >> yarp write ... /generic_locoman/switch:i
%    start
%    quit
%    >> yarp write ... /generic_locoman/command:i
%    ("your command") 0
%
%
%
