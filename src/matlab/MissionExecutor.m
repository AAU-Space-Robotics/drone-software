% MATLAB ROS2 Node for Drone Arming and Goto
% File: ~/drone-software/src/matlab/MissionExecutor_ArmAndGoto.m

% Clear previous ROS2 environment to avoid conflicts
clear all;

%% Setup paths to generated messages
script_path = mfilename('fullpath');
script_dir = fileparts(script_path);
workspace_dir = fileparts(fileparts(script_dir));
msg_gen_path = fullfile(workspace_dir, 'src', 'matlab_msg_gen', 'glnxa64', 'src', 'interfaces', 'm');

if isfolder(msg_gen_path)
    addpath(msg_gen_path);
    disp(['Added ROS2 message path: ', msg_gen_path]);
else
    error('Generated message path not found at %s. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB to generate messages.', msg_gen_path, workspace_dir);
end

%% Import custom message package
try
    import ros.internal.ros2.custommessages.interfaces.*
    disp('Imported custom message package: ros.internal.ros2.custommessages.interfaces');
catch e
    warning('Failed to import custom message package: %s. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB to generate messages.', e.message, workspace_dir);
end

%% Check available ROS2 message types for debugging
msg_list = ros2('msg', 'list');
if ~any(contains(msg_list, 'interfaces/DroneCommand'))
    warning('Action message interfaces/DroneCommand not found. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB.', workspace_dir);
end

%% Create a ROS2 node with default domain ID (0)
try
    node = ros2node('/drone_control_node', 0);
    disp('ROS2 node created: /drone_control_node');
catch e
    error('Failed to create ROS2 node: %s', e.message);
end

%% Create an action client for DroneCommand with custom QoS settings
qos = struct('Reliability', 'best_effort', 'Durability', 'transient_local', 'History', 'keeplast', 'Depth', 5);
try
    action_client = ros2actionclient(node, '/thyra/in/drone_command', 'interfaces/DroneCommand', qos);
    disp('Waiting for action server /thyra/in/drone_command...');
    waitForServer(action_client, 'Timeout', 10);
    disp('Action server connected.');
catch e
    delete(node);
    error('Failed to create action client or connect to server: %s. Check ROS2 network and topic /thyra/in/drone_command.', e.message);
end

% Define a global variable for fallback disarming (used in sendCommand.m)
global actionClientGlobal;
actionClientGlobal = action_client;

%% Arm the drone asynchronously
disp('Arming the drone...');
sendCommand(action_client, 'arm');
% Wait a short period to let the asynchronous transaction proceed
pause(2);

%% Send a goto command asynchronously
disp('Sending goto command...');
target_pose = [0.0, 0.0, -3.0];
yaw = deg2rad(90);
sendCommand(action_client, 'goto', target_pose, yaw);
pause(5);

%% (Optional) Additional commands can be placed here.

%% Cleanup
disp('Shutting down...');
delete(action_client);
delete(node);
