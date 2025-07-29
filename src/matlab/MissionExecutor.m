% MATLAB ROS2 Node for Drone Arming
% File: ~/drone-software/src/matlab/MissionExecutor_ArmOnly.m

% Clear previous ROS2 environment to avoid conflicts
clear all;

% Get the script's directory and construct message path
script_path = mfilename('fullpath'); % Full path of this script
script_dir = fileparts(script_path); % Directory: ~/drone-software/src/matlab
workspace_dir = fileparts(fileparts(script_dir)); % Workspace: ~/drone-software
msg_gen_path = fullfile(workspace_dir, 'src', 'matlab_msg_gen', 'glnxa64', 'src', 'interfaces', 'm'); % ~/drone-software/src/matlab_msg_gen/glnxa64/src/interfaces/m

% Add generated message path
if isfolder(msg_gen_path)
    addpath(msg_gen_path);
    disp(['Added ROS2 message path: ', msg_gen_path]);
else
    error('Generated message path not found at %s. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB to generate messages.', msg_gen_path, workspace_dir);
end

% Import custom message package
try
    import ros.internal.ros2.custommessages.interfaces.*
    disp('Imported custom message package: ros.internal.ros2.custommessages.interfaces');
catch e
    warning('Failed to import custom message package: %s. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB to generate messages.', e.message, workspace_dir);
end

% Check available ROS2 message types for debugging
disp('Available ROS2 message types:');
msg_list = ros2('msg', 'list');
disp(msg_list);
if ~any(contains(msg_list, 'interfaces/DroneCommand'))
    warning('Action message interfaces/DroneCommand not found. Run "ros2genmsg(''%s/src/interfaces'', ''CreateShareableFile'', true)" in MATLAB.', workspace_dir);
end

% Create a ROS2 node with default domain ID (0)
try
    node = ros2node('/drone_control_node', 0);
    disp('ROS2 node created: /drone_control_node');
catch e
    error('Failed to create ROS2 node: %s', e.message);
end

% Create an action client for DroneCommand with QoS settings
qos = struct('Reliability', 'best_effort', 'Durability', 'transient_local', 'History', 'keeplast', 'Depth', 10);
try
    action_client = ros2actionclient(node, '/thyra/in/drone_command', 'interfaces/DroneCommand', qos);
    disp('Waiting for action server /thyra/in/drone_command...');
    waitForServer(action_client, 'Timeout', 10); % Wait up to 10 seconds
    disp('Action server connected.');
catch e
    delete(node);
    error('Failed to create action client or connect to server: %s. Check ROS2 network and topic /thyra/in/drone_command.', e.message);
end

% Helper function to send action goal and wait for result
function [success, message] = sendCommand(client, command_type)
    try
        % Create goal message
        goal_msg = ros2message(client);
        goal_msg.command_type = command_type;
        goal_msg.target_pose = double([]); % Empty array of doubles for arm/disarm
        goal_msg.yaw = double(0.0); % Default yaw as double for arm/disarm

        % Send goal and get goal handle
        disp(['Sending command: ', command_type]);
        goal_handle = sendGoal(client, goal_msg);
        
        % Inspect goal handle
        disp('Goal handle structure:');
        disp(goal_handle);

        % Wait for result
        disp(['Waiting for result of command: ', command_type]);
        [is_ready, result, status, status_text] = waitForResult(goal_handle, 'Timeout', 10);
        
        % Check if result is ready
        if is_ready
            disp('Result received:');
            disp(result);
            % Check result fields based on action definition
            if isfield(result, 'success') && isfield(result, 'message')
                success = result.success;
                message = result.message;
                if success
                    disp(['Command ', command_type, ' succeeded: ', message]);
                else
                    disp(['Command ', command_type, ' failed: ', message]);
                end
            else
                % Additional debugging for unexpected result format
                disp('Available fields in result:');
                disp(fieldnames(result));
                error('Result missing expected fields (success, message).');
            end
        else
            error('Timeout waiting for result of command %s: %s', command_type, status_text);
        end
    catch e
        disp(['Error sending command ', command_type, ': ', e.message]);
        success = false;
        message = e.message;
    end
end

% Arm the drone
try
    disp('Arming the drone...');
    [success, message] = sendCommand(action_client, 'arm');
    if ~success
        error('Failed to arm the drone: %s', message);
    end
    pause(2); % Allow time for arming to complete
catch e
    disp(['Error occurred: ', e.message]);
    % Attempt to disarm the drone in case of error
    disp('Attempting to disarm drone due to error...');
    % sendCommand(action_client, 'disarm');
end

% Cleanup
disp('Shutting down...');
delete(action_client);
delete(node);