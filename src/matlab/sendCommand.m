function sendCommand(actionClient, commandType, targetPose, yaw)
    % Create and fill goal message
    goalMsg = ros2message(actionClient);
    goalMsg.command_type = string(commandType);
    if nargin >= 3 && ~isempty(targetPose)
        goalMsg.target_pose = targetPose;
    end
    if nargin >= 4 && ~isempty(yaw)
        goalMsg.yaw = yaw;
    end

    % Display command info
    fprintf('Sending command: %s', commandType);
    if exist('targetPose', 'var') && ~isempty(targetPose)
        fprintf(', target_pose: [%s]', num2str(targetPose));
    end
    if exist('yaw', 'var') && ~isempty(yaw)
        fprintf(', yaw: %.2f', yaw);
    end
    fprintf('\n');

    try
        % Send goal, only one output
        goalHandle = sendGoal(actionClient, goalMsg, @goalResponseCallback);

        % Get future for result asynchronously
        resultFuture = goalHandle.get_result_async();
        resultFuture.addlistener('Completed', @resultCallback);

    catch e
        warning('Failed to send goal: %s', e.message);
    end

    function goalResponseCallback(goalHandle)
        if isempty(goalHandle)
            disp('[Goal Response] Goal was rejected by server.');
        else
            disp('[Goal Response] Goal accepted. Waiting for result...');
        end
    end

    function resultCallback(event)
        result = event.Result.result;
        fprintf('[Result] success=%d, message="%s"\n', result.success, result.message);

        if ~result.success && ~strcmp(commandType, 'disarm')
            disp('[Recovery] Attempting to disarm due to failure...');
            global actionClientGlobal;
            if ~isempty(actionClientGlobal)
                sendCommand(actionClientGlobal, 'disarm');
            else
                disp('No global action client available.');
            end
        end
    end
end
