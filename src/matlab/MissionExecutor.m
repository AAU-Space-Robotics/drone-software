function MissionExecutor()
    node = ros2node("/thyra_mission_executor_matlab");
    qos = struct('Reliability', 'besteffort', ...
                 'Durability', 'volatile', ...
                 'History', 'keeplast', ...
                 'Depth', 5);

    % Load camera parameters
    cameraParamsExtracted = load('cameraParams.mat');
    camIntrinsics = cameraParamsExtracted.cameraParams;
    markerSize = 150; % in mm
    
    % Mission parameters
    takeoff_marker_id = 101;
    landing_marker_id = 102;
    all_probes_found = false;
    probe_search_amount = 3;
    search_height = -1.5; % in meters
    landing_aruco_detection_threshold = 5;
    landing_aruco_detection_count = 0;    
    landing_aruco_detection_list = [];
    landing_marker_pose_estimate = [];
    landing_marker_detected = false;
    search_attempts = 0;
    search_attempts_max = 3;


    % Define CAMERA_TO_DRONE_TRANSFORM (4x4 homogeneous matrix) in meters
    cameraToDrone = [...
        0.0, -0.70710678, 0.70710678, 0.137751; ...
        1.0, 0.0, 0.0, -0.018467; ...
        0.0, 0.70710678, 0.70710678, 0.12126; ...
        0.0, 0.0, 0.0, 1.0];

    last_drone_state = [];
    last_RGB_image = [];
    last_pose_synced = [];

    % Subscribers
    sub_state = ros2subscriber(node, "/thyra/out/drone_state", ...
                                "interfaces/DroneState", @msgCallback_dronestate);
    sub_RGB_image = ros2subscriber(node, "/thyra/out/color_image/compressed", ...
                                   "sensor_msgs/CompressedImage", @msgCallback_RGB, qos);
    sub_pose_synced = ros2subscriber(node, "/thyra/out/pose/synced_with_RGBD", ...
                                     "geometry_msgs/PoseStamped", @msgCallback_Pose, qos);

    % Action client for DroneCommand
    [ac, goalMsg] = ros2actionclient(node, ...
        "/thyra/in/drone_command", ... 
        "interfaces/DroneCommand");

    disp('Waiting for DroneCommand action server...');
    waitForServer(ac);

    % Misson code
    disp('Mission execution is now beginning...');

    % Arm
    sendCommand('arm', [], 0);
    pause(1);

    % Takeoff
    sendCommand('takeoff', [-2.0], 0);
    disp('Waiting for takeoff to complete...');
    waitForTrajectoryCompletion(0.1);

    % Spin
    disp('Spinning until all probes and landing marker are found...');
    while ~(all_probes_found && landing_marker_detected)
        sendCommand('spin', [0.0, 1.0, 0.0], 0);
        waitForTrajectoryCompletion(0.1);
        check_if_all_probes_found();
        if search_attempts < search_attempts_max
            search_attempts = search_attempts + 1;  
        end
        if search_attempts >= search_attempts_max
            disp('Max search attempts reached. Stopping search.');
            break;
        end
        pause(0.5); % Give time for callbacks to process images
    end

    landing_marker_pose_estimate_xy = [];
    if all_probes_found
        if landing_marker_detected
            disp('All probes and landing marker found.');
            landing_marker_pose_estimate_xy = [landing_marker_pose_estimate(1:2), search_height];
        else
            disp('All probes found, but landing marker not detected.');
            landing_marker_pose_estimate_xy = [0, 0, search_height];
        end
    elseif landing_marker_detected
        disp('Landing marker found, but not all probes detected.');
        landing_marker_pose_estimate_xy = [landing_marker_pose_estimate(1:2), search_height];
    end

    % Goto landing marker location
    
    sendCommand('goto', landing_marker_pose_estimate_xy, 0);
    disp('Waiting for goto to complete...');
    waitForTrajectoryCompletion(0.1);

    % Land
    sendCommand('land', [], 0);
    disp('Waiting for landing to complete...');
    waitForTrajectoryCompletion(0.1);

    disp('Mission execution completed.');

    % === Callbacks ===
    function msgCallback_dronestate(msg)
        last_drone_state = msg;
    end
    function msgCallback_RGB(msg)
        last_RGB_image = msg;
        rgb_image = rosReadImage(msg);

        [ids, tvecs, rvecs] = detectArucoInCameraFrame(rgb_image, camIntrinsics, markerSize);

        if isempty(ids)
            disp('No markers detected.');
            return;
        end

        if isempty(last_pose_synced)
            disp('No synced global pose available yet.');
            return;
        end

        T_world_drone = poseMsgToMatrix(last_pose_synced);

        disp(['Detected markers: ', num2str(ids(:)')]);
        for i = 1:length(ids)
            rvec = rvecs(i,:);
            %disp(['Marker ', num2str(ids(i)), ' in drone frame: ', mat2str(pose_in_drone_frame(1:3)', 4)]);
            % If the landing marker is found, add to the list of detections and increment the count
            if ids(i) == landing_marker_id && ~landing_marker_detected
                % Marker in drone frame
                pose_in_drone_frame = cameraToDrone * [tvecs(i,:)'; 1];

                % Transform to world frame
                pose_in_world = T_world_drone * pose_in_drone_frame;
                %disp(['Marker ', num2str(ids(i)), ' in world frame: ', mat2str(pose_in_world(1:3)', 4)]);

                landing_aruco_detection_count = landing_aruco_detection_count + 1;
                landing_aruco_detection_list = [landing_aruco_detection_list; pose_in_world(1:3)'];

                if landing_aruco_detection_count == landing_aruco_detection_threshold
                    landing_marker_pose_estimate = mean(landing_aruco_detection_list, 1);
                    landing_marker_detected = true;
                    disp(['Average landing marker pose (world frame): ', mat2str(landing_marker_pose_estimate, 4)]);
                end

            end

        end
    end

    function msgCallback_Pose(msg)
        last_pose_synced = msg;
    end
    
    % === Functions ===
    
    function sendCommand(cmd, target, yaw)
        goalMsg.command_type = cmd;
        goalMsg.target_pose = target;
        goalMsg.yaw = yaw;
        sendGoalAndWait(ac, goalMsg);
    end

    function sendGoalAndWait(ac, goalMsg)
        % Send the goal and get the handle
        goalHandle = sendGoal(ac, goalMsg);
    
        disp('Goal sent. Waiting for result...');
    
        % Block until result is received
        result = getResult(goalHandle);
    
        disp(['Result: ', result.message]);
    end

    function waitForTrajectoryCompletion(pause_time)
        if nargin < 1
            pause_time = 0.1;
        end
        target_mode = 2;
        while true
            pause(pause_time);
            if ~isempty(last_drone_state) && isfield(last_drone_state, 'trajectory_mode')
                if int16(last_drone_state.trajectory_mode) == target_mode
                    disp(['Trajectory mode ', num2str(target_mode), ' reached.']);
                    break;
                end
            end
        end
    end

    function check_if_all_probes_found()
        if ~isempty(last_drone_state) && isfield(last_drone_state, 'probes_found')
            if last_drone_state.probes_found >= probe_search_amount
                all_probes_found = true;
            else
                all_probes_found = false;
            end
        end
    end

    function T = poseMsgToMatrix(poseMsg)
        % Extract position
        px = poseMsg.pose.position.x;
        py = poseMsg.pose.position.y;
        pz = poseMsg.pose.position.z;

        % Extract quaternion
        qx = poseMsg.pose.orientation.x;
        qy = poseMsg.pose.orientation.y;
        qz = poseMsg.pose.orientation.z;
        qw = poseMsg.pose.orientation.w;

        % Convert quaternion to rotation matrix
        R = quat2rotm([qw, qx, qy, qz]); % MATLAB expects [w x y z]

        % Build homogeneous transform
        T = [R [px; py; pz]; 0 0 0 1];
    end


end
