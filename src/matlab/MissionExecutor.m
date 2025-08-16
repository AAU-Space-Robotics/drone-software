function MissionExecutor()
    node = ros2node("/thyra_mission_executor_matlab");
    qos = struct('Reliability', 'besteffort', ...
                 'Durability', 'volatile', ...
                 'History', 'keeplast', ...
                 'Depth', 5);

    % Load camera parameters
    cameraParamsExtracted = load('cameraParams.mat');
    camIntrinsics = cameraParamsExtracted.cameraParams;
    markerSize = 102.15; % in mm
    
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
    landing_marker_pose_estimate_used = [];
    yaw_to_marker_estimate = 0.0; % Initial yaw to look at marker
    yaw_to_marker_estimate_used = 0.0;
    landing_marker_detected = false;
    search_attempts = 0;
    search_attempts_max = 3;
    arUco_landing_tries = 0;
    arUco_marker_landing_correction_threshold = 5;


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

    % Determine landing marker pose estimate
    if  landing_marker_detected
        landing_marker_pose_estimate_xy = [landing_marker_pose_estimate(1:2), search_height];

        if all_probes_found
            disp('All probes and landing marker found. Proceeding to landing.');
        else
            disp('Landing marker found, but not all probes detected.');
        end
    else
        landing_marker_pose_estimate_xy = [0, 0, search_height];
        if all_probes_found 
            disp('All probes found, but landing marker not detected.');
        else
            disp('Neither all probes nor landing marker detected. Using default landing position.');
        end  
    end

    % Goto landing marker location
    goToArUcoMarker(0.1);

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
            return;
        end

        if isempty(last_pose_synced)
            disp('No synced global pose available yet.');
            return;
        end

        T_world_drone = poseMsgToMatrix(last_pose_synced);

        % Parameters for averaging
        aruco_window_size = 5; % Change to 3 if you want last 3

        disp(['Detected markers: ', num2str(ids(:)')]);
        for i = 1:length(ids)
            if ids(i) == landing_marker_id
                % Print pose in camera frame
                disp(['Pose in camera frame (marker ', num2str(ids(i)), '): ', mat2str(tvecs(i,:), 4)]);

                % Marker in drone frame
                pose_in_drone_frame = cameraToDrone * [tvecs(i,:)'; 1];
                % Transform to world frame
                pose_in_world = T_world_drone * pose_in_drone_frame;

                % Add to detection list (keep only last N)
                landing_aruco_detection_list = [landing_aruco_detection_list; pose_in_world(1:3)'];
                if size(landing_aruco_detection_list, 1) > aruco_window_size
                    landing_aruco_detection_list(1, :) = []; % Remove oldest
                end

                % Compute average if enough detections
                if size(landing_aruco_detection_list, 1) == aruco_window_size
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

    function goToArUcoMarker(pause_time)
        marker_threshold = 0.2; % 20 cm
        % Verify the input time 
        if nargin < 1
            pause_time = 0.1;
        end

        yaw_to_marker_estimate = estimate_yaw_to_marker();

        % Send the command to go to the ArUco marker
        sendCommand('goto', landing_marker_pose_estimate_xy, yaw_to_marker_estimate);
        landing_marker_pose_estimate_used = landing_marker_pose_estimate;
        yaw_to_marker_estimate_used = yaw_to_marker_estimate;

        % While waiting for the drone to reach the marker, check if aruco marker is getting a new estimate. 
        while true
            pause(pause_time);

            % Check if the drone has reached the ArUco marker
            if ~isempty(last_drone_state) && isfield(last_drone_state, 'trajectory_mode')
                if int16(last_drone_state.trajectory_mode) == 2 % Assuming 2 is the mode for "goto"
                    disp('Reached the ArUco marker location.');
                    break;
                end
            end

            % Check if a new estimate is available
            if ~isempty(landing_marker_pose_estimate) && ~isequal(landing_marker_pose_estimate, landing_marker_pose_estimate_used) && arUco_landing_tries <= arUco_marker_landing_correction_threshold
                % Check if XY position differs by more than 20 cm
                delta_xy = norm(landing_marker_pose_estimate(1:2) - landing_marker_pose_estimate_used(1:2));
                if delta_xy > marker_threshold
                    disp(['New landing marker XY estimate differs by more than 20 cm (', num2str(delta_xy), ' m). Updating target position.']);
                    marker_xy_zero_z = [landing_marker_pose_estimate(1:2), search_height];
                    yaw_to_marker_estimate = estimate_yaw_to_marker();
                    landing_marker_pose_estimate_used = landing_marker_pose_estimate;
                    yaw_to_marker_estimate_used = yaw_to_marker_estimate;
                    sendCommand('goto', marker_xy_zero_z, yaw_to_marker_estimate_used);
                    arUco_landing_tries = arUco_landing_tries + 1;
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
   
    function yaw = estimate_yaw_to_marker()
        % Estimate orientation to look at marker
        if isempty(last_pose_synced) || isempty(landing_marker_pose_estimate)
            yaw = NaN;
            disp('Cannot estimate yaw: missing pose or marker estimate.');
            return;
        end
        T_world_drone = poseMsgToMatrix(last_pose_synced);
        drone_pos = T_world_drone(1:3,4)';
        marker_pos = landing_marker_pose_estimate(1:3);
        vec_to_marker = marker_pos - drone_pos;
        yaw = atan2(vec_to_marker(2), vec_to_marker(1));
        if yaw < 0
            yaw = yaw + 2*pi;
        end
        disp(['Suggested yaw to look at marker (rad): ', num2str(yaw, 4)]);
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
