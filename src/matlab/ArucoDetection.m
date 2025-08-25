function ArucoDetector()
    % Initialize ROS2 node
    node = ros2node("/aruco_detector");

    % Quality of Service settings
    qos = struct('Reliability', 'besteffort', ...
                 'Durability', 'volatile', ...
                 'History', 'keeplast', ...
                 'Depth', 5);

    % Load camera parameters
    cameraParamsExtracted = load('cameraParams.mat');
    camIntrinsics = cameraParamsExtracted.cameraParams;
    markerSize = 144.0; % in mm

    % Define CAMERA_TO_DRONE_TRANSFORM (4x4 homogeneous matrix) in meters
    cameraToDrone = [...
        0.0, -0.70710678, 0.70710678, 0.137751; ...
        1.0, 0.0, 0.0, -0.018467; ...
        0.0, 0.70710678, 0.70710678, 0.12126; ...
        0.0, 0.0, 0.0, 1.0];

    % Subscribers
    sub_RGB_image = ros2subscriber(node, "/thyra/out/color_image/compressed", ...
                                   "sensor_msgs/CompressedImage", @msgCallback_RGB, qos);

    disp('ArUco Detector started. Waiting for RGB images...');

    % Callback for RGB image
    function msgCallback_RGB(msg)
        rgb_image = rosReadImage(msg);

        % Detect ArUco markers
        [ids, tvecs, rvecs] = detectArucoInCameraFrame(rgb_image, camIntrinsics, markerSize);

        if isempty(ids)
            disp('No ArUco markers detected.');
            return;
        end

        

        % Print detected marker poses
        disp(['Detected markers: ', num2str(ids(:)')]);
        for i = 1:length(ids)
            pose_in_drone_frame = cameraToDrone * [tvecs(i,:)'; 1];
            %disp(['Marker ID: ', num2str(ids(i)), ', Pose in camera frame (mm): ', mat2str(tvecs(i,:), 4)]);
            disp(['Marker ID: ', num2str(ids(i)), ', Pose in drone frame (m): ', mat2str(pose_in_drone_frame(1:3), 4)]);
        end
    end

    % Keep the program running to process callbacks
    while true
        pause(0.1); % Adjust as needed to reduce CPU usage
    end
end