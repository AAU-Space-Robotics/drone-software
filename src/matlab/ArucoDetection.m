% Load camera parameters
cameraParamsExtracted = load('cameraParams.mat');
camIntrinsics = cameraParamsExtracted.cameraParams;
markerSize = 150; % in mm

% Define CAMERA_TO_DRONE_TRANSFORM (4x4 homogeneous matrix)
cameraToDrone = [...
    0.0, -0.70710678, 0.70710678, 0.137751; ...
    1.0, 0.0, 0.0, -0.018467; ...
    0.0, 0.70710678, 0.70710678, 0.12126; ...
    0.0, 0.0, 0.0, 1.0];

% Global variable for latest drone pose
global latestDronePoseMsg;
latestDronePoseMsg = [];

% Define functions
function [ids, tvecs, rvecs] = DetectAruco(I, camIntrinsics, markerSize)
    [ids, locs] = readArucoMarker(I);
    
    if isempty(ids)
        ids = [];
        tvecs = [];
        rvecs = [];
        return;
    end

    numMarkers = size(locs, 3);
    objectPoints = [...
        markerSize/2, markerSize/2;
        markerSize/2, markerSize/2;
        markerSize/2, markerSize/2;
        markerSize/2, markerSize/2];

    tvecs = zeros(numMarkers, 3);
    rvecs = zeros(numMarkers, 3);
    
    for i = 1:numMarkers
        imagePoints = squeeze(locs(:,:,i)); % 4x2
        
        % Use MATLAB's solvePnP-style function:
        [R, t] = extrinsics(imagePoints, objectPoints, camIntrinsics);

        rvec = rotationMatrixToVector(R);
        tvecs(i, :) = t;
        rvecs(i, :) = rvec;
    end
end

function imageCallback(~, imgMsg, pub, camIntrinsics, markerSize, cameraToDrone)
    global latestDronePoseMsg;

    disp('Received image message');
    
    if isempty(latestDronePoseMsg)
        return; % No drone pose received yet
    end
    
    % Decompress the image
    image = readImage(imgMsg);
    
    [ids, tvecs, rvecs] = DetectAruco(image, camIntrinsics, markerSize);
    
    if isempty(ids)
        disp('No marker detected')
        return;
    end

    disp('Marker detected')

    % Get drone pose
    dronePose = latestDronePoseMsg.Pose;
    dronePos = [dronePose.Position.X; dronePose.Position.Y; dronePose.Position.Z];
    droneQuat = [dronePose.Orientation.W, dronePose.Orientation.X, dronePose.Orientation.Y, dronePose.Orientation.Z];
    R_drone_to_global = quat2rotm(droneQuat);
    T_drone_to_global = [R_drone_to_global, dronePos; 0, 0, 0, 1];

    % Compute T_camera_to_global = T_drone_to_global * cameraToDrone
    T_camera_to_global = T_drone_to_global * cameraToDrone;
    R_camera_to_global = T_camera_to_global(1:3, 1:3);
    t_camera_to_global = T_camera_to_global(1:3, 4);

    markerArray = ros2message('visualization_msgs/MarkerArray');
    
    for i = 1:length(ids)
        t = tvecs(i, :)' / 1000; % Convert to meters, column vector
        rvec = rvecs(i, :);
        R_marker_to_camera = rotationVectorToMatrix(rvec);
        
        % Compute position in global
        pos_global = R_camera_to_global * t + t_camera_to_global;
        
        % Compute rotation in global: R_marker_to_global = R_camera_to_global * R_marker_to_camera
        R_marker_to_global = R_camera_to_global * R_marker_to_camera;
        quat_global = rotm2quat(R_marker_to_global);
        
        marker = ros2message('visualization_msgs/Marker');
        marker.Header.FrameId = latestDronePoseMsg.Header.FrameId;
        marker.Header.Stamp = imgMsg.Header.Stamp; % Use image stamp for consistency
        marker.Ns = 'aruco_markers';
        marker.Id = ids(i);
        marker.Type = marker.CUBE;
        marker.Action = marker.ADD;
        marker.Scale.X = markerSize / 1000;
        marker.Scale.Y = markerSize / 1000;
        marker.Scale.Z = 0.01;
        marker.Color.R = 0.5;
        marker.Color.G = 0.5;
        marker.Color.B = 0.5;
        marker.Color.A = 1.0;
        
        marker.Pose.Position.X = pos_global(1);
        marker.Pose.Position.Y = pos_global(2);
        marker.Pose.Position.Z = pos_global(3);
        
        marker.Pose.Orientation.X = quat_global(2);
        marker.Pose.Orientation.Y = quat_global(3);
        marker.Pose.Orientation.Z = quat_global(4);
        marker.Pose.Orientation.W = quat_global(1);
        
        markerArray.Markers(i) = marker;

        disp('Published marker');

    end
    
    send(pub, markerArray);
end

function poseCallback(~, msg)
    global latestDronePoseMsg;
    latestDronePoseMsg = msg;
end

disp('Starting script');

% Create ROS2 node
node = ros2node('/aruco_detector', 0);
cleanupObj = onCleanup(@() cleanupROS(node));

% Define cleanup function
function cleanupROS(node)
    disp('Cleaning up ROS2 node...');
    delete(node);  % Explicitly delete node
    pause(0.5);    % Allow time for cleanup
end
disp('Node created');

% Create publisher
pub = ros2publisher(node, '/aruco_markers', 'visualization_msgs/MarkerArray');
disp('Publisher created');

% Create subscribers with proper topic verification
topics = ros2("topic", "list");
imageTopic = '/thyra/out/color_image/compressed';
poseTopic = '/thyra/out/pose/synced_with_RGBD';

% Check if topics exist before creating subscribers
if ~any(strcmp(topics, imageTopic))
    error(['Image topic not found: ', imageTopic]);
end
if ~any(strcmp(topics, poseTopic))
    error(['Pose topic not found: ', poseTopic]);
end

% Create subscribers with full callback setup
sub_img = ros2subscriber(node, imageTopic, 'sensor_msgs/CompressedImage', ...
    @(src, msg) imageCallback(src, msg, pub, camIntrinsics, markerSize, cameraToDrone), ...
    'Reliability', 'besteffort', 'Durability', 'volatile', 'Depth', 5);

sub_pose = ros2subscriber(node, poseTopic, 'geometry_msgs/PoseStamped', ...
    @poseCallback, ...
    'Reliability', 'besteffort', 'Durability', 'volatile', 'Depth', 10);

disp('Subscribers created successfully');
disp('ROS2 ArUco detection node is running');

% Run ROS2 node
disp('ROS2 ArUco detection node is running');
while true
    pause(0.1);
end

