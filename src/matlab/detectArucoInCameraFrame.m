function [ids, tvecs, rvecs] = detectArucoInCameraFrame(Image, camIntrinsics, markerSize)
    % Ensure uint8 RGB
    if ~isa(Image, 'uint8')
        Image = im2uint8(Image);
    end
    if size(Image,3) ~= 3
        error('Input must be a color image with 3 channels.');
    end

    % Detect markers
    try
        [ids, locs] = readArucoMarker(Image, ...
            'RefineCorners', true, ...
            'ResolutionPerBit', 20, ...
            'WindowSizeStep', 2, ...
            'WindowSizeRange', [3 10], ...
            'RefinementMaxIterations', 100, ...
            'RefinementTolerance', 0.01);
    catch ME
        warning('Aruco detection failed: %s', string(ME.message));
        ids = [];
        tvecs = [];
        rvecs = [];
        return;
    end

    if isempty(ids)
        ids = [];
        tvecs = [];
        rvecs = [];
        return;
    end

    numMarkers = size(locs, 3);

    objectPoints = [...
        -markerSize/2,  markerSize/2, 0;
         markerSize/2,  markerSize/2, 0;
         markerSize/2, -markerSize/2, 0;
        -markerSize/2, -markerSize/2, 0];

    tvecs = zeros(numMarkers, 3);
    rvecs = zeros(numMarkers, 3);

    for i = 1:numMarkers
        imagePoints = squeeze(locs(:,:,i));

        try
            % Robust PnP
            [worldOrientation, worldLocation] = estimateWorldCameraPose(...
                imagePoints, objectPoints, camIntrinsics, ...
                'MaxReprojectionError', 3, ...
                'Confidence', 98, ...
                'MaxNumTrials', 2000);

            % Convert pose
            R = worldOrientation';
            t = -worldLocation * R;  % Camera to marker
            rvec = rotationMatrixToVector(R);
            tvecs(i,:) = t / 1000;  % Convert mm to meters
            rvecs(i,:) = rvec;

        catch ME
            % failure - assign NaN values
            tvecs(i,:) = [NaN NaN NaN];
            rvecs(i,:) = [NaN NaN NaN];
        end
    end
end