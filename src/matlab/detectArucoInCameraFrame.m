function [ids, tvecs, rvecs] = detectArucoInCameraFrame(Image, camIntrinsics, markerSize)
    % Ensure uint8 RGB
    if ~isa(Image, 'uint8')
        Image = im2uint8(Image);
    end
    if size(Image,3) ~= 3
        error('Input must be a color image with 3 channels.');
    end

    % Optional: Resize for speed if huge
    if size(Image,1) > 1080
        scaleFactor = 1080 / size(Image,1);
        Image = imresize(Image, scaleFactor);
    end

    % Detect markers
    try
        [ids, locs] = readArucoMarker(Image);
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
        [R, t] = extrinsics(imagePoints, objectPoints(:,1:2), camIntrinsics);
        rvec = rotationMatrixToVector(R);
        tvecs(i, :) = t / 1000;
        rvecs(i, :) = rvec;
    end
end
