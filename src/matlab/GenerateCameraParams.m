% Program to load, display, and optionally regenerate cameraParams.mat

% Specify the path to the cameraParams.mat file
matFilePath = fullfile(pwd, 'src', 'matlab', 'cameraParams.mat');

% Step 1: Check if cameraParams.mat exists and load it
if isfile(matFilePath)
    fprintf('Loading %s...\n', matFilePath);
    data = load(matFilePath);
    
    % Step 2: Display the contents
    fprintf('\nContents of %s:\n', matFilePath);
    fieldNames = fieldnames(data);
    for i = 1:length(fieldNames)
        fieldName = fieldNames{i};
        fprintf('Field: %s\n', fieldName);
        
        % Check if the field is a cameraIntrinsics object
        if isa(data.(fieldName), 'cameraIntrinsics')
            intrinsics = data.(fieldName);
            fprintf('  Camera Intrinsics:\n');
            fprintf('    Focal Length: [%.6f, %.6f]\n', intrinsics.FocalLength(1), intrinsics.FocalLength(2));
            fprintf('    Principal Point: [%.6f, %.6f]\n', intrinsics.PrincipalPoint(1), intrinsics.PrincipalPoint(2));
            fprintf('    Image Size: [%d, %d]\n', intrinsics.ImageSize(1), intrinsics.ImageSize(2));
            fprintf('    Radial Distortion: [%.6f, %.6f, %.6f]\n', intrinsics.RadialDistortion);
            fprintf('    Tangential Distortion: [%.6f, %.6f]\n', intrinsics.TangentialDistortion);
        
        % Check if the field is a cameraParameters object
        elseif isa(data.(fieldName), 'cameraParameters')
            cameraParams = data.(fieldName);
            fprintf('  Camera Parameters:\n');
            if ~isempty(cameraParams.Intrinsics)
                fprintf('    Intrinsics:\n');
                fprintf('      Focal Length: [%.6f, %.6f]\n', cameraParams.Intrinsics.FocalLength(1), cameraParams.Intrinsics.FocalLength(2));
                fprintf('      Principal Point: [%.6f, %.6f]\n', cameraParams.Intrinsics.PrincipalPoint(1), cameraParams.Intrinsics.PrincipalPoint(2));
                fprintf('      Image Size: [%d, %d]\n', cameraParams.Intrinsics.ImageSize(1), cameraParams.Intrinsics.ImageSize(2));
                fprintf('      Radial Distortion: [%.6f, %.6f, %.6f]\n', cameraParams.Intrinsics.RadialDistortion);
                fprintf('      Tangential Distortion: [%.6f, %.6f]\n', cameraParams.Intrinsics.TangentialDistortion);
            else
                fprintf('    Intrinsics: Not available\n');
            end
            fprintf('    Rotation Matrix:\n');
            if ~isempty(cameraParams.RotationMatrices)
                disp(cameraParams.RotationMatrices);
            else
                fprintf('      Not available\n');
            end
            fprintf('    Translation Vector:\n');
            if ~isempty(cameraParams.TranslationVectors)
                fprintf('      [%.6f, %.6f, %.6f]\n', cameraParams.TranslationVectors);
            else
                fprintf('      Not available\n');
            end
        
        % Handle structs or other types
        elseif isstruct(data.(fieldName))
            fprintf('  Field type: struct\n');
            structFields = fieldnames(data.(fieldName));
            for j = 1:length(structFields)
                subField = structFields{j};
                fprintf('    %s: \n', subField);
                disp(data.(fieldName).(subField));
            end
        else
            fprintf('  Field type: %s\n', class(data.(fieldName)));
            disp('  Contents:');
            disp(data.(fieldName));
        end
    end
else
    fprintf('File %s does not exist.\n', matFilePath);
end

% Step 3: Prompt user to generate a new cameraParams.mat
prompt = sprintf('\nDo you want to generate a new %s with Intel RealSense D435 parameters? (y/n): ', matFilePath);
response = input(prompt, 's');

if lower(response) == 'y'
    % Define Intel RealSense D435 parameters
    focalLength = [425.8813171386719, 425.8813171386719]; % [fx, fy]
    principalPoint = [430.5101623535156, 238.53343200683594]; % [cx, cy]
    %imageSize = [640, 480]; % Actual camera resolution
    imageSize = [1280, 720]; % Test image resolution
    radialDistortion = [0.0, 0.0, 0.0]; % [k1, k2, k3]
    tangentialDistortion = [0.0, 0.0]; % [p1, p2]
    
    % Create cameraIntrinsics object
    cameraParams = cameraIntrinsics(focalLength, principalPoint, imageSize, ...
        'RadialDistortion', radialDistortion, ...
        'TangentialDistortion', tangentialDistortion);
    
    % Save to cameraParams.mat (overwrites existing file)
    fprintf('Generating new %s...\n', matFilePath);
    save(matFilePath, 'cameraParams'); % Save as 'cameraParams' to match existing pipeline
    
    fprintf('New %s saved with RealSense D435 parameters.\n', matFilePath);
else
    fprintf('No changes made to %s.\n', matFilePath);
end