classdef SegmentationNode < handle
    % SegmentationNode MATLAB class for ROS2-based probe detection
    
    properties
        node
        rgb_sub
        depth_sub
        image_publisher
        probe_publisher
        rgb_image
        depth_image
        previous_time
        fps
        confidence_threshold
        model % Placeholder for YOLO model
        model_path
        bridge % Placeholder for CvBridge-like functionality
    end
    
    methods
        function obj = SegmentationNode()
            % Constructor: Initialize ROS2 node and setup subscribers/publishers
            obj.node = ros2node('/segmentation_node');
            obj.previous_time = 0;
            obj.fps = 1;
            obj.confidence_threshold = 0.75;
            obj.rgb_image = [];
            obj.depth_image = [];
            
            % Subscribers for new compressed image topics
            obj.rgb_sub = ros2subscriber(obj.node, ...
                '/camera/camera/color/image_raw/compressed', ...
                'sensor_msgs/CompressedImage', ...
                @(msg) obj.imageCallback(msg, 'rgb'));
            obj.depth_sub = ros2subscriber(obj.node, ...
                '/camera/camera/depth/image_rect_raw/compressed', ...
                'sensor_msgs/CompressedImage', ...
                @(msg) obj.imageCallback(msg, 'depth'));
            
            % Publishers
            obj.image_publisher = ros2publisher(obj.node, ...
                '/probe_detector/segmented_image', ...
                'sensor_msgs/Image');
            obj.probe_publisher = ros2publisher(obj.node, ...
                '/probe_detector/probe_locations', ...
                'interfaces/ProbeLocations');
            
            % Initialize YOLO model (assumes model is converted to MATLAB format)
            package_name = 'probe_detection';
            try
                % Note: MATLAB does not have a direct equivalent to get_package_share_directory
                % Assume model is in a known path or configure manually
                obj.model_path = fullfile(pwd, 'models', 'YOLO11m.pt');
                if ~isfile(obj.model_path)
                    error('Model file not found at: %s', obj.model_path);
                end
                % Placeholder: Load YOLO model (requires Deep Learning Toolbox or custom implementation)
                % Example: obj.model = importONNXNetwork(obj.model_path);
                fprintf('Loaded YOLO model from %s\n', obj.model_path);
            catch e
                fprintf('Failed to initialize model: %s\n', e.message);
                rethrow(e);
            end
        end
        
        function imageCallback(obj, msg, type)
            % Callback for synchronized RGB and depth images
            persistent rgb_msg depth_msg rgb_time depth_time
            
            try
                % Store message and timestamp
                current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                if strcmp(type, 'rgb')
                    rgb_msg = msg;
                    rgb_time = current_time;
                else
                    depth_msg = msg;
                    depth_time = current_time;
                end
                
                % Check if both messages are available and synchronized (within 0.1s)
                if ~isempty(rgb_msg) && ~isempty(depth_msg) && abs(rgb_time - depth_time) < 0.1
                    % Skip frames if time difference is too small
                    if obj.previous_time + (1 / obj.fps) > rgb_time
                        return;
                    else
                        obj.previous_time = rgb_time;
                    end
                    
                    fprintf('Synchronized RGB Time: %f, Depth Time: %f\n', rgb_time, depth_time);
                    
                    % Convert compressed RGB image
                    rgb_data = uint8(msg.data);
                    obj.rgb_image = imdecode(rgb_data, 'Color'); % Requires Image Processing Toolbox
                    
                    % Convert compressed depth image
                    depth_data = uint8(depth_msg.data);
                    obj.depth_image = imdecode(depth_data, 'Grayscale'); % Depth as grayscale image
                    
                    % Run YOLO inference
                    [probe_boxes, probe_masks, probe_confidences] = obj.infer_yolo(obj.rgb_image);
                    
                    % Process detected probes
                    if ~isempty(probe_masks)
                        point_cloud = obj.compressed_depth_to_array(obj.depth_image);
                        probe_locations = obj.compute_probe_locations(probe_masks, point_cloud, ...
                            obj.rgb_image, probe_confidences);
                        
                        if ~isempty(probe_locations)
                            % Sort by confidence
                            [~, idx] = sort([probe_locations.confidence], 'descend');
                            sorted_locations = probe_locations(idx);
                            
                            % Prepare ProbeLocations message
                            probe_msg = ros2message('interfaces/ProbeLocations');
                            probe_msg.header = rgb_msg.header;
                            probe_msg.classification_confidence = single([sorted_locations.confidence]);
                            probe_msg.num_probes = length(sorted_locations);
                            probe_msg.probes = single([sorted_locations.x, sorted_locations.y, sorted_locations.z]);
                            probe_msg.centroid_x = single([sorted_locations.centroid_x]);
                            probe_msg.centroid_y = single([sorted_locations.centroid_y]);
                            
                            % Publish probe locations
                            publish(obj.probe_publisher, probe_msg);
                            fprintf('Published %d probe locations\n', probe_msg.num_probes);
                        end
                        
                        % Clear stored messages
                        rgb_msg = [];
                        depth_msg = [];
                    end
                end
            catch e
                fprintf('Error in imageCallback: %s\n', e.message);
            end
        end
        
        function probe_locations = compute_probe_locations(obj, probe_masks, point_cloud, rgb_image, probe_confidences)
            % Compute 3D locations of probes using depth data
            probe_locations = struct('centroid_x', {}, 'centroid_y', {}, 'confidence', {}, ...
                'x', {}, 'y', {}, 'z', {});
            
            fprintf('RGB Image Size: [%d %d %d]\n', size(rgb_image));
            fprintf('Depth Image Size: [%d %d %d]\n', size(point_cloud));
            
            % Extract x, y, z from point cloud
            x_image = point_cloud(:, :, 1);
            y_image = point_cloud(:, :, 2);
            z_image = point_cloud(:, :, 3);
            
            for i = 1:length(probe_masks)
                try
                    mask = probe_masks{i};
                    % Resize mask to match depth image
                    mask = imresize(mask, [size(x_image, 1), size(x_image, 2)], 'Method', 'bilinear');
                    binary_mask = mask > 0.5;
                    
                    % Compute centroid
                    [y, x] = find(binary_mask);
                    if isempty(x) || isempty(y)
                        continue;
                    end
                    centroid_x = round(mean(x));
                    centroid_y = round(mean(y));
                    
                    % Extract 3x3 kernel around centroid
                    kernel_size = 3;
                    half_kernel = floor(kernel_size / 2);
                    x_min = max(centroid_x - half_kernel, 1);
                    x_max = min(centroid_x + half_kernel, size(x_image, 2));
                    y_min = max(centroid_y - half_kernel, 1);
                    y_max = min(centroid_y + half_kernel, size(x_image, 1));
                    kernel = binary_mask(y_min:y_max, x_min:x_max);
                    
                    % Extract positions in kernel
                    position_kernel = cat(3, x_image(y_min:y_max, x_min:x_max), ...
                        y_image(y_min:y_max, x_min:x_max), ...
                        z_image(y_min:y_max, x_min:x_max));
                    
                    % Calculate median of valid points
                    valid_x = position_kernel(:, :, 1);
                    valid_y = position_kernel(:, :, 2);
                    valid_z = position_kernel(:, :, 3);
                    valid_x = valid_x(kernel & isfinite(valid_x));
                    valid_y = valid_y(kernel & isfinite(valid_y));
                    valid_z = valid_z(kernel & isfinite(valid_z));
                    
                    median_x = median(valid_x, 'omitnan');
                    median_y = median(valid_y, 'omitnan');
                    median_z = median(valid_z, 'omitnan');
                    
                    if isnan(median_x) || isnan(median_y) || isnan(median_z) || ...
                            isinf(median_x) || isinf(median_y) || isinf(median_z)
                        continue;
                    end
                    
                    % Store probe location
                    probe_locations(end+1) = struct(...
                        'centroid_x', centroid_x, ...
                        'centroid_y', centroid_y, ...
                        'confidence', probe_confidences(i), ...
                        'x', median_x, ...
                        'y', median_y, ...
                        'z', median_z);
                catch e
                    fprintf('Error processing mask %d: %s\n', i, e.message);
                    continue;
                end
            end
            
            % Publish annotated RGB image
            resized_rgb = imresize(rgb_image, [size(x_image, 1), size(x_image, 2)]);
            for i = 1:length(probe_locations)
                loc = probe_locations(i);
                % Draw red dot at centroid
                resized_rgb = insertShape(resized_rgb, 'Circle', ...
                    [loc.centroid_x, loc.centroid_y, 3], 'Color', 'red', 'LineWidth', 1);
                % Add text with confidence and coordinates
                text_str = sprintf('conf=%.2f, x=%.2f, y=%.2f, z=%.2f', ...
                    loc.confidence, loc.x, loc.y, loc.z);
                resized_rgb = insertText(resized_rgb, [loc.centroid_x + 5, loc.centroid_y - 5], ...
                    text_str, 'FontSize', 12, 'TextColor', 'white');
            end
            
            % Publish segmented image
            try
                img_msg = ros2message('sensor_msgs/Image');
                img_msg.header = rgb_msg.header;
                img_msg.height = size(resized_rgb, 1);
                img_msg.width = size(resized_rgb, 2);
                img_msg.encoding = 'bgr8';
                img_msg.data = reshape(permute(resized_rgb, [3, 1, 2]), [], 1);
                publish(obj.image_publisher, img_msg);
            catch e
                fprintf('Failed to publish segmented image: %s\n', e.message);
            end
        end
        
        function [probe_boxes, probe_masks, probe_confidences] = infer_yolo(obj, rgb_image)
            % Placeholder for YOLO inference
            % Requires Deep Learning Toolbox or custom implementation
            probe_boxes = [];
            probe_masks = {};
            probe_confidences = [];
            
            try
                % Example placeholder (replace with actual YOLO implementation)
                % results = obj.model.predict(rgb_image, 'imgsz', 640, 'conf', obj.confidence_threshold);
                % For demo, assume dummy detections
                results = struct('boxes', [], 'masks', [], 'conf', []);
                
                if ~isempty(results.boxes)
                    for i = 1:length(results.boxes)
                        box = results.boxes(i).xyxy;
                        mask = results.masks(i).data;
                        mask = imresize(mask, [size(rgb_image, 1), size(rgb_image, 2)]);
                        probe_boxes = [probe_boxes; box];
                        probe_masks{end+1} = mask;
                        probe_confidences = [probe_confidences; results.conf(i)];
                    end
                end
            catch e
                fprintf('Error in YOLO inference: %s\n', e.message);
            end
        end
        
        function point_cloud = compressed_depth_to_array(obj, depth_image)
            % Convert compressed depth image to 3D point cloud array
            % Note: Assumes depth_image is in meters (float32) encoded as grayscale
            [height, width] = size(depth_image);
            point_cloud = zeros(height, width, 3, 'single');
            
            % Placeholder: Convert depth to 3D points (requires camera parameters)
            % For simplicity, assume depth_image is in meters and assign dummy x, y
            [X, Y] = meshgrid(1:width, 1:height);
            point_cloud(:, :, 1) = X; % Placeholder x-coordinates
            point_cloud(:, :, 2) = Y; % Placeholder y-coordinates
            point_cloud(:, :, 3) = single(depth_image); % Depth (z)
            
            % Actual conversion requires camera intrinsics and depth scaling
            fprintf('Warning: Depth to point cloud conversion is a placeholder\n');
        end
    end
end

% Main script to run the node
function main()
    ros2('init');
    try
        node = SegmentationNode();
        % MATLAB does not have a direct equivalent to rclpy.spin
        % Run a loop to keep the node alive
        while true
            pause(0.1); % Process callbacks
        end
    catch e
        fprintf('Error in main: %s\n', e.message);
    end
    ros2('shutdown');
end