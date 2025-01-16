% Initialize ROS node
% (Ensure ROS node is initialized properly in your environment)

% Create a subscriber for the LIDAR data
pcSub = rossubscriber('/lidar_points', 'sensor_msgs/PointCloud2');

% Receive the data
pointCloudMsg = receive(pcSub);
ptCloud = readXYZ(pointCloudMsg);

% Define Z range (e.g., filter out points that are too close to the floor)
minZ = 0.1;   % Minimum height in meters (adjust according to your scenario)
maxZ = 0.6;   % Maximum height in meters (adjust according to your scenario)

% Apply Z filter
validZIdx = (ptCloud(:, 3) >= minZ) & (ptCloud(:, 3) <= maxZ);

% Filter the point cloud based on Z values
validPoints = ptCloud(validZIdx, :);

% Optional: Filter based on distance too (to remove noise points)
distances = sqrt(sum(validPoints.^2, 2));  % Calculate distances
minRange = 0.1;   % LIDAR minimum range
maxRange = 20.0; % LIDAR maximum range
validRangeIdx = (distances >= minRange) & (distances <= maxRange);

% Apply distance filter
validPoints = validPoints(validRangeIdx, :);

Pontos = double(validPoints);
epsilon = 0.67;
min_pts = 3;
Px = Pontos(:,1);
Py = Pontos(:,2);
Pz = Pontos(:,3);
Pxy = [Px, Py];
idx = dbscan(Pxy, epsilon, min_pts);

% Unique cluster labels
uniqueLabels = unique(idx);

% Initialize cell array to store convex hulls
hullData = cell(length(uniqueLabels), 1);

% Create a figure
figure;
hold on;
grid on;

% Plot all points
scatter(Px, Py, 20, idx, 'filled');

% Generate and plot the convex hull for each cluster
for i = 1:length(uniqueLabels)
    clusterLabel = uniqueLabels(i);
    if clusterLabel == -1
        % Skip noise points
        continue;
    end

    % Extract points belonging to the current cluster
    clusterPoints = Pxy(idx == clusterLabel, :);

    % Compute the convex hull
    K = convhull(clusterPoints(:,1), clusterPoints(:,2));
    
    % Save convex hull data
    hullData{i}.vertices = clusterPoints(K, :);
    hullData{i}.faces = K;
    
    % Plot the convex hull
    plot(clusterPoints(K,1), clusterPoints(K,2), 'LineWidth', 2);
end

% Add labels and title for the LIDAR plot
xlabel('X [m]');
ylabel('Y [m]');
title('DBSCAN Clustering with Convex Hulls and Robot Position');

hold off;

% Generate the matrix describing the polytope
polytopeFaces = [];
for i = 1:length(hullData)
    if isempty(hullData{i}.faces)
        continue;
    end
    K = hullData{i}.faces;
    numFaces = length(K);
    if numFaces > 0
        faces = [K(:), circshift(K(:), -1)];
        polytopeFaces = [polytopeFaces; faces];
    end
end

% Display the matrix describing the polytope (faces)
disp('Matrix describing the polytope (faces):');
disp(polytopeFaces);
