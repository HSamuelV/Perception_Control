% Initialize ROS node

% Create a subscriber for the LIDAR data
pcSub = rossubscriber('/lidar_ust', 'sensor_msgs/LaserScan');

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
Pxyz = [Px,Py,Pz];
idx = dbscan(Pxyz,epsilon,min_pts);

% Unique cluster labels
uniqueLabels = unique(idx);

% Create a figure
figure;
hold on;
grid on;

% Plot all points
scatter3(Px, Py, Pz, 20, idx, 'filled');

% Generate and plot the convex hull for each cluster
for i = 1:length(uniqueLabels)
    clusterLabel = uniqueLabels(i);
    if clusterLabel == -1
        % Skip noise points
        continue;
    end

    % Extract points belonging to the current cluster
    clusterPoints = Pxyz(idx == clusterLabel, :);

    % Compute the convex hull
    K = convhull(clusterPoints(:,1), clusterPoints(:,2));

    % Plot the convex hull
    plot(clusterPoints(K,1), clusterPoints(K,2), 'LineWidth', 2);
    
    hull_points = Pxyz(K,:);
    
    % Calculate the half-space representation
    % Each edge of the convex hull defines a half-space
    num_edges = length(K);
    A = zeros(num_edges, 2);
    B = zeros(num_edges, 1);
    A_ = [];
    B_ = [];

    for i = 1:num_edges
        % Get the start and end points of the edge
        p1 = hull_points(i, :);
        p2 = hull_points(mod(i, num_edges) + 1, :);

        % Calculate the line equation coefficients
        a = p2(2) - p1(2); % y2 - y1
        b = p1(1) - p2(1); % x1 - x2
        c = a * p1(1) + b * p1(2); % - (a * x1 + b * y1)

        % Store the half-space representation Ax >= B
        A(i, :) = [a, b];
        B(i) = -c;
    end
end

% Add labels and title for the LIDAR plot
xlabel('X [m]');
ylabel('Y [m]');
title('DBSCAN Clustering with Convex Hulls and Robot Position');

hold off;




% % Define the robot's position (x, y)
% robotPos = [2, 3]; % Example position in 2D
% 
% % Define the size of the robot (width and height)
% robotSize = [1.5, 1]; % Width = 1.5 meters, Height = 1 meter
% 
% % Create a figure
% figure;
% hold on;
% 
% % Plot the robot as a rectangle
% rectangle('Position', [robotPos - robotSize/2, robotSize], 'Curvature', [0.2, 0.2], 'FaceColor', [0 0.5 0.5]);
% 
% % Plot the robot's position as a point (optional)
% plot(robotPos(1), robotPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% 
% % Set axis limits and labels
% axis equal;
% xlim([0 10]); ylim([0 10]); % Adjust based on your map
% xlabel('X [m]'); ylabel('Y [m]');
% title('Robot Position and Size');
% grid on;
