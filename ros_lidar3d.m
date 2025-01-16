%Variables
robot_length = 0.99;  % Adjust as per your robot's dimensions
robot_width = 0.67;   % Adjust as per your robot's dimensions

% Define Z range (e.g., filter out points that are too close to the floor)
minZ = 0.05;   % Minimum height in meters (adjust according to your scenario)
maxZ = 0.7;   % Maximum height in meters (adjust according to your scenario)

minRange = 0.0;   % LIDAR minimum range
maxRange = 60.0; % LIDAR maximum range

epsilon = 0.67; %for dbscan
min_pts = 3; %for dbscan

% Create a subscriber for the LIDAR data
pcSub = rossubscriber('/lidar_3d', 'sensor_msgs/PointCloud2');
%pos_rob = rossubscriber('/husky_velocity_controller/odom','nav_msgs/Odometry');
state = rossubscriber('/gazebo/model_states','gazebo_msgs/ModelStates');

% Create a figure for the plot
figure();


% Como passar a posiçao local para a global. 
% para a posiçao do robo nao ser 0 e a dos objetos ser
% em relaçao a ele.


while(1)
    % Receive the data

    rob_msg = receive(state);
    pointCloudMsg = receive(pcSub);
    ptCloud = readXYZ(pointCloudMsg); %tentar rosReadXYZ


    % Define the robot's position (x, y) and orientation
    robotPos = rob_msg.Pose(37,1).Position; % Example position in 2D
    posx = robotPos.X;
    posy = robotPos.Y;

    % Apply Z filter
    validZIdx = (ptCloud(:, 3) >= minZ) & (ptCloud(:, 3) <= maxZ);

    % Filter the point cloud based on Z values
    validPoints = ptCloud(validZIdx, :);

    % Optional: Filter based on distance too (to remove noise points)
    distances = sqrt(sum(validPoints.^2, 2));  % Calculate distances
    validRangeIdx = (distances >= minRange) & (distances <= maxRange);

    % Apply distance filter
    validPoints = validPoints(validRangeIdx, :);
    
    %Define the points for the cluster
    Pontos = double(validPoints);
    Px = Pontos(:,1) + posx;
    Py = Pontos(:,2) + posy;
    Pz = Pontos(:,3);
    Pxyz = [Px,Py];
    idx = dbscan(Pxyz,epsilon,min_pts);

    % Unique cluster labels
    uniqueLabels = unique(idx); %creates a unique label for each cluster

    % Plot all points
    gscatter(Px, Py, idx);
    hold on;

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

        %Get the hull points to generate the matrixes
        hull_points = Pxyz(K,:);

        % Calculate the half-space representation
        % Each edge of the convex hull defines a half-space
        num_edges = length(K) - 1;
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

            % Store the half-space representation Ax <= B
            A(i, :) = -[a, b];
            B(i) = c;
        end
    end

    % Add labels and title for the LIDAR plot
    xlabel('X [m]');
    ylabel('Y [m]');
    title('DBSCAN Clustering with Convex Hulls and Robot Position');

    orientation = rob_msg.Pose(37,1).Orientation;
    q = [orientation.W, orientation.X, orientation.Y, orientation.Z];
    yaw = quat2eul(q);  % Convert quaternion to Euler angles and extract yaw

    % Plot the robot as an ellipse
    t = linspace(0, 2*pi, 100);  % Parametric angles for the ellipse
    % Ellipse shape, centered at (x, y) with orientation given by yaw
    x_ellipse = (robot_length/2) * cos(t);
    y_ellipse = (robot_width/2) * sin(t);

    % Rotate the ellipse based on the robot's yaw
    R = [cos(yaw(1)) -sin(yaw(1)); sin(yaw(1)) cos(yaw(1))];
    ellipse_points = R * [x_ellipse; y_ellipse];

    % Plot the ellipse representing the robot
    fill(ellipse_points(1,:) + posx, ellipse_points(2,:) + posy, 'k', 'FaceAlpha', 0.5);

    % Set axis limits and labels
    %axis equal;
    xlim([-20 20]); ylim([-20 20]); % Adjust based on your map
    grid on;
    %pause(5);
    hold off;

end