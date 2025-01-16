clear
clc

%% Variaveis globais
robot_length = 0.99;  % Adjust as per your robot's dimensions
robot_width = 0.67;   % Adjust as per your robot's dimensions
epsilon = 0.67;
min_pts = 3;
circle_radius = 0.05;
figure();

while(1)
    % Variaveis locais
    all_polytopes = [];

    % Create a subscriber for the LIDAR data
    pcSub = rossubscriber('/lidar_ust', 'sensor_msgs/LaserScan');
    state = rossubscriber('/gazebo/model_states','gazebo_msgs/ModelStates');

    % Receive the data
    lidar_msg = receive(pcSub);
    rob_msg = receive(state);f

    %Extract the data from the lidar
    ranges = lidar_msg.Ranges; % Extract range data
    angles = linspace(lidar_msg.AngleMin, lidar_msg.AngleMax, length(ranges))'; % Calculate angles

    % Define the robot's position (x, y) and orientation
    robotPos = rob_msg.Pose(37,1).Position; % Example position in 2D
    posx = robotPos.X;
    posy = robotPos.Y;
    
    %Orientation of the robot
    orientation = rob_msg.Pose(37,1).Orientation;
    q = [orientation.W, orientation.X, orientation.Y, orientation.Z];
    yaw = quat2eul(q);  % Convert quaternion to Euler angles and extract yaw
    R_polytope = [cos(yaw(1)) -sin(yaw(1));...
                sin(yaw(1)) cos(yaw(1))];

    % Convert ranges to Cartesian coordinates
    x = ranges .* cos(angles + yaw(1)) + posx;
    y = ranges .* sin(angles + yaw(1)) + posy;
    
    x(isinf(x)) = [];
    y(isinf(y)) = [];
    %Pontos = double(validPoints);

    %%
    Pxy = [x,y];
    idx = dbscan(Pxy,epsilon,min_pts);

    % Unique cluster labels
    uniqueLabels = unique(idx); %creates a unique label for each cluster

    % Plot all points
    %gscatter(Px, Py, idx);
    hold on;
    
    %Plot the Ellipse
    pos_polytope = [posx posy]'; %pegar do gazebo/sensores
    D_polytope = [1/robot_length^2 0;0 1/robot_width^2];
    Q_polytope = round(R_polytope'*D_polytope*R_polytope,2);
    %ellob2 = ellipsoid(pos_polytope, Q_polytope^-1);
    ellob2 = ellipsoid(pos_polytope, Q_polytope^-1);
    plot(ellob2,'k');  
    
    
    %% Generate the convex hull for each cluster
    for i = 1:length(uniqueLabels)
        clusterLabel = uniqueLabels(i);
        if clusterLabel == -1
            % Skip noise points
            continue;
        end
        % Extract points belonging to the current cluster
        clusterPoints = double(Pxy(idx == clusterLabel, :));

        % Compute the convex hull
        K = convhull(clusterPoints(:,1), clusterPoints(:,2));

        % Plot the convex hull
        %plot(clusterPoints(K,1), clusterPoints(K,2), 'LineWidth', 2);

        %Get the hull points to generate the matrixes
        hull_points = clusterPoints(K,:);

        % Calculate the half-space representation
        % Each edge of the convex hull defines a half-space
        num_edges = length(K) - 1;
        A_matrices = zeros(num_edges, 2);
        b_vectors = zeros(num_edges, 1);
        
        
        %% Get the Matrices A and B
        for j = 1:num_edges
            % Get the start and end points of the edge
            p1 = hull_points(j, :);
            p2 = hull_points(mod(j, num_edges) + 1, :);

            % Calculate the line equation coefficients
            a = p2(2) - p1(2); % y2 - y1
            b = p1(1) - p2(1); % x1 - x2
            c = a * p1(1) + b* p1(2); % - (a * x1 + b * y1)

            A_matrices(j, :) = [a, b]; % Store as negative to conform to Ax <= b
            b_vectors(j) = c;           % Store c
        end
        
        %take the lines with inf or NaN
        A_matrices(any(isinf(A_matrices) | isnan(A_matrices),2), :) = [];
        b_vectors(any(isinf(b_vectors) | isnan(b_vectors),2), :) = [];
        
        A_matrices
        b_vectors

        % %check if the Matrices are empty
        if ~isempty(A_matrices) && ~isempty(b_vectors) && size(A_matrices, 1) == size(b_vectors, 1)
            % Create a Polyhedron from A and b for the current cluster
            P = Polyhedron('A', A_matrices, 'b', b_vectors);
            all_polytopes = [all_polytopes, P]; % Store the polyhedron

        else
            warning('Empty or incompatible matrices for cluster %d', clusterLabel);
        end
    end

    % Plot all polytopes
    for poly = all_polytopes
        plot(poly, 'Color', rand(1, 3)); % Random color for distinction
    end

    % Add labels and title for the LIDAR plot
    xlabel('X [m]');
    ylabel('Y [m]');
    title('DBSCAN Clustering with Convex Hulls and Robot Position');
    % Set axis limi ts and labels
    xlim([-10 10]); ylim([-10 10]); % Adjust based on your map
    grid on;
    hold off;
    pause(2);
    clf;

    w = [];
    for k = 1:length(A_matrices)
        w(k) = (b_vectors(k)-A_matrices(k,:)*pos_polytope + (A_matrices(k,:)*Q_polytope*A_matrices(k,:)')^1/2 )/((A_matrices(k,:)*(A_matrices(k,:)'))^1/2);
    end
fprintf('fim');


end