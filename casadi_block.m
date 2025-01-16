classdef casadi_block < matlab.System
    properties (Access = private)
        casadi_solver
        lbx
        ubx
        lbg
        ubg
        u0
    end


    methods (Access = protected)
        function setupImpl(obj, ~, ~)
            %addpath('/home/hugo/Downloads/CaSadi/casadi-3.6.5-linux64-matlab2018b')
            import casadi.*
            % Define the problem
            T = 0.1; % Sampling time [s]
            N = 50; % Prediction horizon
            v_max = 1; 
            v_min = -v_max;
            omega_max = 2 * v_max / 0.555; 
            omega_min = -omega_max;

            % CasADi setup
            x = casadi.SX.sym('x'); 
            y = casadi.SX.sym('y'); 
            theta = casadi.SX.sym('theta');
            states = [x; y; theta]; 
            n_states = length(states);
            v = casadi.SX.sym('v'); 
            omega = casadi.SX.sym('omega');
            controls = [v;omega]; 
            n_controls = length(controls);
            rhs = [v*cos(theta);v*sin(theta);omega];
            
            f = casadi.Function('f',{states,controls},{rhs}); %nonlinear mapping function f(x,u)
            U = casadi.SX.sym('U',n_controls,N); %decision variable
            P = casadi.SX.sym('P',n_states+n_states); % parameters (which include the initial state and the reference state)
            X = casadi.SX.sym('X',n_states,(N+1)); % A vector that represents the states over the optimization problem.
            

            % Objective function and constraints
            obj_func = 0; %objective function
            g = []; %constraints vector
           
            Q = zeros(3,3);Q(1,1) = 20;Q(2,2) = 50;Q(3,3) = 5; %weight matrix for position
            R = zeros(2,2);R(1,1) = 0.1;R(2,2) = 0.05; %weight matrix for control
            
            st = X(:,1); %initial state
            g = [g;st-P(1:3)]; %initial condition constraints

            for k = 1:N
                st = X(:,k); 
                con = U(:,k);
                obj_func = obj_func+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con;
                %value in P are the second half of P
                st_next = X(:,k+1);
                f_value = f(st,con);
                st_next_euler = st+(T*f_value);
                g = [g; st_next-st_next_euler]; %compute constraints
            end

            %obstacles constraints
            obs_x = 1.0; % position meters
            obs_y = 1.0; % position meters
            obs_diam = 0.5; % diameter meters
            %robot_height = 0.39; %meters
            rob_diam_cler = 1; %diameter of robot rotating in axes z - meters

            for k=1:N+1
                g = [g ; -sqrt((X(1,k)-obs_x)^2+(X(2,k)-obs_y)^2) + (rob_diam_cler/2 + obs_diam/2)];
                %inequality constraint for obstacle
            end

            OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];
            %turn the decision variable one column vector
            nlp_prob = struct('f', obj_func, 'x', OPT_variables, 'g', g, 'p', P);

            opts = struct;
            opts.ipopt.max_iter = 2000;
            opts.ipopt.print_level = 0;
            opts.print_time = 0;
            opts.ipopt.acceptable_tol = 1e-8;
            opts.ipopt.acceptable_obj_change_tol = 1e-6;

            obj.casadi_solver = casadi.nlpsol('solver', 'ipopt', nlp_prob, opts);

            % Constraints bounds
            obj.lbg = zeros(3*(N+1),1); %Equality constraints
            obj.ubg = zeros(3*(N+1),1); %Equality constraints
            obj.lbg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = -inf; % inequality constraints
            obj.ubg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = 0; % inequality constraints

            %obj.lbx = -inf*ones(3*(N+1)+2*N,1);
            %obj.ubx = inf*ones(3*(N+1)+2*N,1);
            obj.lbx(1:3:3*(N+1),1) = -10; %state x lower bound
            obj.ubx(1:3:3*(N+1),1) = 10;  % state x upper bound
            obj.lbx(2:3:3*(N+1),1) = -10; %state y lower bound
            obj.ubx(2:3:3*(N+1),1) = 10;  %state y upper bound
            obj.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
            obj.ubx(3:3:3*(N+1),1) = inf;  %state theta upper bound

            obj.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
            obj.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
            obj.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
            obj.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

            obj.u0 = zeros(N,2); % Initial control input guess
        end

        function [lin_vel,ang_vel] = stepImpl(obj,ref,msg)%,A,B,size
            % variaveis locais
            robot_length = 0.99;
            robot_width = 0.67;
            N_ = 50;
            w = [];
            
            % Extract pose information
            pos = msg; %receive the position
            quat = pos.Pose(37,1).Orientation; %receive the orientation
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]); %get the angles for each 
            theta = angles(1); %get angles of yaw
            pose = [pos.Pose(37,1).Position.X; pos.Pose(37,1).Position.Y;theta]; %put the position and orientation in a var
            pose_X = [pos.Pose(37,1).Position.X]; %used to send the position X
            pose_Y = [pos.Pose(37,1).Position.Y]; %used to send the position y
            pose_T = theta; %used to send the angle theta

            % Matriz modelo posiçao robo
            Posxy = [pose_X,pose_Y]';
            R_pos = [cos(angles(1)) -sin(angles(1));...
                         sin(angles(1)) cos(angles(1))];
            D_rob_size = [power(robot_width,2) 0; 0 power(robot_length,2)];
            Q_pos_rob = R_pos*D_rob_size*(R_pos');

            % Initial state
            x0 = pose; %get the current position
            X0 = repmat(x0,1,N_+1); %states variable
            args.p = [x0;ref]; %assign the initial position and the reference
            args.x0 = [reshape(X0',3*(N_+1),1);reshape(obj.u0',2*N_,1)];
            
            % Solve the optimization problem
            sol = obj.casadi_solver('x0', args.x0, 'lbx', obj.lbx, 'ubx', obj.ubx, ...
                                    'lbg', obj.lbg, 'ubg', obj.ubg, 'p', args.p);
            u = reshape(full(sol.x(3*(N_+1)+1:end))',2,N_)'; %get the control only of the solution
            controle = u(1,:); %get the current controls
            lin_vel = controle(1); %get the current linear velocity control
            ang_vel = controle(2); %get the current angluar velocity control

            
            for k = 1:length(A)
                w(k) = (B(k)-A(k,:)*Posxy + (A(k,:)*Q_pos_rob*A(k,:)')^1/2 )/((A(k,:)*(A(k,:)'))^1/2);
            end
        end

        function [out1, out2, out3] = getOutputSizeImpl(~)
            %this determine the size of the output
            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            %determine the type of the output
            out1 = 'double';
            out2 = 'double';
            out3 = 'double';
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            %determine if the output has a fixed size
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
            %determine if the output produce complex numbers
            out1 = false;
            out2 = false;
            out3 = false;
        end
    end
end
