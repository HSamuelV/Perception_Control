llfunction [v,w,pose] = casadi_script(ref,msg,u_0)

addpath('/home/hugo/Downloads/CaSadi/casadi-3.6.5-linux64-matlab2018b')
import casadi.*

%funcao bloco simulink
pos = msg ;
quat = pos.Pose(37, 1).Orientation;
% Tranformação quartenion para Euller
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
theta = angles(1);
pose = [pos.Pose(37, 1).Position.X; pos.Pose(37, 1).Position.Y; theta];

%%  controle

T = 0.1; %[s]
N = 50; % prediction horizon

v_max = 1; v_min = -v_max;
omega_max = 2*v_max/0.555; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial state and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

%% Problem Setup

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 100;Q(2,2) = 100;Q(3,3) = 10; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.1; R(2,2) = 0.1; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints

for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    %Values in P are the 2nd half of P
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st + (T*f_value);
    g = [g;st_next - st_next_euler]; % compute constraints
end

% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0; % Equality constraints
args.ubg(1:3*(N+1)) = 0; % Equality constraints

args.lbx(1:3:3*(N+1),1) = -20; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 20; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -20; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 20; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

%% Soluçao

% solução OCP
x0=pose;
u0 = u_0; zeros(N,2); %control input
X0 = repmat(x0,1,N+1); %states variables
args.p   = [x0;ref]; %parameters vector

% initial value of the optimization variables
args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution

controle=u(1,:);
v=controle(1);
w=controle(2);

