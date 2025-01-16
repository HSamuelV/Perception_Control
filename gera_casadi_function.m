clear all
close all
clc

addpath('/home/hugo/Downloads/CaSadi/casadi-3.6.5-linux64-matlab2018b')
import casadi.*


T = 0.1; %[s]
N = 50; % prediction horizon

rob_length = 0.99;
rob_width = 0.67;
rob_diam = 1;
v_max = 1.1; v_min = -v_max;
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

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 20;Q(2,2) = 50;Q(3,3) = 5; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.1; R(2,2) = 0.5; % weighing matrices (controls)

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

%obstacles constraints
obs_x = 2.0; % position meters
obs_y = 2.5; % position meters
obs_diam = 1.0; % diameter meters
robot_height = 0.39; %meters
rob_diam_cler = 1; %diameter of robot rotating in axes z - meters

for k=1:N+1
    g = [g ; -sqrt((X(1,k)-obs_x)^2+(X(2,k)-obs_y)^2) + (rob_diam_cler/2 + obs_diam/2)];
%inequality constraint for obstacle
end

% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f',obj,'x',OPT_variables,'g',g,'p',P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver','ipopt',nlp_prob,opts);

args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints
args.lbg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = -inf; % inequality constraints
args.ubg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = 0; % inequality constraints

args.lbx(1:3:3*(N+1),1) = -10; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 10; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -10; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 10; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP

% solução OCP
%Time simulation
t0 = 0;
x0 = [0.0; 0.0; 0.0]; %posiçao inicial
xs = [5.0; 7.0; 3*pi/4]; %posicao de referencia
xx(:,1) = x0; %history of states
t(1) = t0; %initial time
u0 = zeros(N,2); %control input
X0 = repmat(x0,1,N+1); %states variables
sim_time = 20; %simulation time

%MPC
mpciter = 0; 
xx1 = []; %predicted trajectory
u_cl = []; %predicted control

%'norm > 1e-2' is how close the result is to the ref
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_time/T)
    args.p   = [x0;xs]; %parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1) = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; %predicted trajectory
    u_cl = [u_cl; u(1,:)];
    t(mpciter+1) = t0;
    %apply control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u, f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)';
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter;
    mpciter = mpciter +1
end

ss_error = norm((x0-xs),2)

%Draw_MPC_point_stabilization_mod (t,xx,xx1,u_cl,xs,N,rob_length,rob_width)
Draw_MPC_point_stabilization_obs (t,xx,xx1,u_cl,xs,N,rob_length,rob_width,obs_x,obs_y,obs_diam)
%Draw_MPC_point_stabilization_v1 (t,xx,xx1,u_cl,xs,N,rob_diam)

