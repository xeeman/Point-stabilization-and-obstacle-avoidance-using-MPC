classdef single_robot_obs < matlab.System & matlab.system.mixin.Propagates
% untitled Add summary here

% This template includes the minimum set of functions required
% to define a System object with discrete state.

properties
% Public, tunable properties.

end

properties (DiscreteState)
end

properties (Access = private)
% Pre-computed constants.
casadi_solver 
x0
lbx
ubx
lbg
ubg
p
end

methods (Access = protected)
function num = getNumInputsImpl(~)
    num = 1;
end
function num = getNumOutputsImpl(~)
    num = 2;
end
function [dt1,dt2] = getOutputDataTypeImpl(~)
    dt1 = 'double'; dt2 = 'double';
end
function dt1 = getInputDataTypeImpl(~)
    dt1 = 'double';
end
function [sz1,sz2] = getOutputSizeImpl(~)
    sz1 = [2,1]; sz2 = [1,1];
end
function sz1 = getInputSizeImpl(~)
    sz1 = [12,1];
end
function cp1 = isInputComplexImpl(~)
    cp1 = false;
end
function [cp1,cp2] = isOutputComplexImpl(~)
    cp1 = false; cp2 = false;
end
function fz1 = isInputFixedSizeImpl(~)
    fz1 = true;
end
function [fz1,fz2] = isOutputFixedSizeImpl(~)
    fz1 = true; fz2 = true;
end
function setupImpl(obj,~,~)
% Implement tasks that need to be performed only once such as pre-computed constants.

addpath('D:\Users\sanim\Documents\MATLAB\MPC\casadi-windows-matlabR2016a-v3.4.5')
import casadi.*

T = 0.1; % sampling time (s)
N = 20; % prediction horizon

v_max = 0.06; v_min = -v_max; % linear speed constraints
omega_max = pi/2; omega_min = -omega_max; % angular speed constrains

xmax = 5; xmin = -xmax; ymax = xmax; ymin = -ymax;  % states constraints

x1 = SX.sym('x1'); x2 = SX.sym('x2'); x3 = SX.sym('x3'); % declaration of states variables
states = [x1;x2;x3]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega'); % declaration of control variables
controls = [v;omega]; n_controls = length(controls);

rhs = [v*cos(x3);v*sin(x3);omega]; % system r.h.s
U = SX.sym('U',n_controls,N); % Decision variables (controls)
X = SX.sym('X',n_states,(N+1)); % Decision variable (states)
% A vector that represents the states over the optimization problem.
P = SX.sym('P',n_states + n_states + 6); % 6 is for obstacles parameters = 2 obs * 3(x,y,r) of the obstacles
% parameters (which include the initial state and the reference state)

f = casadi.Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)

J = 0; % Objective function
g = [];  % constraints vector

Q = diag([1;1;0.001]); % weighing matrices (states)
R = diag([1;1]); % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  
    con = U(:,k);
    J = J + (st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    %J = (k/N)^ m * J;
    st_next = X(:,k+1);
   %f_value = f(st,con);    st_next_euler = st + (T*f_value); % Euler Discretization
    k1 = f(st, con); k2 = f(st + T/2*k1, con); k3 = f(st + T/2*k2, con); k4 = f(st + T*k3, con);
    st_next_RK4 = st + T/6*(k1+2*k2+2*k3+k4);
    g = [g;st_next-st_next_RK4]; % compute constraints
end

QN = 100000*Q;
J = J + (st-P(4:6))'*QN*(st-P(4:6)); % stabilizing cost function

r_rob = 0.1; % radius of the robot

% constraints for obstacles avoidance
for k=1:N+1 % First obstacle
   g = [g;-sqrt( (X(1,k)-P(7))^2 + (X(2,k)-P(8))^2 ) + (P(9) + r_rob)]; 
end
for k=1:N+1 % Second obstacle
   g = [g;-sqrt( (X(1,k)-P(10))^2 + (X(2,k)-P(11))^2 ) + (P(12) + r_rob)]; 
end

% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)]; % multiple shooting

% converting optimization to nonlinear programming problem
nlp_prob = struct('f', J, 'x', OPT_variables, 'g', g, 'p', P); 

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts); %employing ipopt solver to solve the problem
obj.casadi_solver = solver;

%obj = struct;

obj.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
obj.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

obj.lbg(3*(N+1)+1:3*(N+1)+(N+1)) = -inf;  % -1e-20  % Equality constraints First
obj.ubg(3*(N+1)+1:3*(N+1)+(N+1)) = 0;  % 1e-20   % Equality constraints

obj.lbg(3*(N+1)+(N+1)+1:3*(N+1)+(N+1)+(N+1)) = -inf;  % -1e-20  % Equality constraints  second
obj.ubg(3*(N+1)+(N+1)+1:3*(N+1)+(N+1)+(N+1)) = 0;  % 1e-20   % Equality constraints

obj.lbx(1:3:3*(N+1),1) = xmin; %state x lower bound
obj.ubx(1:3:3*(N+1),1) = xmax; %state x upper bound
obj.lbx(2:3:3*(N+1),1) = ymin; %state y lower bound
obj.ubx(2:3:3*(N+1),1) = ymax; %state y upper bound
obj.lbx(3:3:3*(N+1),1) = -pi; %state theta lower bound
obj.ubx(3:3:3*(N+1),1) = pi; %state theta upper bound

obj.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
obj.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
obj.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
obj.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound

end

function [u, comp_time] = stepImpl(obj,x)
% this function takes new variables to give the new outputs

N = 20; 

tic % to start the clock for measuring the computation time

xm = x(1:3);    % initial condition.
xr = x(4:6);    % referent posture
obs = x(7:12);  % obstacles paramters

u0 = zeros(N,2);        % initialization of the control decision variables
X0 = repmat(xm,1,N+1)'; % initialization of the states decision variables

% initial value of the optimization variables
obj.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
obj.p   = [xm;xr;obs]; % set the values of the parameters vector
solver = obj.casadi_solver;
sol = solver('x0', obj.x0, 'lbx', obj.lbx, 'ubx', obj.ubx,...
        'lbg', obj.lbg, 'ubg', obj.ubg,'p', obj.p); % solution

u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)';  % cutting only the control decision variables
u = u(1,:)';  
comp_time = toc % to stop the clock
end
function resetImpl(obj)
    % Initialize discrete-state properties.
end
end
end