clear all;
close all;
clc;

% File for point stabilization and obstacles avoidance.

x0 = -1; y0 = -1; th0 = 0; % Initial posture of the robot
Ref = [1;1;0];  % referent posture for point stabilization

% Defining the size and the initial positions of the obstacles (circular-shaped) 
r_obs1 = 0.2;  % radius of the obstacle 1
x_obs1 = -0.5; y_obs1 = -0.5; %initial postion of obstacle 1
obs1 = [x_obs1;y_obs1;r_obs1]; % defined to be used by the simulink

r_obs2 = 0.2; % radius of the obstacle 2
x_obs2 = 0.5; y_obs2 = 0.5; %initial postion of obstacle 2
obs2 = [x_obs2;y_obs2;r_obs2]; % defined to be used by the simulink

sim_time = 60; % simulation time
Ts = 0.1; % sampling time

sim('Sim_PS_StaticOBS')  % simulink file to call for static obstacles avoidance
%% Plots 
% processing the outputs from the simulink for plotting
t = X.time;
x = X.signals.values(:,1); y = X.signals.values(:,2); th = X.signals.values(:,3);
Ex = err(:,1); Ey = err(:,2); Et = err(:,3);
v = V.signals.values(:,1); w = V.signals.values(:,2);
% Approximating the robot and the obstacles as circles using their respective radius
ang=0:0.005:2*pi;
r_rob = 0.1; % radius of the robot
xrob = r_rob*cos(ang); yrob = r_rob*sin(ang); % circular coordinate of the robot
xr1 = r_obs1*cos(ang); yr1 = r_obs1*sin(ang); % circular coordinate of Obstacle 1
xr2 = r_obs2*cos(ang); yr2 = r_obs2*sin(ang); % circular coordinate of Obstacle 2

% Adding a triagle inside the robot circle to show final direction
ht = 0.05; wt = 0.075; % triangle parameters
% for plotting the triangle at the start
x_tri = [x(1,1)+ht*cos(th(1,1)), x(1,1)+(wt/2)*cos((pi/2)-th(1,1)), x(1,1)-(wt/2)*cos((pi/2)-th(1,1))];
y_tri = [y(1,1)+ht*sin(th(1,1)), y(1,1)-(wt/2)*sin((pi/2)-th(1,1)), y(1,1)+(wt/2)*sin((pi/2)-th(1,1))];
% for plotting the triangle at the goal
x_tri_end = [x(end,1)+ht*cos(th(end,1)), x(end,1)+(wt/2)*cos((pi/2)-th(end,1)), x(end,1)-(wt/2)*cos((pi/2)-th(end,1))];
y_tri_end = [y(end,1)+ht*sin(th(end,1)), y(end,1)-(wt/2)*sin((pi/2)-th(end,1)), y(end,1)+(wt/2)*sin((pi/2)-th(end,1))];

figure
subplot(221) % plotting the robot trajectory
plot(x,y,'-g','LineWidth',3); hold on; 
fill(x(1,1)+xrob/2,y(1,1)+yrob/2,'g'); hold on; 
fill(x(end,1)+xrob/2,y(end,1)+yrob/2,'g');
text(x(1,1)-0.15,y(1,1)-0.15,'Start'); 
text(x(end,1)-0.2,y(end,1)+0.2,'Goal');hold on
fill(x_tri,y_tri,'g'); hold on; % ploting robot triangle at the start
fill(x_tri_end,y_tri_end,'g'); hold on; % ploting robot triangle at the end
fill(x_obs1+xr1,y_obs1+yr1,'b') % Obstacle 1
hold on
fill(x_obs2+xr2,y_obs2+yr2,'b') % Obstacle 1

xlabel('x-axis [m]', 'FontSize', 20)
ylabel('y-axis [m]', 'FontSize', 20)
grid on
title('Robot Trajectory', 'FontSize', 20)
axis([-1.5 1.5 -1.5 1.5])

subplot(222) % plotting the state errors
plot(t,Ex,'-','LineWidth',1); hold on
plot(t,Ey,'--','LineWidth',1); hold on
plot(t,Et,'-.','LineWidth',1);
grid on
xlabel('time [s]', 'FontSize', 20)
ylabel('state vector error', 'FontSize', 20)
title('State vector error', 'FontSize', 20)
legend('e_{x}[m]','e_{y}[m]','e_{\theta}[rad]')

subplot(223) % plotting the linear speed
stairs(t,v,'-k','LineWidth',2);hold on
plot(t,0.06*ones(length(t)),'-.r','LineWidth',0.01); hold on
plot(t,-0.06*ones(length(t)),'-.r','LineWidth',0.01)
grid on
xlabel('time [s]', 'FontSize', 20)
ylabel('v [m/s]', 'FontSize', 20)
title('Linear speed', 'FontSize', 20)
axis([0 sim_time -0.075 0.075])

subplot(224) % plotting the angular speed
stairs(t,w,'-k','LineWidth',2); hold on
plot(t,pi/2*ones(length(t)),'-.r','LineWidth',0.01); hold on
plot(t,-pi/2*ones(length(t)),'-.r','LineWidth',0.01)
grid on
xlabel('time [s]', 'FontSize', 20)
ylabel('w [rad/s]', 'FontSize', 20)
title('Angular speed', 'FontSize', 20)
axis([0 sim_time -1.9 1.9])
%% Video Simulation of Static Obstacle Avoidance

% L = size(X.signals.values,1);
% % initialize
% xp1 = []; yp1 = []; thp1 = [];
% figure
% 
% for k = 1:L
%   %************************Robot************************************
%   xp = X.signals.values(k,1); 
%   yp = X.signals.values(k,2);
%   thp = X.signals.values(k,3);
%   xp1 = [xp1 xp]; yp1 = [yp1 yp]; thp1 = [thp1 thp];
%   xp_tri = [xp+ht*cos(thp), xp+(wt/2)*cos((pi/2)-thp), xp-(wt/2)*cos((pi/2)-thp)];
%   yp_tri = [yp+ht*sin(thp), yp-(wt/2)*sin((pi/2)-thp), yp+(wt/2)*sin((pi/2)-thp)];
%   
%   fill(x0+xrob/4,y0+yrob/4,'g'); hold on; 
%   fill(Ref(1,1)+xrob/4,Ref(2,1)+yrob/4,'g'); hold on;
%   text(x(1,1)-0.15,y(1,1)-0.15,'Start'); hold on;
%   text(x(end,1)-0.2,y(end,1)+0.2,'Goal');hold on
%   %********************Plot video********************************************
%   plot(xp1,yp1,'-g')
%   hold on
%   fill(xp_tri,yp_tri,'g') % ploting robot triangle
%   hold on
%   plot(xp+xrob,yp+yrob,'--g')
%   hold on
%   
%   fill(x_obs1+xr1,y_obs1+yr1,'b') % Obstacle 1
%   hold on
%   fill(x_obs2+xr2,y_obs2+yr2,'b') % Obstacle 1
% 
%   axis([-1.5 1.5 -1.5 1.5])
%   hold off
%   pause(0.1); % synchronized with the sampling time. Ts = 0.1sec
%   box on;
%   grid on;
%   drawnow
%   F(k) = getframe(gcf); % to get the current frame
% end

