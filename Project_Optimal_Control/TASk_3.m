%% OPTIMAL CONTROL - Course Project #1
% Optimal Control of a Robotic Manipulator
% Group 27:
% Santoro Luca, 0001005415
% Armando Spennato, 0001006172
% Professor: Giuseppe Notarstefano   Tutor: Lorenzo Sforni

% Chapter 4
%% TASK 3 - Trajectory Tracking

addpath ./Functions;
addpath ./RobotManipulator;
visu_simulation=1; % Flag to enable the simulation.

%{
In the this task we want to linearize the robot dynamics about the 
optimal trajectory implemented in the previous task. To do this we use the
LQR algorithm in order to obtain an optimal feedback controller that allow 
us to track the reference trajectory.
%}

% xx_star and uu_star are two 'vectors' defined in Task_2, using DDP
% algorithm, that contain the optimal trajectories, that our system has to 
% follow in order to start from the initial position with down elbow (45,30)
% and then draw the ellipse.

% To perform the trajectory tracking, we solve a LQR optimal control problem.
% First of all, let's linearize the system about the optimal trajectory:

pp_useless = zeros(state_dim, TT);
x0 = [deg2rad(45); 0; deg2rad(30); 0]; % Expected initialization
%x0 = [deg2rad(0); 0; deg2rad(20); 0]; % Error in the initialization


% Linearization tensor. For every tt we have the linearization of the
% system in a certain instant.
AA_star = zeros(state_dim, state_dim, TT);
BB_star = zeros(state_dim, input_dim, TT);

for tt=1:TT
    [xxp_dummy, fx_star, fu_star, ~] = Dynamics(xx_star(:,tt,star), uu_star(:,tt,star), pp_useless(:,tt));
    AA_star(:,:,tt) = fx_star';
    BB_star(:,:,tt) = fu_star';
end

% Now we have to define the weight matrices with which we can perform the
% LQR control, considering that it's very important for us to precisely
% track the first and third component of the state

% In this case we choose the same state costs of task 2 and we increase only the input
% costs in order to track precisely the trajectory even if there are changes in the inizialization
% and parameters.
wq_star1 = 1000;
wq_star2 = 10;
wq_star3 = 1000;
wq_star4 = 10;
QQ_star = [wq_star1, 0, 0, 0; ...
           0, wq_star2, 0, 0; ...
           0, 0, wq_star3, 0; ...
           0, 0, 0, wq_star4];
wr_star1 = 0.5;
wr_star2 = 0.5;
RR_star = [wr_star1, 0;...
           0, wr_star2];

QQf_star = QQ_star;
xx = zeros(state_dim,TT);
xx(:,1) = x0;
uu = zeros(input_dim,TT);
PP = zeros(state_dim,state_dim,TT);
KK_star = zeros(input_dim,state_dim,TT);
PP(:,:,end) = QQf_star; 

% Backward iteration to compute P
for tt = TT-1:-1:1
    AAt = AA_star(:,:,tt);
    BBt = BB_star(:,:,tt);
    PPtp = PP(:,:,tt+1);        
    PP(:,:,tt) = QQ_star + AAt'*PPtp*AAt - (AAt'*PPtp*BBt)*inv(RR_star + BBt'*PPtp*BBt)*(BBt'*PPtp*AAt);
end

% Forward iteration to compute K,u,x_t+1
for tt = 1:TT-1
    AAt = AA_star(:,:,tt);
    BBt = BB_star(:,:,tt);
    PPtp = PP(:,:,tt+1);
    KK_star(:,:,tt) = -(RR_star + BBt'*PPtp*BBt)\(BBt'*PPtp*AAt);

    uu(:,tt) = uu_star(:,tt, star) + KK_star(:,:,tt)*(xx(:,tt)-xx_star(:,tt, star));
    [xx(:,tt+1), ~] = Dynamics(xx(:,tt), uu(:,tt), pp_useless(:,tt));
end 

uu(:,TT) = uu(:,TT-1);


% Now, to perform the simulation we have to pass data to Simscape Multibody
% scheme, in this way we are able to move the model of our manipulator.
u1 = zeros(TT,2);
u2 = zeros(TT,2);
x1 = zeros(TT,2);
x2 = zeros(TT,2);
x3 = zeros(TT,2);
x4 = zeros(TT,2);
for jj=0:TT-1
    x1(jj+1,1) = dt*jj;
    x1(jj+1,2) = xx(1,jj+1);
    x2(jj+1,1) = dt*jj;
    x2(jj+1,2) = xx(2,jj+1);
    x3(jj+1,1) = dt*jj;
    x3(jj+1,2) = xx(3,jj+1);
    x4(jj+1,1) = dt*jj;
    x4(jj+1,2) = xx(4,jj+1);
    u1(jj+1,1) = dt*jj;
    u1(jj+1,2) = uu(1,jj+1);
    u2(jj+1,1) = dt*jj;
    u2(jj+1,2) = uu(2,jj+1);
end
%% Plots
% PLOT OF RESULTING STATE AND INPUT TRAJECTORIES

figure(13); % position joint1
stairs(1:TT, xx(1,:),'LineWidth',2);
hold on;
stairs(1:TT, xx_star(1,:,star),'--','LineWidth',2);
ylabel('x1_t (rad)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution $$\theta_{1}$$ [rad]' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best'); 

figure(14); % velocity joint1
stairs(1:TT, xx(2,:),'LineWidth',2);
hold on;
stairs(1:TT, xx_star(2,:,star),'--','LineWidth',2);
ylabel('x2_t (rad/s)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution $$\dot{\theta}_{1}$$ [rad]' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best'); 

figure(15); % position joint2
stairs(1:TT, xx(3,:),'LineWidth',2);
hold on;
stairs(1:TT, xx_star(3,:,star),'--','LineWidth',2);
ylabel('x3_t (rad)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution $$\theta_{2}$$ [rad]' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best'); 

figure(16); % velocity joint2
stairs(1:TT, xx(4,:),'LineWidth',2);
hold on;
stairs(1:TT, xx_star(4,:,star),'--','LineWidth',2);
ylabel('x4_t (rad/s)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution $$\dot{\theta}_{2}$$ [rad]' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best'); 

figure(17); % input first joint
stairs(1:TT, uu(1,:),'LineWidth',2);
hold on;
stairs(1:TT, uu_star(1,:,star),'--','LineWidth',2);
ylabel('u1_t (Nm)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution of $$u_{1}$$ ' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best');

figure(18); %input second joint
stairs(1:TT, uu(2,:),'LineWidth',2);
hold on;
stairs(1:TT, uu_star(2,:,star),'--','LineWidth',2);
ylabel('u2_t (Nm)');
xlabel('t');
grid on;
zoom on;
title( 'Evolution of $$u_{2}$$ ' ,'Interpreter','latex', 'FontSize',20);
legend({'Evolution','Optimal'});
legend('Location','best');

%% Task 4 - Visualisation of the end-effector trajectory and simulation
% Chapter 5 
if visu_simulation==1
disp('Task_4');
% In this section we visualize the End-effector trajectory.
% For the simulation we use Simscape Multibody.
% Simscape Multibody provides a multibody simulation environment for 
% 3D mechanical systems such as robots

robot_manipulator_DataFile
simOut = sim('robot_manipulator'); % We simulate the model 
disp('Start simulation');

figure(200);
ss=length(simOut.x_motion);
zz = linspace(0,0,ss); 
comet3(simOut.x_motion,zz, simOut.y_motion);
plot3(simOut.x_motion,zz, simOut.y_motion,'LineWidth',3);
grid on;
axis equal;
zoom on;
title('Desired curve');
legend({'End effector position'});
legend('Location','best'); 
xlabel('X Coordinate [m]');
ylabel('Z Coordinate [m]'); % Fixed not change (2 DOF manipulator)
zlabel('Y Coordinate [m]');
end