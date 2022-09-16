%% OPTIMAL CONTROL - Course Project #1
% Optimal Control of a Robotic Manipulator
% Group 27:
% Santoro Luca, 0001005415
% Armando Spennato, 0001006172
% Professor: Giuseppe Notarstefano   Tutor: Lorenzo Sforni

% Chapter 2
%% TASK 1 - Trajectory Exploration: Automated Warehouse

clear all
close all
clc


addpath ./Functions;

%{
In this section we want to design an optimal trajectory that our robot will follow to move from 
one equilibrium configuration to another,  using the DDP algorithm; The achievement of this task 
is a relevant aspect in pick and place operations in order to move industrial manipulators within 
automated wharehouses.
%}

% Before we begin to perform our task, we need to define the dynamics of our 2-DOF manipulator. 
% To do this we execute the script GenerateDynamics.m which allows to generate the Dynamics.m 
% function (See comments within script for more).
%% Definition of Parameters:

max_iters = 2e2;           % Maximum number of iterations.
tol = 1e-6;                % Tolerance of our algorithm.

tf = 30;                   % Length of time window (we inizialized the final time) [seconds]   

%{

It is useful to have longer time horizon in order to have more stable phases. 
The idea is to isolate the perturbations in the middle of the time horizon.

%}

% Parameters:
params.dyn.dt = 1e-3;       % Sampling time that we use to sample a continuous dynamics 
                            % we are using for this optimal control problem.

params.dyn.mm1 = 2;         % [kg]
params.dyn.mm2 = 2;         % [kg]
params.dyn.gg = 9.81;       % [m/s^2]
params.dyn.ll1 = 1;         % [m]
params.dyn.ll2 = 1;         % [m]
params.dyn.rr1 = 0.5;       % [m]
params.dyn.rr2 = 0.5;       % [m]
params.dyn.J_iner1 = 0.5;   % [kg*m^2]
params.dyn.J_iner2 = 0.5;   % [kg*m^2]

dt = params.dyn.dt;
mm1 = params.dyn.mm1;
mm2 = params.dyn.mm2;
gg = params.dyn.gg;
ll1 = params.dyn.ll1;
ll2 = params.dyn.ll2;
rr1 = params.dyn.rr1;
rr2 = params.dyn.rr2;
J_iner1 = params.dyn.J_iner1;
J_iner2 = params.dyn.J_iner2;


TT = tf/params.dyn.dt; % Number of samples

nn = 4; % Length of the State vector
nu = 2; % Length of the Input vector
 
%Step-size selection rule:
% Flags for Armijo
ARMIJO_flag = 1; % We use Armijo step size rule (minimization rule)
visu_armijo = 1;

gamma_fix = 0.8;  % Fixed step size, it remain constant at each iteration 

%{
in general, increasing the step size allows you to move in the direction in which 
the function is decreasing and therefore to converge much faster to a global minimum. 
The step size CANNOT be very large because otherwise we have problems, in prticular we diverge.
%}

fprintf("Parameters defined\n")
% We define the weight matrices that we will use to define our standard
% Cost Function,a quadratic function which contains matrices Q, R and QF. 

% We define the weights of the matrix Q
wq1 = 1500;
wq2 = 100; 
wq3 = 1500; 
wq4 = 100; 
params.cost.QQ = [wq1, 0, 0, 0; ...
                  0, wq2, 0, 0; ...
                  0, 0, wq3, 0; ...
                  0, 0, 0, wq4];
params.cost.QQf = [wq1, 0, 0, 0; ...
                  0, wq2, 0, 0; ...
                  0, 0, wq3, 0; ...
                  0, 0, 0, wq4]; 

% We define the weights of the matrix R
wr1 = 0.005;
wr2 = 0.005;
params.cost.RR = [wr1, 0;...
                  0, wr2];
% It is important to select weights to achieve some desired behaviors (or
% performances).

fprintf("Weight matrices defined\n")
% References (Desired behaviour)
% We define the reference angles and velocities (xx_des) and the
% corresponding reference inputs (uu_des)

% Robot initial configuration:
ref_q1_i_deg = 45; % initial [deg]
ref_q2_i_deg = 30; % initial
% Robot final configuration:
ref_q1_f_deg = 150; % final [deg]
ref_q2_f_deg = -30; % final

xx_des = zeros(nn, TT);
uu_des = zeros(nu, TT);

% We converts angle units from degrees to radians and we plug their values
% in the desired state and input vector:
xx_des(1,1:TT/2) = deg2rad(ref_q1_i_deg); 
xx_des(1,TT/2:end) = deg2rad(ref_q1_f_deg);
xx_des(3,1:TT/2) = deg2rad(ref_q2_i_deg);
xx_des(3,TT/2:end) = deg2rad(ref_q2_f_deg);

fprintf("Reference defined\n")
% Plots of our reference state
figure(1);

sgt = sgtitle('Angle (state) reference');
sgt.FontSize = 20;
subplot(1,2,1) % θ1 desidered
plot(rad2deg(xx_des(1,:)),'LineWidth',3);
axis tight
grid on
title('\theta_1 desired');
ylabel('\theta [deg]');
xlabel('t');
subplot(1,2,2) % θ2 desidered
plot(rad2deg(xx_des(3,:)),'LineWidth',3); 
grid on
title('\theta_2 desired');
ylabel('\theta [deg]');
xlabel('t');
% To allow the robot to keep the initial equilibrium configuration and then the final one, 
% where q'= 0 and q''= 0, our desired input u_des must balance the term g(q).

uu_des(1,1:TT/2) = (mm1*rr1+mm2*ll1)*gg*cos(xx_des(1,1))+mm2*gg*rr2*cos(xx_des(1,1)+xx_des(3,1));
uu_des(1,TT/2:end) = (mm1*rr1+mm2*ll1)*gg*cos(xx_des(1,TT))+mm2*gg*rr2*cos(xx_des(1,TT)+xx_des(3,TT));
uu_des(2, 1:TT/2) = mm2*gg*rr2*cos(xx_des(1,1)+xx_des(3,1));
uu_des(2, TT/2:end) = mm2*gg*rr2*cos(xx_des(1,TT)+xx_des(3,TT));

% Plots our inputs (torque) reference
figure(2);

sgt = sgtitle('Torque (input) reference');
sgt.FontSize = 20;
subplot(1,2,1)
plot(uu_des(1,:),'LineWidth',3); % u1 desidered
grid on
ylabel('u [Nm]');
xlabel('t');
title('u_1 desired');
subplot(1,2,2)
plot(uu_des(2,:),'LineWidth',3); % u2 desidered
grid on
ylabel('u [Nm]');
xlabel('t');
title('u_2 desired');

fprintf("Reference defined\n")

% TRAJECTORY DEFINITION

% After having defined the step between the two equilibrium configurations,
% we want to define the optimal trajectory to pass from the initial
% configuration (xx_des(1),uu_des(1)) to the final one (xx_des(2),uu_des(2))

xx = zeros(nn, TT, max_iters);
uu = zeros(nu, TT, max_iters);

% Since DDP algorithm is an iterative procedure, it is necessary to define
% a value to the initial iteration.

% INITIALIZATION of x and u: we use the inverse dynamic control in order 
% to not start from a trajectory too far from the final one.

[xx(:,:,1),uu(:,:,1)] = controller(xx_des,uu_des, params, 1); 

JJ = zeros(max_iters,1);
descent = zeros(max_iters,1);

fprintf('End of initialization\n');

% WE start with our optimization procedure:

kk = 1; % Just to get the cost at the first iteration 
 
% In the following loop we build the FIRST iteration of the Cost based on
% state and input from the initial instant to the second last one
for tt=1:TT-1
  [cost_dummy, ~] = Stage_Cost(xx(:,tt,kk), uu(:,tt,kk), xx_des(:,tt), uu_des(:,tt), params);
  JJ(kk) = JJ(kk) + cost_dummy;
end

% Here we define the last value of the cost, exploiting the state at the
% last instant (TT)
[cost_dummy, ~] = Term_Cost(xx(:,TT,kk), xx_des(:,TT), params);
JJ(kk) = JJ(kk) + cost_dummy;


% MAIN LOOP (runs until max_iters or until tolerance is reached)

for kk=1:max_iters-1 
 
  KK = zeros(nu,nn, TT);
  sigma = zeros(nu, TT);
  
  pp = zeros(nn, TT);
  PP = zeros(nn,nn, TT);
  
  % Initialization of the terms p and P that have to be used to perform the
  % control:
  % p_T = q_T = gradient of the terminal cost
  % P_T = Q_T = hessian of the terminal cost
  % The gradient and the hessian are given as second and third outputs by 
  % the function term_cost
  [~, pp(:,TT), PP(:,:,TT)] = Term_Cost(xx(:,TT,kk), xx_des(:,TT), params);
  
%   Backward iteration 
  for tt = TT-1:-1:1
    
    [~, fx, fu, pfxx, pfuu, pfux] = Dynamics(xx(:,tt,kk), uu(:,tt,kk),pp(:,tt+1));
    [~, lx, lu, lxx, luu, lux] = Stage_Cost(xx(:,tt,kk), uu(:,tt,kk),xx_des(:,tt), uu_des(:,tt), params);

    % Compute gain and ff descent direction
    
    % Gain 
    KK(:,:,tt) = -(luu + fu*PP(:,:,tt+1)*fu'+ pfuu)\(lux + fu*PP(:,:,tt+1)*fx' + pfux);
    
   % ff term                            
    sigma(:,tt) = -(luu + fu*PP(:,:,tt+1)*fu'+ pfuu)\(lu + fu*pp(:,tt+1));
                             
                              
    % Update PP and pp
    PP(:,:,tt) = (lxx + fx*PP(:, :, tt+1)*fx' + pfxx) - KK(:, :,tt)'*(luu + fu*PP(:,:,tt+1)*fu' + pfuu)*KK(:, :, tt);
    pp(:,tt) = (lx + fx*pp(:,tt+1))- KK(:, :,tt)'*(luu + fu*PP(:,:,tt+1)*fu' + pfuu)*sigma(:, tt);
    
     descent(kk) = descent(kk) + (lu + fu*pp(:,tt+1))'*sigma(:,tt); % Descent quantity is the norm of ff term 
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %   ARMIJO gamma_stepsize selection  %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %gamma_stepsize selection
  if ARMIJO_flag
    
    cc = 0.1;
    rho = 0.5;
    gammas = 1;
    
    cost_arm = [];
    
    xx_temp = zeros(nn,TT);
    uu_temp = zeros(nu,TT);

    xx_temp(:,1) = xx_des(:,1);
    JJtemp = 0;
    
    for tt = 1:TT-1
      
      
      uu_temp(:,tt) = uu(:,tt,kk) + gammas(end)*sigma(:,tt) + ...
        KK(:,:,tt)*(xx_temp(:,tt) - xx(:,tt,kk));
      
      [xx_temp(:,tt+1),~] = Dynamics(xx_temp(:,tt),uu_temp(:,tt),...
        pp(:,tt+1));

      [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), ...
        xx_des(:,tt), uu_des(:,tt), params);
      JJtemp = JJtemp + cost_dummy;
    end
    
    [cost_dummy, ~] = Term_Cost(xx_temp(:,TT), xx_des(:,TT), params);
    
    JJtemp = JJtemp + cost_dummy;
    
    cost_arm = [cost_arm; JJtemp];
    
  % ARMIJO LOOP
    while cost_arm(end) > JJ(kk) + cc*gammas(end)*descent(kk)
      
      gammas = [gammas; gammas(end)*rho];
      
      % Evaluate cost for gamma_i
      xx_temp(:,1) = xx_des(:,1);
      
      JJtemp = 0;
      
      for tt = 1:TT-1
        % Compute input
        uu_temp(:,tt) = uu(:,tt,kk) + gammas(end)*sigma(:,tt) + KK(:,:,tt)*(xx_temp(:,tt) - xx(:,tt,kk));
        %
        [xx_temp(:,tt+1),~] = Dynamics(xx_temp(:,tt),uu_temp(:,tt), pp(:,tt+1));
        
        [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), xx_des(:,tt), uu_des(:,tt), params);
        JJtemp = JJtemp + cost_dummy;
      end
      
      [cost_dummy, ~] = Term_Cost(xx_temp(:,TT), xx_des(:,TT), params);
      JJtemp = JJtemp + cost_dummy;
      
      cost_arm = [cost_arm; JJtemp];
      
    end % End while
    
    gamma_steps = gammas;
    gamma = gammas(end);
    
  else
    
    gamma = gamma_fix;
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %   Descent direction plot
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  if visu_armijo==1
    points = 10;
    
    gamma_stepsizes = linspace(1,0,points);
    
    cost_temp = zeros(points,1);
    xx_temp = zeros(nn, TT);
    uu_temp = zeros(nu, TT);
    
    for ii = 1:length(gamma_stepsizes)
      
      gamma_i = gamma_stepsizes(ii);
      
      % Evaluate cost for gamma_i
      xx_temp(:,1) = xx_des(:,1);
      for tt = 1:TT-1
        uu_temp(:,tt) = uu(:,tt,kk) + gamma_i*sigma(:,tt) + ...
          KK(:,:,tt)*(xx_temp(:,tt) - xx(:,tt,kk));
        %
        [xx_temp(:,tt+1),~] = Dynamics(xx_temp(:,tt),uu_temp(:,tt),...
          pp(:,tt+1));
        %
        [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), ...
          xx_des(:,tt), uu_des(:,tt), params);
        JJtemp = JJtemp + cost_dummy;
        %
        [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), ...
          xx_des(:,tt), uu_des(:,tt), params);
        cost_temp(ii) = cost_temp(ii) + cost_dummy;
      end
      
      [cost_dummy, ~] = Term_Cost(xx_temp(:,TT), xx_des(:,TT), params);
      cost_temp(ii) = cost_temp(ii) + cost_dummy;
          
    end
    
    figure(1900); % Descent plot Armijo
    
    plot(gamma_stepsizes, min(cost_temp, 5*JJ(kk)), 'LineWidth',2, ...
      'DisplayName','$J(x^k - \gamma\nabla J^k)$');
    hold on;
    grid on;
    zoom on;
    %
    plot(gamma_stepsizes, JJ(kk) + gamma_stepsizes*descent(kk), ...
      'r--', 'LineWidth',2,'DisplayName','$J(x^k) + \gamma descent$');
    %
    plot(gamma_stepsizes, JJ(kk) + cc*gamma_stepsizes*descent(kk), ...
      'g--', 'LineWidth',2,'DisplayName','$J(x^k) + c \gamma descent$');
    %
    plot(gammas, cost_arm, '*', 'DisplayName', 'Armijo steps');
    zoom on
    
    tit = sprintf('Iter: %d', kk);
    title(tit);
    
    legend('Interpreter','latex', 'FontSize', 12, 'Location','southeast');
    
    hold off
    
    drawnow    
  end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Update trajectory
  
xx(:,1,kk+1) = xx_des(:,1);
  for tt=1:TT-1
    %
    uu(:,tt,kk+1) = uu(:,tt,kk) + gamma*sigma(:,tt) + KK(:,:,tt)*(xx(:,tt,kk+1) - xx(:,tt,kk));
    %
    [xx(:,tt+1, kk+1),~] = Dynamics(xx(:,tt,kk+1), uu(:,tt,kk+1), pp(:,tt+1));
    %
    [cost_dummy, ~,] = Stage_Cost(xx(:,tt,kk+1), uu(:,tt,kk+1), xx_des(:,tt), uu_des(:,tt), params);
    JJ(kk+1) = JJ(kk+1) + cost_dummy;   
  end

  [cost_dummy, ~] = Term_Cost(xx(:,TT,kk+1), xx_des(:,TT), params);
  JJ(kk+1) = JJ(kk+1) + cost_dummy;
  
  fprintf('Iter: %d\n',kk);
  fprintf('descent: %.4e\n', descent(kk));
  fprintf('cost: %.4e\n', JJ(kk));
  
  if abs(descent(kk))<tol
    max_iters = kk;
    fprintf('Tolerance reached!\n');
    break;
  end
  
end % main loop

%% Add last samples (for plots)

uu(:,TT,max_iters) = uu(:,TT-1,max_iters);

%% Plots

% PLOT OF RESULTING STATE AND INPUT TRAJECTORIES

star = max_iters;

figure(3); % Angle of the first joint

stairs(1:TT, xx(1,:,star),'LineWidth',2);
hold on;
stairs(1:TT, xx_des(1,:),'--','LineWidth',2);
ylabel('x1_t [rad]');
xlabel('t');
grid on;
zoom on;
title('Evolution of the angle of the first joint')
legend({'Evolution','Desidered'})
legend('Location','best');

figure(4); % Velocity of the first joint

stairs(1:TT, xx(2,:,star),'LineWidth',2);
hold on;
stairs(1:TT, xx_des(2,:),'--','LineWidth',2);
ylabel('x2_t [rad/s]');
xlabel('t');
grid on;
zoom on;
title('Evolution of the velocity of the first join')
legend({'Evolution','Desidered'})
legend('Location','best');

figure(5); % Angle of the second joint

stairs(1:TT, xx(3,:,star),'LineWidth',2);
hold on;
stairs(1:TT, xx_des(3,:),'--','LineWidth',2);
ylabel('x3_t [rad]');
xlabel('t');
grid on;
zoom on;
title('Evolution of the angle of the second joint')
legend({'Evolution','Desidered'})
legend('Location','best');

figure(6); % Velocity of the second joint

stairs(1:TT, xx(4,:,star),'LineWidth',2);
hold on;
stairs(1:TT, xx_des(4,:),'--','LineWidth',2);
ylabel('x4_t [rad/s]');
xlabel('t');
grid on;
zoom on;
title('Evolution of the velocity of the second joint')
legend({'Evolution','Desidered'})
legend('Location','best');

figure(7); % Torque of the first joint

stairs(1:TT, uu(1,:,star),'LineWidth',2);
hold on;
stairs(1:TT, uu_des(1,:),'--','LineWidth',2);
ylabel('u1_t [Nm]');
xlabel('t');
grid on;
zoom on;
title('Evolution of the torque of the first joint')
legend({'Evolution','Desidered'})
legend('Location','best');

figure(8); % Torque of the second joint

stairs(1:TT, uu(2,:,star),'LineWidth',2);
hold on;
stairs(1:TT, uu_des(2,:),'--','LineWidth',2);
ylabel('u2_t [Nm])');
xlabel('t');
grid on;
zoom on;
title('Evolution of the torque of the second joint')
legend({'Evolution','Desidered'})
legend('Location','best');

% Descent direction
figure(9);
semilogy(1:max_iters, abs(descent(1:max_iters)), 'LineWidth',2);
ylabel('descent');
xlabel('iter')
grid on;
zoom on;
title('Descent direction');

% Cost error (normalized)
figure(10);
semilogy(1:max_iters, abs((JJ(1:max_iters)-JJ(max_iters))/JJ(max_iters)), 'LineWidth',2);
ylabel('J(u^k)-J(u^{max})/J(u^{max})');
xlabel('iter')
grid on;
zoom on;
title('Normalized Cost Error');



