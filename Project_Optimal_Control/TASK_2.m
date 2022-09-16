%% OPTIMAL CONTROL - Course Project #1
% Optimal Control of a Robotic Manipulator
% Group 27:
% Santoro Luca, 0001005415
% Armando Spennato, 0001006172
% Professor: Giuseppe Notarstefano   Tutor: Lorenzo Sforni

% Chapter 3
%% TASK 2 - Trajectory Optimization

close all; clear; clc
addpath ./Functions;
addpath ./RobotManipulator;
%{ 
Now we use our manipulator to draw a desired curve in the workspace.
In particular, we exploit again the DDP in order to define a trajectory in the joint 
space that our manipulator should follow in order to obtain the desired shape.
%}

%% Parameters definition

max_iters = 2e2;
tol = 1e-6;

tf = 30; % seconds

params.dyn.dt = 1e-3;
params.dyn.mm1 = 2;         % kg
params.dyn.mm2 = 2;         % kg
params.dyn.gg = 9.81;       % m/s^2
params.dyn.ll1 = 1;         % m
params.dyn.ll2 = 1;         % m
params.dyn.rr1 = 0.5;       % m
params.dyn.rr2 = 0.5;       % m
params.dyn.J_iner1 = 0.5;   % kg*m^2
params.dyn.J_iner2 = 0.5;   % kg*m^2

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

% We define the weights of the matrix Q
wq1 = 1000;
wq2 = 10;
wq3 = 1000;
wq4 = 10;
params.cost.QQ = [wq1, 0, 0, 0; ...
                  0, wq2, 0, 0; ...
                  0, 0, wq3, 0; ...
                  0, 0, 0, wq4];
params.cost.QQf = [wq1, 0, 0, 0; ...
                  0, wq2, 0, 0; ...
                  0, 0, wq3, 0; ...
                  0, 0, 0, wq4];         

% We define the weights of the matrix R
wr1 = 0.0005;
wr2 = 0.0005;
params.cost.RR = [wr1, 0;...
                  0, wr2];

TT = tf/params.dyn.dt;

state_dim = 4;
input_dim = 2;

% Flags
visu_workspace_flag =1; % To visualize the workspace consideration
visu_traj = 0; % To understand the behaviour in real time.
plot_flag = 1;

% Flags for Armijo
ARMIJO_flag = 1;
gamma_fix = 0.5;



fprintf("Parameters defined\n")
%% Visualizazation possible configuration in Workspace

if visu_workspace_flag == 1

% With the following piece of code we can see the points that the
% end-effector can reach in the workspace, the reachable area depends on
% the length of two links and relative angles theta1 and theta2.

% For example:
% Assume that the first joint has limited freedom to rotate and it can rotate between 0 and 180 degrees. 
% Similarly, assume that the second joint has limited freedom to rotate and can rotate between 0 and 180 degrees. 
% Hence, 0<=theta1<=pi and 0<=theta2<=pi.

ttheta1 = 0:0.1:pi; % all possible theta1 values
ttheta2 = 0:0.1:pi; % all possible theta2 values

[THETA1,THETA2] = meshgrid(ttheta1,ttheta2); % generate grid of angle values

X = ll1 * cos(THETA1) + ll2 * cos(THETA1 + THETA2); % compute x coordinates
Y = ll1 * sin(THETA1) + ll2 * sin(THETA1 + THETA2); % compute y coordinates


%example2 0<=theta1<=2*pi and 0<=theta2<=2*pi.
theta11 = 0:0.1:2*pi; % all possible theta1 values
theta22 = 0:0.1:2*pi; % all possible theta2 values

[THETA11,THETA22] = meshgrid(theta11,theta22); % generate grid of angle values

X2 = ll1 * cos(THETA11) + ll2 * cos(THETA11 + THETA22); % compute x coordinates
Y2 = ll1 * sin(THETA11) + ll2 * sin(THETA11 + THETA22); % compute y coordinates


 figure(1);
    sgt = sgtitle('X-Y coordinates for all theta1 and theta2 combinations');
    sgt.FontSize = 15;
    subplot(1,2,1)
    plot(X(:),Y(:),'r.');
    axis equal;
    grid on
    title( '$$0<=\theta_{1}<=\pi$$ and $$0<=\theta_{2}<=\pi$$' ,'Interpreter','latex', 'FontSize',10);
    ylabel('Y');
    xlabel('X');
    subplot(1,2,2)
    plot(X2(:),Y2(:),'r.');
    axis equal;
    grid on;
    title( '$$0<=\theta_{1}<=2\pi$$ and $$0<=\theta_{2}<=2\pi$$' ,'Interpreter','latex', 'FontSize',10);
    ylabel('Y');
    xlabel('X');

end
%% Shape definition
% Due to the length of the two links we have that the work area is a circle
% with a radius of 2 [m] around the origin.

% In choosing the shape of the curve to draw we must respect the previous constraint

% Limit Work area
r = 2; % m

x_circle = zeros(24000,1); 
y_circle = zeros(24000,1); 
x_off = 0; % Center of the circle
y_off = 0; % Center of the circle

l1 = 1; % lengths of the joint 1 
l2 = 1; % lengths of the joint 2 

i = 1; % first index of a vector

for teta = 0:2.6180e-04:2*pi
    x_circle(i) = r*cos(teta)+x_off; % parametric expression of a circle
    y_circle(i) = r*sin(teta)+y_off;
    i = i+1;
end

%{
 We choose as curve an Ellipse, that has the following equation:
           ((x^2 - α )^2)/ a^2 + ((y^2 - β )^2)/ b^2 = 1

That is the equation of a translated ellipse, where α  and  β  are the coordinates 
of the center of the ellipse.
we define: α=0 [m]; β =1,5 [m]; a=1 [m]; b=0,3 [m];
(a and b are respectively the half of the greater length and the less one)
%}

% Ellipse 

aa = 1; % m
bb = 0.3; % m

x_ellipse = zeros(24000,1); 
y_ellipse = zeros(24000,1); 
x_off_ellipse = 0; % Center of the ellipse
y_off_ellipse = 1.5; % Center of the ellipse

 

i = 1; % first index of a vector

for teta = 0:2.6180e-04:2*pi
    x_ellipse(i) = aa*cos(teta)+x_off_ellipse; % parametric expression of a Ellipse
    y_ellipse(i) = bb*sin(teta)+y_off_ellipse;
    i = i+1;
end

% With the following plots we visualize the curve shape in the Workspace: 

figure(2) % Shape curve in the Workspace

plot(x_circle,y_circle,'LineWidth',2) % Boundaary area
axis equal
grid on
hold on
plot(x_ellipse,y_ellipse,'LineWidth',2) % Desidered curve Ellipse
plot(x_off_ellipse,y_off_ellipse,'.','LineWidth',2) % Cente of the ellipse
plot(0,0,'x','LineWidth',2)         % Origin work area

title('Shape Curve');
legend({'Boundary area ','Desidered Ellipse','Center Ellipse','Origin work area'});
legend('Location','best');         % Legend settings.

%% Inverse kinematic

teta1 = zeros(size(x_ellipse)); % Vectors of the same dimension 
teta2 = zeros(size(x_ellipse)); % of the number of points of the shape

for i=1:size(x_ellipse,1)
    x = x_ellipse(i); % Define the point for which 
    y = y_ellipse(i); % we require the IK solution

    % IK solution:
    cost2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
    sint2 = sqrt(1-cost2^2);

    if (imag(sint2)~=0) % check if point is outside workspace 
      disp("IK ERROR"); % (-1<= cost2 <=1)
    end

    teta2(i) = atan2(sint2,cost2);

    teta1(i) = atan2(y,x)-atan2((l1*sint2),(l1+l2*cost2));
    %slide pag 123/144

% we have the same result with: 
%     sint1 = ((l1+l2*cost2)*y-l2*sint2*x)/(x^2+y^2);
%     cost1 = ((l1+l2*cost2)*x+l2*sint2*y)/(x^2+y^2);
% 
%     teta1(i) = atan2(sint1,cost1);

end

% With the following graphs we visualize the theta1 and theta2 angles 
% found with Inverse Kinematics: 

b=[0:1:24000-1]'; 

figure(100) % theta1 output InverseKinematics
plot(b,rad2deg(teta1(:)),'.');
grid on;
title( '$$\theta_{1}$$ [deg] (Inverse Kinematics)' ,'Interpreter','latex', 'FontSize',20);
ylabel('$$\theta_{1}$$','Interpreter','latex', 'FontSize',15);
xlabel('t');

figure(101) % theta2 output InverseKinematics
plot(b,rad2deg(teta2(:)),'.');
grid on;
title( '$$\theta_{2}$$ [deg] (Inverse Kinematics)' ,'Interpreter','latex', 'FontSize',20);
ylabel('$$\theta_{2}$$','Interpreter','latex', 'FontSize',15);
xlabel('t');
%% REFERENCE DEFINITION
% Now we have to define the reference signal that we give as input to the
% DDP in order to obtain the optimal trajectory in joint space.
% 
% We choose the following reference:
%  --> Initial (start) position teta = (45,30) for 2 [s];
%  --> Fifth order polynomial to the beginning of the ellipse for 4 [s]
%  --> Ellipse for the remaining 24 seconds
%  30s ----> 30000 samples.

xx_ref = zeros(state_dim, TT);
uu_ref = zeros(input_dim, TT);

% POLYNOMIAL DEFINITION, in order to start from our initial position and 
% to reach the point to draw the ellipse:
teta1_dot_init = (teta1(20)-teta1(1))/(20);
teta2_dot_init = (teta2(20)-teta2(1))/(20);

a01 = deg2rad(45);
a11 = 0;
a21 = 0;
a31 = 1/(2*4001^3)*(20*(teta1(1)-deg2rad(45)) - 8*4001*teta1_dot_init);
a41 = 1/(2*4001^4)*(30*(-teta1(1)+deg2rad(45)) + 14*4001*teta1_dot_init);
a51 = 1/(2*4001^5)*(12*(teta1(1)-deg2rad(45)) - 6*4001*teta1_dot_init);

% initial position theta1
xx_ref(1,1:1999) = deg2rad(45); % Because we want to stay at the position (45,30) for two seconds


for i=2000:6000
    % We build the the polynomial trajectory:
    xx_ref(1,i) = a01 + a31*(i-1999)^3 + a41*(i-1999)^4 + a51*(i-1999)^5;
end
xx_ref(1,6001:end) = teta1(:);


a03 = deg2rad(30);
a13 = 0;
a23 = 0;
a33 = 1/(2*4001^3)*(20*(teta2(1)-deg2rad(30)) - 8*4001*teta2_dot_init);
a43 = 1/(2*4001^4)*(30*(deg2rad(30)-teta2(1)) + 14*4001*teta2_dot_init);
a53 = 1/(2*4001^5)*(12*(teta2(1)-deg2rad(30)) - 6*4001*teta2_dot_init);

% Initial position Theta2
xx_ref(3,1:1999) = deg2rad(30); % Because we want to stay at the position (45,30) for two seconds.

for i=2000:6000
    % We build the polynomial trajectory:
    xx_ref(3,i) = a03 + a33*(i-1999)^3 + a43*(i-1999)^4 + a53*(i-1999)^5;    
end
xx_ref(3,6001:end) = teta2(:);

% Now we have tu define the input reference vector:
% u_ref is such that it balances the g(q_ref) term. In this way, it keeps
% the manipulator in the desired equilibrium position, with q_dot and
% q_doubledot equal to zero
for i=1:TT
    uu_ref(1,i) = (mm1*rr1+mm2*ll1)*gg*cos(xx_ref(1,i))+mm2*gg*rr2*cos(xx_ref(1,i)+xx_ref(3,i));
    uu_ref(2,i) = mm2*gg*rr2*cos(xx_ref(1,i)+xx_ref(3,i));
end

% Plots references 
if plot_flag == 1
    figure(3);
    sgt = sgtitle('Reference angles');
    sgt.FontSize = 15;
    subplot(1,2,1)
    plot(rad2deg(xx_ref(1,:)),'LineWidth',2);
    grid on
    title('\theta_1 reference');
    ylabel('\theta (deg)');
    xlabel('t');
    subplot(1,2,2)
    plot(rad2deg(xx_ref(3,:)),'LineWidth',2);
    grid on
    title('\theta_2 reference');
    ylabel('\theta (deg)');
    xlabel('t');
    
    figure(4);
    sgt = sgtitle('Reference input');
    sgt.FontSize = 15;
    subplot(1,2,1)
    plot(uu_ref(1,:),'LineWidth',2);
    grid on
    ylabel('u (Nm)');
    xlabel('t');
    title('u_1 reference');
    subplot(1,2,2)
    plot(uu_ref(2,:),'LineWidth',2);
    grid on
    ylabel('u (Nm)');
    xlabel('t');
    title('u_2 reference');
end

fprintf("Reference defined\n")

%% DDP IMPLEMENTATION
xx_star = zeros(state_dim, TT, max_iters);
uu_star = zeros(input_dim, TT, max_iters);

% INITIALIZATION of x and u: we use the inverse dynamic control in order 
% to not start from a trajectory too far from the final one. 

[xx_star(:,:,1), uu_star(:,:,1)] = controller(xx_ref,uu_ref, params, 2);

JJ = zeros(max_iters,1);
descent = zeros(max_iters,1);

fprintf('-*-*-*-*-*-\n');

kk = 1; % iteration index

for tt=1:TT-1
  % In this for loop we build the FIRST iteration of the cost based on
  % state and input from the initial instant to the second last one
  [cost_dummy, ~] = Stage_Cost(xx_star(:,tt,kk), uu_star(:,tt,kk), xx_ref(:,tt), uu_ref(:,tt), params);
  JJ(kk) = JJ(kk) + cost_dummy;
end

% Here we define the last value of the cost, exploiting the state at the
% last instant (TT)
[cost_dummy, ~] = Term_Cost(xx_star(:,TT,kk), xx_ref(:,TT), params);
JJ(kk) = JJ(kk) + cost_dummy;

% MAIN LOOP (runs until max_iters or until tolerance is reached)
for kk=1:max_iters-1 
 
    if visu_traj
    
    figure(30);

    stairs(1:TT, xx_star(1,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(1,:),'--','LineWidth',2);
    ylabel('x1_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Position Joint1');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow

    figure(31);
    stairs(1:TT, xx_star(2,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(2,:),'--','LineWidth',2);
    ylabel('x2_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Velocity Joint1');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow
 
    figure(32);
    stairs(1:TT, xx_star(3,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(3,:),'--','LineWidth',2);
    ylabel('x3_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Position Joint2');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow
 
    figure(33);
    stairs(1:TT, xx_star(4,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(4,:),'--','LineWidth',2);
    ylabel('x4_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Velocity Joint2');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow
    
    figure(34);
    stairs(1:TT, uu_star(1,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, uu_ref(1,:),'--','LineWidth',2);
    ylabel('u1_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Input Joint1');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow

    figure(35);
    stairs(1:TT, uu_star(2,:,kk),'LineWidth',2);
    hold on;
    stairs(1:TT, uu_ref(2,:),'--','LineWidth',2);
    ylabel('u2_t');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('Evolution Input Joint2');
    legend({'Evolution','Reference'});
    legend('Location','best'); 
    drawnow
   
    %pause % uncomment to stop at each iteration
    
  end
 
  KK = zeros(input_dim,state_dim, TT);
  sigma = zeros(input_dim, TT);
  
  pp = zeros(state_dim, TT);
  PP = zeros(state_dim,state_dim, TT);
  
  % Initialization of the terms p and P that have to be used to perform the
  % control:
  % p_T = q_T = gradient of the terminal cost
  % P_T = Q_T = hessian of the terminal cost
  % The gradient and the hessian are given as second and third outputs by 
  % the function term_cost
  [~, pp(:,TT), PP(:,:,TT)] = Term_Cost(xx_star(:,TT,kk), xx_ref(:,TT), params);
  
% Backward iteration
  for tt = TT-1:-1:1
    
    [~, fx, fu, pfxx, pfuu, pfux] = Dynamics(xx_star(:,tt,kk), uu_star(:,tt,kk),pp(:,tt+1));
    [~, lx, lu, lxx, luu, lux] = Stage_Cost(xx_star(:,tt,kk), uu_star(:,tt,kk),xx_ref(:,tt), uu_ref(:,tt), params);

    % Compute gain and ff descent direction
    
    KK(:,:,tt) = -(luu + fu*PP(:,:,tt+1)*fu'+ pfuu)\(lux + fu*PP(:,:,tt+1)*fx' + pfux);
    %                            
    sigma(:,tt) = -(luu + fu*PP(:,:,tt+1)*fu'+ pfuu)\(lu + fu*pp(:,tt+1));
                              
    % Update PP and pp
    PP(:,:,tt) = (lxx + fx*PP(:, :, tt+1)*fx' + pfxx) - KK(:, :,tt)'*(luu + fu*PP(:,:,tt+1)*fu' + pfuu)*KK(:, :, tt);
    %
    pp(:,tt) = (lx + fx*pp(:,tt+1))- KK(:, :,tt)'*(luu + fu*PP(:,:,tt+1)*fu' + pfuu)*sigma(:, tt);
    %
    descent(kk) = descent(kk) + (lu + fu*pp(:,tt+1))'*sigma(:,tt);
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %   ARMIJO gamma_stepsize selection %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %gamma_stepsize selection
  if ARMIJO_flag
    
    cc = 0.1;
    rho = 0.5;
    gammas = 1;
    
    cost_arm = [];
    
    xx_temp = zeros(state_dim,TT);
    uu_temp = zeros(input_dim,TT);

    xx_temp(:,1) = xx_ref(:,1);
    JJtemp = 0;
    
    for tt = 1:TT-1
      
      uu_temp(:,tt) = uu_star(:,tt,kk) + gammas(end)*sigma(:,tt) + ...
        KK(:,:,tt)*(xx_temp(:,tt) - xx_star(:,tt,kk));
      %
      [xx_temp(:,tt+1),~] = Dynamics(xx_temp(:,tt),uu_temp(:,tt),...
        pp(:,tt+1));
      %
      [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), ...
        xx_ref(:,tt), uu_ref(:,tt), params);
      JJtemp = JJtemp + cost_dummy;
    end
    
    [cost_dummy, ~] = Term_Cost(xx_temp(:,TT), xx_ref(:,TT), params);
    
    JJtemp = JJtemp + cost_dummy;
    
    cost_arm = [cost_arm; JJtemp];
    
    % ARMIJO LOOP
    
    while cost_arm(end) > JJ(kk) + cc*gammas(end)*descent(kk)
      
      gammas = [gammas; gammas(end)*rho];
      
      % Evaluate cost for gamma_i
      xx_temp(:,1) = xx_ref(:,1);
      
      JJtemp = 0;
      
      for tt = 1:TT-1
        % Compute the input
        uu_temp(:,tt) = uu_star(:,tt,kk) + gammas(end)*sigma(:,tt) + KK(:,:,tt)*(xx_temp(:,tt) - xx_star(:,tt,kk));
        
        [xx_temp(:,tt+1),~] = Dynamics(xx_temp(:,tt),uu_temp(:,tt), pp(:,tt+1));
        
        [cost_dummy, ~,~] = Stage_Cost(xx_temp(:,tt), uu_temp(:,tt), xx_ref(:,tt), uu_ref(:,tt), params);
        JJtemp = JJtemp + cost_dummy;
      end
      
      [cost_dummy, ~] = Term_Cost(xx_temp(:,TT), xx_ref(:,TT), params);
      JJtemp = JJtemp + cost_dummy;
      
      cost_arm = [cost_arm; JJtemp];
      
    end
    
    gamma_steps = gammas;
    gamma = gammas(end);
    
  else
    
    gamma = gamma_fix;
    
  end
  
% Update trajectory
  
xx_star(:,1,kk+1) = xx_ref(:,1);
  for tt=1:TT-1
    
    uu_star(:,tt,kk+1) = uu_star(:,tt,kk) + gamma*sigma(:,tt) + KK(:,:,tt)*(xx_star(:,tt,kk+1) - xx_star(:,tt,kk));
    %
    [xx_star(:,tt+1, kk+1),~] = Dynamics(xx_star(:,tt,kk+1), uu_star(:,tt,kk+1), pp(:,tt+1));
    %
    [cost_dummy, ~,] = Stage_Cost(xx_star(:,tt,kk+1), uu_star(:,tt,kk+1), xx_ref(:,tt), uu_ref(:,tt), params);
    
    JJ(kk+1) = JJ(kk+1) + cost_dummy;
    
  end
  
  [cost_dummy, ~] = Term_Cost(xx_star(:,TT,kk+1), xx_ref(:,TT), params);
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

uu_star(:,TT,max_iters) = uu_star(:,TT-1,max_iters);
%% Plots

% PLOT OF RESULTING STATE AND INPUT TRAJECTORIES

star = max_iters;

if plot_flag == 1

    figure(5); %theta1 [rad]
    
    stairs(1:TT, xx_star(1,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(1,:),'--','LineWidth',2);
    ylabel('x1_t (rad)');
    xlabel('t');
    grid on;
    zoom on;
    title('Position joint1');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    figure(6);  %theta1_dot [rad]
    stairs(1:TT, xx_star(2,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(2,:),'--','LineWidth',2);
    ylabel('x2_t (rad/s)');
    xlabel('t');
    grid on;
    zoom on;
    title('Velocity joint1');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    figure(7); %theta2 [rad]
    stairs(1:TT, xx_star(3,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(3,:),'--','LineWidth',2);
    ylabel('x3_t (rad)');
    xlabel('t');
    grid on;
    zoom on;
    title('Position joint2');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    figure(8); %theta2_dot [rad]
    stairs(1:TT, xx_star(4,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, xx_ref(4,:),'--','LineWidth',2);
    ylabel('x4_t (rad/s)');
    xlabel('t');
    grid on;
    zoom on;
    title('Velocity joint2');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    figure(9); %input first joint
    stairs(1:TT, uu_star(1,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, uu_ref(1,:),'--','LineWidth',2);
    ylabel('u1_t (Nm)');
    xlabel('t');
    grid on;
    zoom on;
    title('input(Torque) joint1');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    figure(10); %input second joint
    stairs(1:TT, uu_star(2,:,star),'LineWidth',2);
    hold on;
    stairs(1:TT, uu_ref(2,:),'--','LineWidth',2);
    ylabel('u2_t (Nm)');
    xlabel('t');
    grid on;
    zoom on;
    title('input(Torque) joint2');
    legend({'Evolution','Reference'});
    legend('Location','best');         % Legend settings.
    
    % Descent direction
    figure(11);
    semilogy(1:max_iters, abs(descent(1:max_iters)), 'LineWidth',2);
    ylabel('descent');
    xlabel('iter')
    grid on;
    zoom on;
    title('Descent direction');
    
    % Cost error (normalized)
    figure(12);
    semilogy(1:max_iters, abs((JJ(1:max_iters)-JJ(max_iters))/JJ(max_iters)), 'LineWidth',2);
    ylabel('J(u^k)-J(u^{max})/J(u^{max})');
    xlabel('iter')
    grid on;
    zoom on;
    title('Normalized Cost Error');

end

