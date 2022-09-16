%% Controller

% Group 27: Luca Santoro, Armando Spennato

function [output1,output2] = controller(xx_ref,uu_ref, params, task)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function allows to implement the inverse dynamic control.
% Argument
%       - xx_ref --> reference state at time t
%       - uu_ref --> reference input at time t 
%       - params --> Parameters of our system
%       
%   Return
%       - xx_init --> inizialization state  
%       - uu_init --> inizialization input torque
%
% We have choose this control in order not to start from a trajectory too 
% far from the final one.
% The control law is the following:
%
%      u= M(q)*(Kp*q_tilde + Kd*q_tildedot) + c(q,q')q'+g(q)
%
%   Where:
%     Kp,Kd ∈ R^2x2
%     M(q) ∈ R^2x2
%     c(q,q') ∈ R^2x2
%     g(q) ∈ R^2 
%     q_tilde ∈ R^2 Position error
%     q_tildadot ∈ R^2 Velocity error
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
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
TT = 30/dt;
state_dim = 4;
input_dim = 2;

% Plot flags
plot_flag = 0;
plot_flag_tuning=0;

% Our State vector:
%  x(t):= [x1(t) x2(t) x3(t) x4(t)]'=[θ1(t) θ1(t)' θ2(t) θ2(t)']' ∈ R^4 
% Our Input vector:
%  u(t):=[u1(t) u2(t)]'=[τ1(t) τ2(t)]' ∈ R^2
% 
% The joint variables as follow:
% 
% Joint position vector --> q:= [θ1 θ2]' = [x1(t) x3(t)]'
% Joint velocity vector --> q':= [θ1' θ2']' = [x2(t) x4(t)]'
% Joint acceleration vector --> q'':= [θ1'' θ2'']'= [x2'(t) x4'(t)]'

% qq = [xx(1);xx(3)];
% qq_dot = [xx(2);xx(4)];
% qq_doubledot = MM\(uu - CC*qq_dot - GG);

x_tilda = zeros(input_dim,TT); % position error
x_dot_tilda = zeros(input_dim,TT); % velocity error
 
% Control parameters 
% Control paramaters used in the first Task
if task == 1
  Kp = [25, 0;...
        0, 25];
  Kd = [10, 0;...
      0, 10];
end
% Control paramaters used in the second Task
if task == 2
    Kp = [80, 0;...
              0, 80];
    Kd = [10, 0;...
              0, 10];
end

gravity = zeros(input_dim,1,TT); 
Mass= zeros(input_dim,input_dim,TT);
CC=zeros(input_dim,input_dim,TT);

kk = 1; % first iteration
xx_init = zeros(state_dim, TT, kk);
uu = zeros(input_dim, TT, kk);

uu_init = zeros(input_dim, TT, kk);

xx_init(1,:,1) = deg2rad(45); % initialize x
xx_init(3,:,1) = deg2rad(30);

pp = zeros(state_dim,TT); % initialized to 0 because it is not needed (when we call the function Dynamics)
xx_dot = zeros(state_dim/2,TT);

for i=1:TT-1
        % Compute position errors:
        x_tilda(1,i) = xx_ref(1,i) - xx_init(1,i,kk); 
        x_tilda(2,i) = xx_ref(3,i) - xx_init(3,i,kk);
        % Compute velocity errors:
        x_dot_tilda(1,i) = xx_ref(2,i) - xx_init(2,i,kk); 
        x_dot_tilda(2,i) = xx_ref(4,i) - xx_init(4,i,kk);
% Now let's calculate all the parameters necessary to compensate for our
% initial (natural) dynamics:
% 1) Matrix M(q):
%    MM = [mm1*rr1^2+mm2*(ll1^2+rr2^2+2*ll1*rr2*cos(xx(3)))+JJ1+JJ2, mm2*(rr2^2+ll1*rr2*cos(xx(3)))+JJ2;...
%     mm2*(rr2^2+ll1*rr2*cos(xx(3)))+JJ2, mm2*rr2^2+JJ2];
       Mass(1,1,i)= mm1*rr1^2+mm2*(ll1^2+rr2^2+2*ll1*rr2*cos(xx_init(3,i,kk)))+J_iner1+J_iner2;
       Mass(1,2,i)= mm2*(rr2^2+ll1*rr2*cos(xx_init(3,i,kk)))+J_iner2;
       Mass(2,1,i)= mm2*(rr2^2+ll1*rr2*cos(xx_init(3,i,kk)))+J_iner2;
       Mass(2,2,i)= mm2*rr2^2+J_iner2;
% 2) Matrix c(q,q')
%    CC = -mm2*ll1*rr2*sin(xx(3))*[xx(4), xx(2)+xx(4); -xx(2), 0];
       CC(1,1,i)= -mm2*ll1*rr2*sin(xx_init(3,i,kk))*xx_init(4,i,kk);
       CC(1,2,i)= -mm2*ll1*rr2*sin(xx_init(3,i,kk))*(xx_init(2,i,kk)+xx_init(4,i,kk));
       CC(2,1,i)= mm2*ll1*rr2*sin(xx_init(3,i,kk))*xx_init(2,i,kk);
       CC(2,2,i)= 0;

% 3) Gravitational vector g(q)
%    GG = [(mm1*rr1+mm2*ll1)*gg*cos(xx(1))+mm2*gg*rr2*cos(xx(1)+xx(3));...
%     mm2*gg*rr2*cos(xx(1)+xx(3))];
       gravity(1,1,i) = (mm1*rr1+mm2*ll1)*gg*cos(xx_init(1,i,kk))+mm2*gg*rr2*cos(xx_init(1,i,kk)+xx_init(3,i,kk));
       gravity(2,1,i) = mm2*gg*rr2*cos(xx_init(1,i,kk)+xx_init(3,i,kk));
 
% We inizialize the velocity term
       xx_dot(1,i) = xx_init(2,i,1); 
       xx_dot(2,i) = xx_init(4,i,1);
% Update u (CONTROL ACTION)
       uu(:,i,kk) = Mass(:,:,i)*( Kp*x_tilda(:,i) + Kd*x_dot_tilda(:,i)) + CC(:,:,i)*xx_dot(:,i) + gravity(:,1,i);
% Update dynamics
        [xx_init(:,i+1, kk),~] = Dynamics(xx_init(:,i,kk), uu(:,i,kk),pp(:,i));
end

% Update last samples gravity 
    gravity(1,1,TT) = (mm1*rr1+mm2*ll1)*gg*cos(xx_init(1,TT,kk))+mm2*gg*rr2*cos(xx_init(1,TT,kk)+xx_init(3,TT,kk));
    gravity(2,1,TT) = mm2*gg*rr2*cos(xx_init(1,TT,kk)+xx_init(3,TT,kk));
% Update last samples mass
    Mass(1,1,TT)= mm1*rr1^2+mm2*(ll1^2+rr2^2+2*ll1*rr2*cos(xx_init(3,TT,kk)))+J_iner1+J_iner2;
    Mass(1,2,TT)= mm2*(rr2^2+ll1*rr2*cos(xx_init(3,TT,kk)))+J_iner2;
    Mass(2,1,TT)= mm2*(rr2^2+ll1*rr2*cos(xx_init(3,TT,kk)))+J_iner2;
    Mass(2,2,TT)= mm2*rr2^2+J_iner2;
 % Update last samples cc  
    CC(1,1,TT)= -mm2*ll1*rr2*sin(xx_init(3,TT,kk))*xx_init(4,TT,kk);
    CC(1,2,TT)= -mm2*ll1*rr2*sin(xx_init(3,TT,kk))*(xx_init(2,TT,kk)+xx_init(4,TT,kk));
    CC(2,1,TT)= mm2*ll1*rr2*sin(xx_init(3,TT,kk))*xx_init(2,TT,kk);
    CC(2,2,TT)= 0;


%     uu_init(1,:,kk) = gravity(1,1,:);
%     uu_init(2,:,kk) = gravity(2,1,:);

    uu(:,TT,kk) = uu(:,TT-1,kk);
    uu_init(:,:,kk) = uu(:,:,kk); 

    output1 = xx_init(:,:,1);
    output2 = uu_init(:,:,1);

 if plot_flag    % plots 

        figure(21); % Plot of the position 
        sgt = sgtitle('State (Angles) inizialization (Controller)');
        sgt.FontSize = 15;

        subplot(2,2,1)
        plot(rad2deg((xx_init(1,:,1))),'LineWidth',2,'Color','r');
        grid on
        zoom on
        title('\theta_1 init');
        ylabel('\theta (deg)');
        xlabel('t');
        
        subplot(2,2,2)
        plot(rad2deg((xx_init(3,:,1))),'LineWidth',2,'Color','r');
        grid on
        zoom on
        title('\theta_2 init');
        ylabel('\theta (deg)');
        xlabel('t');
        
        % Reference
        subplot(2,2,3)
        plot(rad2deg((xx_ref(1,:,1))),'LineWidth',2);
        grid on
        title('\theta_1 ref');
        ylabel('\theta (deg)');
        xlabel('t');
        subplot(2,2,4)
        plot(rad2deg((xx_ref(3,:,1))),'LineWidth',2);
        grid on
        title('\theta_2 ref');
        ylabel('\theta (deg)');
        xlabel('t');
        

        figure(22); % Plot position error
        sgt = sgtitle('Position error (Controller)');
        sgt.FontSize = 20;
        
        subplot(1,2,1)
        plot(x_tilda(1,:),'LineWidth',2);
        grid on
        title('$$\tilde{x}_{1}$$(First joint)','Interpreter','latex', 'FontSize',15);
        xlabel('t');
        subplot(1,2,2)
        plot(x_tilda(2,:),'LineWidth',2);
        grid on
        title('$$\tilde{x}_{2}$$(Second joint)','Interpreter','latex', 'FontSize',15);
        xlabel('t');

        figure(23); % Plot velocity error
        sgt = sgtitle('Velocity error (Controller)');
        sgt.FontSize = 20;

        subplot(1,2,1)
        plot(x_dot_tilda(1,:),'LineWidth',2);
        grid on
        title('$$\dot{\tilde{x}}_{1}$$(First joint)','Interpreter','latex', 'FontSize',15);
        xlabel('t');
        subplot(1,2,2)
        plot(x_dot_tilda(2,:),'LineWidth',2);
        grid on
        title('$$\dot{\tilde{x}}_{2}$$(Second Joint)','Interpreter','latex', 'FontSize',15);
        xlabel('t');

        figure(24);
        sgt = sgtitle('Input (Torque) inizialization (Controller)');
        sgt.FontSize = 15;

        subplot(2,2,1)
        plot(uu_init(1,:,1),'LineWidth',2,'Color','g');
        grid on
        ylabel('u (Nm)');
        xlabel('t');
        title('u1 init');
        subplot(2,2,2)
        plot(uu_init(2,:,1),'LineWidth',2,'Color','g');
        grid on
        ylabel('u (Nm)');
        xlabel('t');
        title('u2 init');

        subplot(2,2,3)
        plot(uu_ref(1,:,1),'LineWidth',2);
        grid on
        title('u1 ref');
        ylabel('u (Nm)');
        xlabel('t');
        subplot(2,2,4)
        plot(uu_ref(2,:,1),'LineWidth',2);
        grid on
        title('u2 ref');
        ylabel('u (Nm)');
        xlabel('t');
 end

 if plot_flag_tuning==1
    
    figure(201) 
    sgt = sgtitle('Tuning (Controller)');
    sgt.FontSize = 20;

     subplot(2,2,1) %theta1
    plot(rad2deg((xx_ref(1,:,1))),'LineWidth',2,'LineStyle','--');
    hold on;
    plot(rad2deg((xx_init(1,:,1))),'LineWidth',2,'Color','r');
    ylabel('\theta [deg]');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('\theta_1');
    legend({'Ref','Init'});
    legend('Location','best'); 
    
    subplot(2,2,2) %theta2
    plot(rad2deg((xx_ref(3,:,1))),'LineWidth',2,'LineStyle','--');
    hold on;
    plot(rad2deg((xx_init(3,:,1))),'LineWidth',2,'Color','r');
    ylabel('\theta [deg]');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('\theta_2 ');
    legend({'Ref','Init'});
    legend('Location','best'); 


    subplot(2,2,3) %u1
    plot(uu_ref(1,:,1),'LineWidth',2,'LineStyle','--');
    hold on;
    plot(uu_init(1,:,1),'LineWidth',2,'Color','g');
    ylabel('u [Nm]');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('u1');
    legend({'Ref','Init'});
    legend('Location','best'); 
    
    subplot(2,2,4) %u2
    plot(uu_ref(2,:,1),'LineWidth',2,'LineStyle','--');
    hold on;
    plot(uu_init(2,:,1),'LineWidth',2,'Color','g');
    ylabel('u [Nm]');
    xlabel('t');
    grid on;
    zoom on;
    hold off;
    title('u2');
    legend({'Ref','Init'});
    legend('Location','best');
 end
end