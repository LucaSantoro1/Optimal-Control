%% Stage Cost

% Group 27: Luca Santoro, Armando Spennato

function [lt, Dlx, Dlu, Dlxx, Dluu, Dlux] = Stage_Cost(xx_t, uu_t, xx_des, uu_des, params)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Argument
%       - xx_t --> State xx at time t
%       - uu_t --> Input uu at time t
%       - xx_des --> Desired state signal at time t 
%       - uu_des --> Desired input signal at time t 
%       - params --> Parameters of the system
%
%   Return
%       - lt --> Current cost 
%       - ∇x_lt(xx,uu) --> Gradient of cost wrt x, at (xx,uu)
%       - ∇u_lt(xx,uu) --> Gradient of cost wrt u, at (xx,uu)
%       - ∇^2lt(xx,uu) --> Hessian of cost wrt xx, uu, ux
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{ 
  Through this matlab function we define the Stage Cost function to understand how far
  we are from the desired behaviour of our system.
  As in the Dynamics.m function case this function allows to compute the Gradient and the Hessian 
  of the Stage Cost. All these things are useful in the implementation of our DDP algorithm.
  (little a and little b parameters that we need to compute the algorithm).
%}

 QQ = params.cost.QQ; % Weight State matrix, Pos. Semi-definite matrix ∈ R^(nn x nn)
 RR = params.cost.RR; % Weight Input matrix, Pos. definite matric ∈ R^(nu x nu)
    
 [nn, ~] = size(xx_des);
 [nu, ~] = size(uu_des);
    
% Optimal Control task is trajectory generation or trajectory exploration.
% Taking into account a possible trajectory (xx_des,uu_des) we want to find a real trajectory close to the possible one. 

% (xx_des,uu_des) represent a desired behavior of the system, in
% general they do NOT satisfy our Dynamics.

% Now we define the Stage Cost:
%{ 
      The Stage Cost is simply defined as a weighted square norm of the
       errors on the state and input errors. 
%}

 lt = (1/2)*(xx_t-xx_des)'*QQ*(xx_t-xx_des) + (1/2)*(uu_t-uu_des)'*RR*(uu_t-uu_des);

 Dlx = QQ*xx_t - QQ*xx_des; % Gradient of the cost ∇x_lt(xx,uu) = Qx
 Dlu = RR*uu_t - RR*uu_des; % Gradient of the cost ∇u_lt(xx,uu) = Ru
 
 Dlxx = QQ;
 Dluu = RR;
 Dlux = zeros(nu,nn);
    
end