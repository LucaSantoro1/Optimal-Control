%% Term Cost

% Group 27: Luca Santoro, Armando Spennato

function [lT, Dlx, Dlxx] = Term_Cost(xx_t, xx_des, params)
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Argument:
%       - xx_t --> State xx at time T
%       - xx_des --> Reference signal at time T
%       - params --> Parameters of the system
%
%   Return:
%       - lT --> Current cost 
%       - ∇x_lT(xx_T) --> Gradient of cost wrt x, at xx_T
%       - ∇^2_lT --> Hessian of the term cost
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% With this function we implement the Terminal Cost that  is the piece of 
% the cost that we consider at time T.
%{
 In particular, the term cost is defined as follows:

           lT(xx_t) := (1/2)*|| xx_T - xx_des ||^2 * QQf

Where QQf is the weight matrix, that is a Pos. Semi-definite matrix ∈ R^(nn x nn)

%}

    QQf = params.cost.QQf;

    lT = (1/2)*(xx_t-xx_des)'*QQf*(xx_t-xx_des);

% As the previous functions, also the Term_cost.m function gives as output not only the cost, 
% but also the Gradient and the Hessian, which  will be useful when we will
% implement the DDP algorithm.
   
    Dlx = QQf*xx_t - QQf*xx_des; % Gradient of the cost: ∇x_lT = Qx
    
    Dlxx = QQf;

end