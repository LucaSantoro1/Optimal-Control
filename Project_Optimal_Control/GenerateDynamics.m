%% Generate dynamics 

% Group 27: Luca Santoro, Armando Spennato

% File generatedynamics.m description:
%{ 
The following file allows to create the Matlab function Dynamics.m. 
We build a function with its Gradient and Hessian.

In particular is used  the symbolic math toolbox,  that allows to build  
all the computations considering x, u, and p as symbolic elements.

The final function Dynamis.m is generated through matlabFunction(.,..,{}). 
The Dynamics.m, will implement all the computations, no in a symbolic way.

It is able to find all the needed quantities in a numerical way, 
substituting the input values to their symbolic counterparts.

%}

% Function Dynamics.m description:
%{
[xxp,dfx,dfu,pfxx,pfuu,pfux] = dynamics(xx,uu,pp)

This is the function that we build through the generate dynamics.m file, it
takes as input:
   - xx --> State at time t
   - uu --> Input at time t
   - pp --> Costate (In order to perform the tensor product)

and return:
   - xxp --> Next state xx_{t+1}
   - dfx --> Gradient of f wrt x, at xx,uu
   - dfu --> Gradient of f wrt u, at xx,uu
   - pfxx,pfuu,pfux --> Tensor product..
(it is more convenient to directly calculate the tensor product (for DDP)
 in this function)

%}


% We create arrays of symbolic scalar variables.

nn = 4; % Length of the State vector x(t)=[x1(t) x2(t) x3(t) x4(t)]'.
nu = 2; % Length of the Input vector u(t)=[u1(t) u2(t)]'.

syms xx [nn,1] real
syms uu [nu,1] real

syms pp [nn,1] real

% Parameters

gg = 9.81;  % Gravitational acceleration [m/s^2].

mm1 = 2;    % Mass link1 [Kg].
mm2 = 2;    % Mass link2 [Kg].

JJ1 = 0.5;  % JJi with i=1,2 is the i-th link inertia,
JJ2 = 0.5;  % about an axis through the CoM and parallel to Z [Kg*m^2].

ll1 = 1;    % Length link1 [m]. 
ll2 = 1;    % Length link2 [m]. 

rr1 = 0.5;  % rri with i=1,2 is the distance between, 
rr2 = 0.5;  % joint i and the CoM of the i-th link [m].

dt = 1e-3;  % Discretization time step of the Forward-Euler Method.

% Countinuous Dynamics:
%{  
Dynamic model (continuous-time dynamics): M(q)q'' + C(q,q')q' + g(q) = τ
   
  M(q):= [ M11  M12 ]     C(q,q'):= [ C11  C12 ]     g(q):= [ G1 ] 
         [ M21  M22 ]               [ C21  C22 ]            [ G2 ]
                  
 q = [θ1 θ2]' ∈ R^2 (Joint variables vector).
 τ = [τ1 τ2]' ∈ R^2 (Control torque).

where: 

M11 = mm1*(rr1)^2 + mm2*(ll1^2 + rr2^2 + 2*ll1*rr2*cos(θ2)) + JJ1 + JJ2
M12 = mm2*(rr2^2 + ll1*rr2*cos(θ2)) + JJ2
M21 = mm2*(rr2^2 + ll1*rr2*cos(θ2)) + JJ2
M22 = mm2*rr2^2 + JJ2

C11 = -mm2*ll1*rr2*sen(θ2)*(θ2')
C12 = -mm2*ll1*rr2*sen(θ2)*(θ1'+θ2')
C21 = -mm2*ll1*rr2*sen(θ2)*(-θ1')
C22 = 0

G1 = (mm1*rr1 + mm2*ll1)*g*cos(θ1) + mm2*g*rr2*cos(θ1 + θ2)
G2 = mm2*g*rr2*cos(θ1 + θ2)

%}


% Discrete Dynamics:
%{ 
In order to write the discrete dynamics we need to use the state-space 
representation, so we have to consider:  
State vector:
 x(t):= [x1(t) x2(t) x3(t) x4(t)]'=[θ1(t) θ1(t)' θ2(t) θ2(t)']' ∈ R^4 
Input vector:
 u(t):=[u1(t) u2(t)]'=[τ1(t) τ2(t)]' ∈ R^2

In this way we can rewrite the joint variables as follow:

Joint position vector --> q:= [θ1 θ2]' = [x1(t) x3(t)]'
Joint velocity vector --> q':= [θ1' θ2']' = [x2(t) x4(t)]'
Joint acceleration vector --> q'':= [θ1'' θ2'']'= [x2'(t) x4'(t)]'

Dynamic model in the state space:
 x1'(t) = x2(t)
 x3'(t) = x4(t)
[x2'(t)]                           [u1(t)]                     [x2(t)]
[      ] = (M(x1(t),x3(t))^(-1))*( [     ] - C(x1(t),..,x4(t)) [     ] - g(x1(t),x3(t)))
[x4'(t)]                           [u2(t)]                     [x4(t)]

To obtain the discrete time model of dynamics we use the Forward Euler
Method:
                      x_{t+1} = x_{t} + dt * x'(t)

Where dt is the discretization time unit.
We get the following discrete time system:

x1_{t+1} = x1_{t} + dt * x1'(t)
x2_{t+1} = x2_{t} + dt * x2'(t)
x3_{t+1} = x3_{t} + dt * x3'(t)
x4_{t+1} = x4_{t} + dt * x4'(t)

%}



uu=[uu(1); uu(2)]; % Input torques vector τ = [τ1 τ2]' ∈ R^2 (Control torque).

ss1 = sin(xx(1));         % sen(θ1).
ss2 = sin(xx(3));         % sen(θ2).
ss12 = sin(xx(1)+ xx(3)); % sen(θ1 + θ2).

cc1 = cos(xx(1));         % cos(θ1).
cc2 = cos(xx(3));         % cos(θ2).
cc12 = cos(xx(1)+ xx(3)); % cos(θ1 + θ2).


%%%%% Elements of the Inertia Matrix M(x1(t),x3(t)) %%%%%%%%%

M11 = mm1*rr1^2+mm2*(ll1^2+rr2^2+2*ll1*rr2*cc2)+JJ1+JJ2;
M12 = mm2*(rr2^2+ll1*rr2*cc2)+JJ2;
M22 = rr2^2*mm2+JJ2;

MM = [M11, M12; M12, M22];

%%%%% Coriolis and centrifugal elements of Matrix C(x1(t),..,x4(t)) %%%%% 

C11 = -mm2*ll1*rr2*ss2*(xx(4)); C12 = -mm2*ll1*rr2*ss2*(xx(2) + xx(4));
C21 = mm2*ll1*rr2*ss2*(xx(2)); C22 = 0;

CC = [C11 C12; C21 C22];

%%%%% Gravity vector g(x1(t),x3(t)) %%%%%%%%%%

gg1 = (mm1*rr1+mm2*ll1)*gg*(cc1) + mm2*gg*rr2*cc12;
gg2 = mm2*gg*rr2*cc12;

GG = [gg1; gg2];
%%%%%%%%%%%%%%%%%

qq = [xx(1); xx(3)];        % Joint position vector --> q:= [θ1 θ2]' = [x1(t) x3(t)]'.

qq_d = [xx(2); xx(4)];      % Joint velocity vector --> q':= [θ1' θ2']' = [x2(t) x4(t)]'.

qq_2d = MM\(uu-CC*qq_d-GG); % Joint acceleration vector --> q'':= [θ1'' θ2'']'= [x2'(t) x4'(t)]'.

% At this point we can write the Dynamic model in the state space rapresentation:

xx1_d = xx(2); 
xx2_d = qq_2d(1);
xx3_d = xx(4);
xx4_d = qq_2d(2);

% And we find the discretized model using Forward Euler method:

f1 = xx(1) + dt * xx1_d;
f2 = xx(2) + dt * xx2_d;
f3 = xx(3) + dt * xx3_d;
f4 = xx(4) + dt * xx4_d;

xx_next = [f1; f2; f3; f4]; % Next state xx_{t+1}.

% Generate the Gradient ∇f(x,u)
%{
                          [∇xf(x,u)]
                ∇f(x,u) = [        ]
                          [∇uf(x,u)]
where:
  ∇xf(x,u) --> Gradient of f(x,u) wrt x (∈ R^nxn)
  ∇uf(x,u) --> Gradient of f(x,u) wrt u (∈ R^mxn)
(The calculation of the Gradient is useful in case of linearization.)
%}

dfx = jacobian(xx_next,xx)'; % Gradient of f wrt x.
dfu = jacobian(xx_next,uu)'; % Gradient of f wrt u.

% dfx and dfu would be the Linearization (dfx=A' and dfu=B'). 
% Whenever we need matrices A and B we can call the dynamic.m function. 

% Generate the Hessian matrix ∇^2f(x,u)

d2fxx = cell(1, nn); % ∇^2_xxf(x,u)
d2fuu = cell(1, nn); % ∇^2_uuf(x,u)
d2fux = cell(1, nn); % ∇^2_xuf(x,u)

% In particular we are interested in performing the tensor products:
% ∇^2_xxf(x,u) * p, ∇^2_uuf(x,u) * p, ∇^2_xuf(x,u) * p
pfxx = zeros(nn,nn);
pfuu = zeros(nu,nu);
pfux = zeros(nu,nn)';

for ii = 1:nn
    d2fxx{ii} = jacobian(dfx(:,ii), xx);
    pfxx = pfxx + (d2fxx{ii}*pp(ii));
    %
    d2fuu{ii} = jacobian(dfu(:,ii), uu);
    pfuu = pfuu + (d2fuu{ii}*pp(ii));
    %
    d2fux{ii} = jacobian(dfx(:,ii), uu);
    pfux = pfux + (d2fux{ii}*pp(ii));
end
pfux=pfux';

matlabFunction(xx_next, dfx, dfu, pfxx, pfuu, pfux, 'File', 'Dynamics','Vars',{xx,uu,pp});

% In this way we obtain the Dynamic.m function that implements all the
% previous computations no in symbolic way, this means that the function Dynamics.m is
% able to find all the quantities in numerical way.