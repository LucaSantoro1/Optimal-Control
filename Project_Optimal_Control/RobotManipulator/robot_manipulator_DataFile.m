% Simscape(TM) Multibody(TM) version: 7.4

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 0.22004150764194225 -0.20000000000000001];  % m
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = 'B[2:-:3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-0.50000000000000044 -5.5511151231257827e-17 -0.20000000000000001];  % m
smiData.RigidTransform(2).angle = 0;  % rad
smiData.RigidTransform(2).axis = [0 0 0];
smiData.RigidTransform(2).ID = 'F[2:-:3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0.5 -1.3607186657192824e-18 -0.14000000000000004];  % m
smiData.RigidTransform(3).angle = 0;  % rad
smiData.RigidTransform(3).axis = [0 0 0];
smiData.RigidTransform(3).ID = 'B[3:-:4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-0.50000000000000067 4.4408920985006262e-16 -0.14000000000000004];  % m
smiData.RigidTransform(4).angle = 0;  % rad
smiData.RigidTransform(4).axis = [0 0 0];
smiData.RigidTransform(4).ID = 'F[3:-:4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 0 0];  % m
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = 'SixDofRigidTransform[2]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(3).mass = 0.0;
smiData.Solid(3).CoM = [0.0 0.0 0.0];
smiData.Solid(3).MoI = [0.0 0.0 0.0];
smiData.Solid(3).PoI = [0.0 0.0 0.0];
smiData.Solid(3).color = [0.0 0.0 0.0];
smiData.Solid(3).opacity = 0.0;
smiData.Solid(3).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 274076.0121443696;  % g
smiData.Solid(1).CoM = [0 1.6224284092896593 0];  % cm
smiData.Solid(1).MoI = [135545722.36473364 239732950.66450399 132742502.62889297];  % g*cm^2
smiData.Solid(1).PoI = [0 0 0];  % g*cm^2
smiData.Solid(1).color = [0.82352899999999996 0.70588200000000001 0.43137300000000001];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'BASE_prt';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 39721.152899197194;  % g
smiData.Solid(2).CoM = [-6.7040349212222567 0 0];  % cm
smiData.Solid(2).MoI = [2323335.4619654869 59829600.659022972 58855852.515172631];  % g*cm^2
smiData.Solid(2).PoI = [0 0 0];  % g*cm^2
smiData.Solid(2).color = [0.058824000000000001 0.37254900000000002 0.68627499999999997];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'PARTE_BRACCIO_INFERIORE_prt';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 41559.245625730546;  % g
smiData.Solid(3).CoM = [-2.475591340927763 0 0];  % cm
smiData.Solid(3).MoI = [1705916.3629063768 68423151.421040744 68125806.840583995];  % g*cm^2
smiData.Solid(3).PoI = [0 0 0];  % g*cm^2
smiData.Solid(3).color = [0.70588200000000001 0.098039000000000001 0.098039000000000001];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'PARTE_BRACCIO_SUPERIORE_prt';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(2).Rz.Pos = 0.0;
smiData.RevoluteJoint(2).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 40.241553841329704;  % deg
smiData.RevoluteJoint(1).ID = '[2:-:3]';

smiData.RevoluteJoint(2).Rz.Pos = 133.29223426247057;  % deg
smiData.RevoluteJoint(2).ID = '[3:-:4]';

