
In the following project particular attention is given to the design and implementation of an optimal control law for a 2-DOF robot. The project
consists of four main steps:

• Task 0 - Problem Setup :
Discretization of the manipulator dynamics and writing the Dynamics.m function;

• Task 1 - Trajectory Exploration :
Design of an optimal trajectory to move from one equilibrium configuration to another, using the DDP algorithm;

• Task 2 - Trajectory Optimization :
The same robotic manipulator is used to draw a custom curve to be
followed by the end effector within its workspace. Furthermore, the
corresponding optimal trajectory is defined in the joint space by using
the DDP algorithm;

• Task 3 - Trajectory Tracking :
Linearization of the model around the optimal trajectory obtained in
the previous point and definition of the optimal feedback controller to
perform trajectory tracking, through LQR algorithm;

• Task 4 - Animation:
Designing an animation using Simscape Multibody, which provides a
multibody simulation environment for 3D mechanical systems such as
robots, to visualize the results obtained
