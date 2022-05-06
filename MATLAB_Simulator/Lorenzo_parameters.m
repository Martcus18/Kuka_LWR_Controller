clear all;
clc;

import lwr;
format long;

%Building with DH directly the robot
% kuka = lwr("inertia.txt","masses.txt").robot;
% controller = lwr("inertia_modified.txt","masses_modified.txt").robot;


%% CHECK THIS COMMAND interactiveRigidBodyTree
%Using the URDF
kuka = importrobot('./models/kuka_lwr.urdf');

addVisual(kuka.Base,"Mesh","./visual/base.STL")
addVisual(kuka.Bodies{1},"Mesh","./visual/link_1.STL")
addVisual(kuka.Bodies{2},"Mesh","./visual/link_2.STL")
addVisual(kuka.Bodies{3},"Mesh","./visual/link_3.STL")
addVisual(kuka.Bodies{4},"Mesh","./visual/link_4.STL")
addVisual(kuka.Bodies{5},"Mesh","./visual/link_5.STL")
addVisual(kuka.Bodies{6},"Mesh","./visual/link_6.STL")
addVisual(kuka.Bodies{7},"Mesh","./visual/link_7.STL")

% controller = importrobot('./models/kuka_lwr_controller.urdf');
controller = importrobot('./models/kuka_lwr_controller_circle.urdf');

kuka.Gravity = [0,0,-9.81];
controller.Gravity = [0,0,-9.81];

kuka.DataFormat = 'row';
controller.DataFormat = 'row';

%% Fault type

fault = 4;

%%
addpath('./Kinematics/'); 
% functions defined by Claudio Gaz
addpath('./Dynamics/'); 
% function for fault "generation"
addpath('./Lorenzo_functions/');


t0=0.0;
tf=10.0;

DeltaT = 0.01;
Ts = DeltaT;

% Friction parameters
friction_magnitude = 0.1;
A_friction =0.5;
B = 0.5;
v_str = 0.01;

% Array initialization
joints = [];
time = zeros(1);
diff_plot = [];
accelerations = [];
variances = [];
predictions = [];
friction_plot = [];
accs_ref = [];
torque_fl = [];
corrective_torques = [];
task_vec = [];
gains = [0];
singular_values = [];

% Observer variables
k0 = 10;
%k0 = 0.004;
vel_hat = [];
x2_hat = [];
z = [];

% Residual variables
gain = 50*eye(7);
sum1 = zeros(7,1);
sum2 = zeros(7,1);
r = [];
torque_faulty = [];

sum1_ob = zeros(7,1);
sum2_ob = zeros(7,1);
r_ob = [];

% Joint trajectory
Tsample = linspace(0,tf + 50*Ts,(tf + 50*Ts - t0)/Ts)';
Xd = 0.6*[repmat(cos(Tsample),[1,6]),zeros(length(Tsample),1),repmat(-sin(Tsample),[1,6]),zeros(length(Tsample),1)];

% Initialization of initial state and array of joints positions-velocities

options_ode = odeset('RelTol',1e-10,'AbsTol',1e-12);
options_opt = optimoptions('fmincon','Algorithm','sqp','Display', 'off');
index = 1;

A = [1;-1];
Aproj = blkdiag(A,A,A,A,A,A,A);

% ROBUST_MAX REALIZED WITH 10

% bproj = ones(14,1) * 20;


%% PREVIOUS BOUND
bproj = ones(14,1) * 10;

% bproj = ones(14,1) * 100;

u0proj = zeros(7,1);

% USING BUILDING WITH URDF
%% INITIAL CONFIGURATION

% robot
q0 = [-1.1,pi/4,0,1.3*pi,-1,0,0,0,0,0,0,0,0,0];
accs = zeros(1,7);

% reduced observer
x2_hat = k0*q0(1:7);
z = zeros(1,7);

% initial generalized momentum for residual with estimated joint velocites
p0 = get_Bnum(q0(1:7))*x2_hat';

%% GAINS
Kp = [100,0,0;0,100,0;0,0,400]*1;
Kd = [10,0,0;0,10,0;0,0,50]*1;
Kv = 10;

J = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));

p_0 = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));

dp_0 = J * q0(8:14)';

% INITIAL ACC = 0

d2p_0 = J * accs(end,:)';

% damping = 0.01;
damping = 0.1;

joints = q0;

TauFL = gravityTorque(controller,q0(1:7));

H = [];

frequency = 1.0;