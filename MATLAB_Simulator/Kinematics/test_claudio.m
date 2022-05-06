%% TESTING CLAUDIO'S JACOBIANS

J_cla = load("Jacobian_LWR.mat");
J_cla = J_cla.J;

dJdt = load("dJacobian_LWR.mat");
dJdt = dJdt.dJdt;

T = load("T_matrix.mat");
T = T.T;

p2 = load("p2.mat");
p3 = load("p3.mat");
p4 = load("p4.mat");
p5 = load("p5.mat");
p6 = load("p6.mat");

p2 = p2.p2;
p3 = p3.p3;
p4 = p4.p4;
p5 = p5.p5;
p6 = p6.p6;


syms q1 q2 q3 q4 q5 q5 q6 q7 dq1 dq2 dq3 dq4 dq5 dq6 dq7 real
syms d1 d2 d3 d0
syms Q1 Q2 Q3 Q4 Q5 Q6 Q7

% q0 = zeros(1,7);
% J = kuka.geometricJacobian(q0(1:7),'kuka_lwr_7_link');
% J = J(4:6,:);

d1  = 0.4;
d2 = 0.39;
d3 = 0.078;
d0 = 0.310;
% syms



% q0 = [-1.1,pi/4,0,1.3*pi,-1,0,0,0,0,0,0,0,0,0];
% q1 = q0(1);
% q2 = q0(2);
% q3 = q0(3);
% q4 = q0(4);
% q5 = q0(5);
% q6 = q0(6);
% q7 = q0(7);
% q1 = 0;
% q2 = 0;
% q3 = 0;
% q4 = 0;
% q5 = 0;
% q6 = 0;
% q7 = 0;
% t = 0
% J_prova = eval(J_cla);
% d1_rob = -0.868;
% d0  = 0.131104569076095 - 2^(1/2)/5 + 4163471060732049/9007199254740992
