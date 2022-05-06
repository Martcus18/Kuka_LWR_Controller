clc;
clear;
close all;

%% Init

load('new_workspace.mat');

[dim, ~] = size(time);
dim = dim-1;
tplot = time(1:dim,1);

%% Position and velocity

q1 = joints(1:dim,1); q2 = joints(1:dim,2);
q3 = joints(1:dim,3); q4 = joints(1:dim,4);
q5 = joints(1:dim,5); q6 = joints(1:dim,6);
q7 = joints(1:dim,7);

figure(1);
plot(tplot,q1,tplot,q2,tplot,q3,tplot,q4,tplot,q5,tplot,q6,tplot,q7);
hold on; grid on;
legend('1','2','3','4','5','6','7');
title('Joint position')

q1_dot = joints(1:dim,8); q2_dot = joints(1:dim,9);
q3_dot = joints(1:dim,10); q4_dot = joints(1:dim,11);
q5_dot = joints(1:dim,12); q6_dot = joints(1:dim,13);
q7_dot = joints(1:dim,14);

figure(2);
plot(tplot,q1_dot,tplot,q2_dot,tplot,q3_dot,tplot,q4_dot,tplot,q5_dot,tplot,q6_dot,tplot,q7_dot);
hold on; grid on;
legend('1','2','3','4','5','6','7');
title('Angular velocity')

%% Estimated velocities

q1_dot_hat = vel_hat(1:dim,1); q2_dot_hat = vel_hat(1:dim,2);
q3_dot_hat = vel_hat(1:dim,3); q4_dot_hat = vel_hat(1:dim,4);
q5_dot_hat = vel_hat(1:dim,5); q6_dot_hat = vel_hat(1:dim,6);
q7_dot_hat = vel_hat(1:dim,7);

figure(3)
subplot(241)
plot(tplot,q1_dot_hat,tplot,q1_dot)
hold on; grid on;
title('1-st joint');

subplot(242)
plot(tplot,q2_dot_hat,tplot,q2_dot)
hold on; grid on;
title('2-nd joint');

subplot(243)
plot(tplot,q3_dot_hat,tplot,q3_dot)
hold on; grid on;
title('3-rd joint');

subplot(244)
plot(tplot,q4_dot_hat,tplot,q4_dot)
hold on; grid on;
title('4-th joint');

subplot(245)
plot(tplot,q5_dot_hat,tplot,q5_dot)
hold on; grid on;
title('5-th joint');

subplot(246)
plot(tplot,q6_dot_hat,tplot,q6_dot)
hold on; grid on;
title('6-th joint');

subplot(247)
plot(tplot,q7_dot_hat,tplot,q7_dot)
hold on; grid on;
title('7-th joint');

sgtitle('Estimated angular velocities')


%% Residual plots

figure(4)
subplot(241)
plot(tplot,r(1,1:dim))
hold on; grid on;
title('1-st residual');

subplot(242)
plot(tplot,r(2,1:dim))
hold on; grid on;
title('2-nd residual');

subplot(243)
plot(tplot,r(3,1:dim))
hold on; grid on;
title('3-rd residual');

subplot(244)
plot(tplot,r(4,1:dim))
hold on; grid on;
title('4-th residual');

subplot(245)
plot(tplot,r(5,1:dim))
hold on; grid on;
title('5-th residual');

subplot(246)
plot(tplot,r(6,1:dim))
hold on; grid on;
title('6-th residual');

subplot(247)
plot(tplot,r(7,1:dim))
hold on; grid on;
title('7-th residual');

sgtitle('Residual');

%% Nominal torque plot

figure(5)
subplot(241)
plot(tplot,torque_fl(:,1))
hold on; grid on;
title('1-st torque');

subplot(242)
plot(tplot,torque_fl(:,2))
hold on; grid on;
title('2-nd torque');

subplot(243)
plot(tplot,torque_fl(:,3))
hold on; grid on;
title('3-rd torque');

subplot(244)
plot(tplot,torque_fl(:,4))
hold on; grid on;
title('4-th torque');

subplot(245)
plot(tplot,torque_fl(:,5))
hold on; grid on;
title('5-th torque');

subplot(246)
plot(tplot,torque_fl(:,6))
hold on; grid on;
title('6-th torque');

subplot(247)
plot(tplot,torque_fl(:,7))
hold on; grid on;
title('7-th torque');

sgtitle('Nominal torque');

%% Torque faulty plot: 'real' joint velocities

figure(6)
subplot(241)
plot(tplot,torque_faulty(:,1),tplot,r(1,1:dim))
hold on; grid on;
title('1-st');

subplot(242)
plot(tplot,torque_faulty(:,2),tplot,r(2,1:dim))
hold on; grid on;
title('2-nd');

subplot(243)
plot(tplot,torque_faulty(:,3),tplot,r(3,1:dim))
hold on; grid on;
title('3-rd');

subplot(244)
plot(tplot,torque_faulty(:,4),tplot,r(4,1:dim))
hold on; grid on;
title('4-th');

subplot(245)
plot(tplot,torque_faulty(:,5),tplot,r(5,1:dim))
hold on; grid on;
title('5-th');

subplot(246)
plot(tplot,torque_faulty(:,6),tplot,r(6,1:dim))
hold on; grid on;
title('6-th');

subplot(247)
plot(tplot,torque_faulty(:,7),tplot,r(7,1:dim))
hold on; grid on;
title('7-th');

sgtitle('Residual vs TauExt: actual joint velocities');

%% Torque faulty plot: estimated joint velocities

figure(7)
subplot(241)
plot(tplot,torque_faulty(:,1),tplot,r_ob(1,1:dim))
hold on; grid on;
title('1-st');

subplot(242)
plot(tplot,torque_faulty(:,2),tplot,r_ob(2,1:dim))
hold on; grid on;
title('2-nd');

subplot(243)
plot(tplot,torque_faulty(:,3),tplot,r_ob(3,1:dim))
hold on; grid on;
title('3-rd');

subplot(244)
plot(tplot,torque_faulty(:,4),tplot,r_ob(4,1:dim))
hold on; grid on;
title('4-th');

subplot(245)
plot(tplot,torque_faulty(:,5),tplot,r_ob(5,1:dim))
hold on; grid on;
title('5-th');

subplot(246)
plot(tplot,torque_faulty(:,6),tplot,r_ob(6,1:dim))
hold on; grid on;
title('6-th');

subplot(247)
plot(tplot,torque_faulty(:,7),tplot,r_ob(7,1:dim))
hold on; grid on;
title('7-th');

sgtitle('Residual vs TauExt: estimated joint velocities');

%% Saving figures

% saveas(f6,'comp.png','png');
% saveas(f7,'comp_hat.png','png');
% saveas(f3,'vel_hat.png','png');
% saveas(f5,'nominal_torque.png','png');
% close all
