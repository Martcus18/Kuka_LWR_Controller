clc
clear
close all

q = load("Q.txt");
q_hat = load("Q_hat.txt");

dq = load("dQ.txt");
dq_hat = load("dQ_hat.txt");

r = load("res.txt");
r_ob = load("res_ob.txt");

torque_fl = load("torque_ref.txt");
torque_faulty = load("torque_faulty.txt");

%% Joint position

figure(1)
hold on; grid on;
for i=1:7
    hold on;
    plot(q(:,i));
end
title('joint position');
legend('1','2','3','4','5','6','7');

%% joint speed

figure(2)
hold on; grid on;
for i=1:7
    hold on;
    plot(dq(:,i));
end
title('joint speed');
legend('1','2','3','4','5','6','7');

%% estimated joint speed

figure(3)
hold on; grid on;
for i=1:7
    plot(dq_hat(:,i));
end
title('estimated joint speed');
legend('1','2','3','4','5','6','7');

%% Comparison velocities

figure(4)
subplot(241)
plot(dq(:,1),'b',dq_hat(:,1),'r')
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat','location','southeast');
title('1-st joint');

subplot(242)
plot(dq(:,2),'b',dq_hat(:,2),'r');
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat');
title('2-nd joint');

subplot(243)
plot(dq(:,3),'b',dq_hat(:,3),'r');
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat');
title('3-rd joint');

subplot(244)
plot(dq(:,4),'b',dq_hat(:,4),'r')
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat');
title('4-th joint');

subplot(245)
plot(dq(:,5),'b',dq_hat(:,5),'r')
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat','location','southeast');
title('5-th joint');

subplot(246)
plot(dq(:,6),'b',dq_hat(:,6),'r')
hold on; grid on;
%ylim([-0.15,0.15]);
%legend('dq','dq\_hat','location','southeast');
title('6-th joint');

subplot(247)
plot(dq(:,7),'b',dq_hat(:,7),'r')
hold on; grid on;
%%ylim([-0.15,0.15]);
%legend('dq','dq\_hat','location','northeast');
title('7-th joint');

S  = axes( 'visible', 'off', 'title', 'Comparison between velocities' );

%% Comparison position

figure(5)
subplot(241)
plot(q(:,1),'b',q_hat(:,1),'r')
hold on; grid on;
%legend('q','q\_hat','location','southeast');
title('1-st joint');

subplot(242)
plot(q(:,2),'b',q_hat(:,2),'r');
hold on; grid on;
%legend('q','q\_hat');
title('2-nd joint');

subplot(243)
plot(q(:,3),'b',q_hat(:,3),'r');
hold on; grid on;
%legend('q','q\_hat');
title('3-rd joint');

subplot(244)
plot(q(:,4),'b',q_hat(:,4),'r')
hold on; grid on;
%legend('q','q\_hat');
title('4-th joint');

subplot(245)
plot(q(:,5),'b',q_hat(:,5),'r')
hold on; grid on;
%legend('q','q\_hat','location','southeast');
title('5-th joint');

subplot(246)
plot(q(:,6),'b',q_hat(:,6),'r')
hold on; grid on;
%legend('q','q\_hat','location','southeast');
title('6-th joint');

subplot(247)
plot(q(:,7),'b',q_hat(:,7),'r')
hold on; grid on;
%legend('q','q\_hat','location','northeast');
title('7-th joint');

S  = axes( 'visible', 'off', 'title', 'Comparison between positions' );

%% Residual plots

figure(6)
subplot(241)
plot(r(:,1),'b',torque_faulty(:,1),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,1))) max(abs(torque_fl(:,1)))]);
title('1-st residual');

subplot(242)
plot(r(:,2),'b',torque_faulty(:,2),'r');
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,2))) max(abs(torque_fl(:,2)))]);
title('2-nd residual');

subplot(243)
plot(r(:,3),'b',torque_faulty(:,3),'r');
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,3))) max(abs(torque_fl(:,3)))]);
title('3-rd residual');

subplot(244)
plot(r(:,4),'b',torque_faulty(:,4),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,4))) max(abs(torque_fl(:,4)))]);
title('4-th residual');

subplot(245)
plot(r(:,5),'b',torque_faulty(:,5),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,5))) max(abs(torque_fl(:,5)))]);
title('5-th residual');

subplot(246)
plot(r(:,6),'b',torque_faulty(:,6),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,6))) max(abs(torque_fl(:,6)))]);
title('6-th residual');

subplot(247)
plot(r(:,7),'b',torque_faulty(:,7),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,7))) max(abs(torque_fl(:,7)))]);
title('7-th residual');
 
S  = axes( 'visible', 'off', 'title', 'Residual with actual joint velocities' );

figure(7)
subplot(241)
plot(r_ob(:,1),'b',torque_faulty(:,1),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,1))) max(abs(torque_fl(:,1)))]);
title('1-st residual');

subplot(242)
plot(r_ob(:,2),'b',torque_faulty(:,2),'r');
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,2))) max(abs(torque_fl(:,2)))]);
title('2-nd residual');

subplot(243)
plot(r_ob(:,3),'b',torque_faulty(:,3),'r');
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,3))) max(abs(torque_fl(:,3)))]);
title('3-rd residual');

subplot(244)
plot(r_ob(:,4),'b',torque_faulty(:,4),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,4))) max(abs(torque_fl(:,4)))]);
title('4-th residual');

subplot(245)
plot(r_ob(:,5),'b',torque_faulty(:,5),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,5))) max(abs(torque_fl(:,5)))]);
title('5-th residual');

subplot(246)
plot(r_ob(:,6),'b',torque_faulty(:,6),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,6))) max(abs(torque_fl(:,6)))]);
title('6-th residual');

subplot(247)
plot(r_ob(:,7),'b',torque_faulty(:,7),'r')
hold on; grid on;
%%ylim([-max(abs(torque_fl(:,7))) max(abs(torque_fl(:,7)))]);
title('7-th residual');

S  = axes( 'visible', 'off', 'title', 'Residual with estimated joint velocities' );

