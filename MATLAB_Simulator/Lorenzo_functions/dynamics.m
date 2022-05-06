function dx = dynamics(t,x)
% dx = dynamics(t,x)
% This function defines the dynamics of the 7R Kuka LWR

% the state is a 14-dimensional vector:
% x(1:7)  --> q
% x(8:14) --> dq

global fault Kp Kd q_d

q = x(1:7);
dq = x(8:14);

B = get_Bnum(q);
S = get_Snum(q,dq);
g = get_gnum(q);

u_n = g + [sin(t); sin(2*t); sin(t); 0; 0; 0; 0];

% % first try with just a gravity compensation
u_n = g;
% 
% faulty control
u_f = fault_control(u_n,t,q,fault);

% applied torque
u = u_n+u_f;

% % PD + gravity compensation
% 
% u = Kp*(q-q_d)-Kd*dq+g;

ddq = inv(B)*(u-g-S*dq);

dx = [dq; ddq];

end
