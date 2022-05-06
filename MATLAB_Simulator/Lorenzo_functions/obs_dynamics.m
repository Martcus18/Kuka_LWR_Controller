function ob = obs_dynamics(t,z)
% ob = obs_dynamics(t,z)
% Defines the dynamics of a reduced observer for estimating the velocities
% of the 7R Kuka LWR robot

% z(1:7)   --> q
% z(8:14)  --> q_dot
% z(15:21) --> z_dot

global k0 fault

% Dynamics of the robot from which we can measure the joint angular position
q = z(1:7);
q_dot = z(8:14);

% Dynamics of the reduced observer
q_dot_hat = z(15:21)+k0*q;

B = get_Bnum(q);
S = get_Snum(q,q_dot);
S_hat = get_Snum(q,q_dot_hat);
g = get_gnum(q);
%u_n = g;

u_n = g + [sin(t); sin(2*t); sin(t); 0; 0; 0; 0];

% faulty control
u_f = fault_control(u_n,t,q,fault);

% applied torque
u = u_n+u_f;

dx = inv(B)*(-S*q_dot-g+u);

dz = inv(B)*(-S_hat*q_dot_hat-g-B*k0*eye(7)*q_dot_hat+u);

ob = [q_dot;dx;dz];
end

