function res = discrete_res(q,dq,t,r,k)
% res = discrete_res(q,dq,t,r,k)
% This functions takes the values of q and dq at time k and the values of 
% the residual up to time k

global gain dt sum1 sum2

B_k = get_Bnum(q);
S_k = get_Snum(q,dq);
g_k = get_gnum(q);

% current torque
tau_k = g_k;
%tau_k = g_k+[sin(t); sin(2*t); sin(t); 0; 0; 0; 0];

% current generalized momentum
p_k = B_k*dq;

% update sum1
sum1 = sum1+(tau_k+S_k'*dq-g_k)*dt;

% update sum2
if k~=1
    sum2 = sum2+r(:,k-1)*dt;
end

res = inv(eye(7)+gain*dt)*gain*(p_k-sum1-sum2);

end

