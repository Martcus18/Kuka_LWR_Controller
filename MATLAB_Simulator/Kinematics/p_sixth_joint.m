function p6 = p6(q1,q2,q3,q4)
%P6
%    P6 = P6(Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Oct-2021 18:33:21

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = sin(q1);
t7 = sin(q2);
t8 = sin(q3);
t9 = sin(q4);
p6 = [t9.*(t6.*t8-t2.*t3.*t4).*(-3.9e+1./1.0e+2)-t2.*t7.*(2.0./5.0)-t2.*t5.*t7.*(3.9e+1./1.0e+2);t9.*(t2.*t8+t3.*t4.*t6).*(3.9e+1./1.0e+2)-t6.*t7.*(2.0./5.0)-t5.*t6.*t7.*(3.9e+1./1.0e+2);t3.*(2.0./5.0)+t3.*t5.*(3.9e+1./1.0e+2)+t4.*t7.*t9.*(3.9e+1./1.0e+2)+3.1e+1./1.0e+2];
