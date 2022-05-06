clear all
close all
clc

% 2021-09-22
% generates J(q) (3x7) and Jdot(q) (3x7)

syms q1 q2 q3 q4 q5 q6 q7
syms d0 d1 d2 d3 real

q = [q1;q2;q3;q4;q5;q6;q7];

A1=DH(0,pi/2,d0,q1);
A2=DH(0,-pi/2,0,q2);
A3=DH(0,-pi/2,d1,q3);
A4=DH(0,pi/2,0,q4);
A5=DH(0,pi/2,d2,q5);
A6=DH(0,-pi/2,0,q6);
A7=DH(0,0,d3,q7);

T = A1*A2*A3*A4*A5*A6*A7;
p = simplify(T(1:3,end));

T6 = A1*A2*A3*A4*A5*A6;
p6 = simplify(T6(1:3,end));

T5 = A1*A2*A3*A4*A5;
p5 = simplify(T5(1:3,end));

T4 = A1*A2*A3*A4;
p4 = simplify(T4(1:3,end));

T3 = A1*A2*A3;
p3 = simplify(T3(1:3,end));

T2 = A1*A2;
p2 = simplify(T2(1:3,end));

J = simplify(jacobian(p,q));

J6 = simplify(jacobian(p6,q));
J5 = simplify(jacobian(p5,q));
J4 = simplify(jacobian(p4,q));
J3 = simplify(jacobian(p3,q));
J2 = simplify(jacobian(p2,q));


syms t
syms Q(t) Q1(t) Q2(t) Q3(t) Q4(t) Q5(t) Q6(t) Q7(t)
Jsubs = subs(J,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});

p2 = subs(p2,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
p3 = subs(p3,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
p4 = subs(p4,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
p5 = subs(p5,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});
p6 = subs(p6,{q1,q2,q3,q4,q5,q6,q7},{Q1(t),Q2(t),Q3(t),Q4(t),Q5(t),Q6(t),Q7(t)});

dJdt = diff(Jsubs,t);

syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 real
dJdt = simplify(subs(dJdt,{diff(Q1),diff(Q2),diff(Q3),diff(Q4),diff(Q5),diff(Q6),diff(Q7)}...
    ,{dq1,dq2,dq3,dq4,dq5,dq6,dq7}));
dJdt = subs(dJdt,{Q1,Q2,Q3,Q4,Q5,Q6,Q7},{q1,q2,q3,q4,q5,q6,q7});