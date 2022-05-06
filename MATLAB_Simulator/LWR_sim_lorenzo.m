%% INIT
clc;
clear;

Lorenzo_parameters;

while(t0 < tf)
    disp(t0);  

    %% TRAJ (CIRCLE)
    
    p_ref(1,1) = p_0(1) -  0.1 * (1 - cos(frequency * (t0)));
    dp_ref(1,1) = dp_0(1) - 0.1 * frequency * sin(frequency * (t0));
    d2p_ff(1,1) = d2p_0(1) - 0.1 * frequency * frequency * cos(frequency * (t0));

    p_ref(2,1) = p_0(2) -  0.1 * (sin(frequency * (t0)));
    dp_ref(2,1) = dp_0(2) - 0.1 * frequency * cos(frequency * (t0));
    d2p_ff(2,1) = d2p_0(2) + 0.1 * frequency * frequency * sin(frequency * (t0));
    
    p_ref(3,1) = p_0(3);
    dp_ref(3,1) = dp_0(3);
    d2p_ff(3,1) = d2p_0(3);
     
    Jold = J;

    qold = q0;
    
    p = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));         
    
    J = J_LWR(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    
    dJ = dJdt_LWR(q0(8),q0(9),q0(10),q0(11),q0(12),q0(13),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
    
    %[~,S,~] = svd(J);
    
    %sigma = min(diag(S));
    
    dp = J * q0(8:14)';
    
    d2p = dJ * q0(8:14)' + J * accs(end,:)';
    
    err_p = p_ref - p;
    
    err_dp = dp_ref - dp;
    
    
    %% PD KINEMATIC    
    d2p_ref = Kp * err_p + Kd * err_dp + d2p_ff;
    
    tspan =[t0,t0+DeltaT];
    
    
    %% EXACT PROJECTOR
    %Pj = eye(7) - pinv(J) * J;  
    
     
    % Friction : TO BE FIXED
%    friction =  0.1*(-A_friction .* sign(q0(8:14)) - 0.001 * q0(8:14));
    friction =  0.0;   
    
    %%DAMPED-LEAST SQUARE    
    
    A = (J' * J + damping * eye(length(q0(1:7))));
    B = J' * (d2p_ref - dJ * q0(8:14)');              
    X1 = lsqminnorm(A,B);    
    Uref_task = X1';
    
    %% PSEUDOINVERSE
    %Uref_task = (pinv(J) * (d2p_ref - dJ * q0(8:14)'))';
    
    uopt = zeros(7,1);
    
    %Uref_redundant = (Pj * uopt)';
    
    %Uref = Uref_task + Uref_redundant;
    Uref = Uref_task;
    
    %% COMPUTED TORQUE FL
    
    g = get_gnum(q0(1:7));
    C = get_Snum(q0(1:7),q0(8:14));
    M = get_Bnum(q0(1:7));
       
    TauFL = g+C*q0(8:14)'+(M*Uref'); 
    
    %TauFL = g;
        
    %% Simulation
    
    % External force applied at the end effector
    
    TauExt = fault_control(TauFL,t0,q0(1:7),fault);
    
    % Applied torque
    Tau = TauFL + TauExt;
    
    % Robot
    acc = (inv(M) * (Tau -friction - C*q0(8:14)' - g))';
    
    % Reduced observer
    C_hat = get_Snum(q0(1:7),x2_hat(1:7));
    z_dot = (inv(M) * (Tau - C_hat*x2_hat(1:7)' - g - M*eye(7)*k0*x2_hat(1:7)'))';       
    
    % EULER INTEGRATION
    
    % robot
    q0(1:7)  = q0(1:7) + q0(8:14) * DeltaT;    
    q0(8:14) = acc * DeltaT + q0(8:14);
    
    % reduced observer
    z = z + z_dot*DeltaT;
    x2_hat = z+k0*q0(1:7);
    
    %% RESIDUAL: actual value of joint velocity
    
    % current value of the gneralized momentum
    p_k = M*q0(8:14)' ;
    
    sum1 = sum1 + (TauFL + C'*q0(8:14)' - g)*DeltaT;
    
    if index~=1
        sum2 = sum2+r(:,index-1)*DeltaT;
    end
    
    res = inv(eye(7)+gain*DeltaT)*gain*(p_k-sum1-sum2);
    
    %% RESIDUAL: estimated value of joint velocity
    
    p_k_hat = M*x2_hat(1:7)';
    
    sum1_ob = sum1_ob + (TauFL + C_hat'*x2_hat(1:7)' -g)*DeltaT;
    
    if index~=1
        sum2_ob = sum2_ob+r_ob(:,index-1)*DeltaT;
    end
    
    res_ob = inv(eye(7)+gain*DeltaT)*gain*(p_k_hat-sum1_ob-sum2_ob-p0);
    
    
    %% Updating variables
    
    % Time
    t0 = t0+DeltaT;
    time = vertcat(time,t0);
    
    % Joint variables: both position and velocities
    joints = vertcat(joints,q0);  
    
    % Estimated velocities
    vel_hat = vertcat(vel_hat,x2_hat);
         
    % Residual
    r = horzcat(r,res);
    
    r_ob = horzcat(r_ob,res_ob);
    
    % Faulty torque
    torque_faulty = vertcat(torque_faulty, TauExt');
    
    % Arrays
%     accs = vertcat(accs,acc);
%     
%     accs_ref = vertcat(accs_ref,[Uref,Uref_task, Uref_redundant]);            
%     
     task_vec = vertcat(task_vec, [p',dp',d2p',p_ref',dp_ref',d2p_ref']);
%     
     torque_fl = vertcat(torque_fl,TauFL');
%     
%     singular_values = vertcat(singular_values,sigma);
%     
    %Updating the step
    index = index + 1;     
    
    
end

save("new_workspace.mat");
general_plots;