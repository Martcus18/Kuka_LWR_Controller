function traj_p = cartesian_traj(traj_x)
    traj_p = zeros(length(traj_x),3);
    for i=1:length(traj_x)
        q0 = traj_x(i,1:7);
        p_0 = f(q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
%         vertcat(traj_p,p_0');
        traj_p(i,:) = p_0';
    end
end

