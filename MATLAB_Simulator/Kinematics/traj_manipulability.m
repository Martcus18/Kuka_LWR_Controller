function manipulability = traj_manipulability(traj)
    manipulability = [];
    for i=1:(length(traj))       
        Jacobian_kin = J_LWR(traj(i,1),traj(i,2),traj(i,3),traj(i,4),traj(i,5),traj(i,6));
        manipulability = vertcat(manipulability,det(Jacobian_kin * Jacobian_kin'));
    end
end

