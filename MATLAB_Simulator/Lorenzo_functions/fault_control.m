function uf = fault_control(u,t,q,fault)
% uf = fault_control(u,t,q,fault)
% this function generetes an external torque acting on the system that can
% be a fault in the actuator or an external force

%global c1x force l1 c2x

uf = zeros(7,1);

switch fault
    
    case 0
        uf = zeros(7,1);
    case 1
        if t >= 5
            uf(1) = -0.9*u(1);
        end
    case 2
        if t >= 5
            uf(2) = -0.9*u(2);
        end        
    case 3
        if t >= 5 
            uf(3) = -0.4*u(3);
        end
    case 4
        if t >= 5
            uf(4) = -0.7*u(4);
        end        
    case 5
        if t >= 5
            uf(5) = -0.9*u(5);
        end
    case 6
        if t >= 5
            uf(6) = -0.7*u(6);
        end
    case 7
        if t >= 5
            uf(7) = -0.9*u(7);
        end 
    case 8
        if t>=5
            J = J_LWR(q(1),q(2),q(3),q(4),q(5),q(6));
            uf = -J'*[0;10;10];
        end
    case 9
        if t>=5
            uf(3) = -0.2*u(3);
            uf(4) = -0.2*u(4);
            uf(2) = -0.9*u(2);
        end
            
        
end

end

