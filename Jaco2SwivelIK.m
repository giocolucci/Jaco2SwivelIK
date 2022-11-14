function [q, cmod] = Jaco2SwivelIK(Tgoal,phi, swivelIKOption)

%% IK solver for Kinova Jaco2 7dof manipulator based on Swivel Angle.
% The function can be used in three ways:
% 
% - Starting from a specified target pose (Tgoal), the function calculates the
% configuration (q) at a desired swivel or elbow angle phi. In this case,
% the swivelIKOption must be 'fixed'
% - The function returns the solution with the higher manipulability index.
% In that case, the swivelIKOption must be 'optimize'. 
% - The function returns all the possible set of solutions. In that case,
% the swivelIKOption must be 'all'
% The reference frame (r.f.) definition is done with the Denhavit-Hartenberg 
% (D.H.) convention and it is consistent with the official Kinova user guide:
% https://drive.google.com/file/d/1xQbkx1-v3SfAentKR9f3p3c2SVdViyQl/view

% Thus, the configuration can be passed through ROS or SDK to the real
% robot to obain the same software configuration.

% The script was successfully tested on a Dell XPS machine with an 
% Intel i7-10510U @ 1.8 GHz processor and Linux Ubuntu 20.04 LTS. The
% average time for the 'fixed' option is 0.001 s and 0.11 s for the 'optimize'
% option.
% -------------------------------------------------------------------------
% INPUT

% Tgoal : Homogeneous 4x4 Transformation Matrix from robot base (b) to end
% effector (7)
% phi : swivel angle [0,2*pi] (rad)
% swivelIKOption : 'fixed' : IK is solved with the given phi angle
%                  'optimize' : IK is solved optimizing the manipulabilty
%                  index of the linear twists part of the jacobian matrix
%                  of the manipulator
%                  'all' : IK is solved for aech specified value of the
%                          discretized elbow angle range phi=[0,2*pi]
% -------------------------------------------------------------------------
% OUTPUT

% q : joint angles vector if 'fixed' or 'optimize' 7x1 (rad)
%                         if 'all'                 7xnpoints (rad)
% cmod : value of the modified manipulability index. 
%        if 'fixed' or 'optimize' 1x1 
%        if 'all'                 npoints x 1 vector
% -------------------------------------------------------------------------

% Author : Giovanni Colucci & Luigi Tagliavini - Politecnico di Torino -
%          Torino (TO), Italy
% Last edit : 11/14/2022 

% -------------------------------------------------------------------------

%% Robot parameters
% Define the fixed transformation matrix from the base frame of the
% manipulator to link 0. 
Tb0 = [1  0  0 0;
       0 -1  0 0;
       0  0 -1 0;
       0  0  0 1];

% Geometric parameters and D.H. parameters
D1 = .2755; %m
D2 = .2050;
D3 = D2;
D4 = .2073;
D5 = .1038;
D6 = .1038;
D7 = .16;
e2 = .0098;

a1 = 0; alpha1 = -pi/2 ; d1 = -D1      ; theta1 =0;
a2 = 0; alpha2 = -pi/2 ; d2 = 0        ; theta2 =0; 
a3 = 0; alpha3 = -pi/2 ; d3 = -(D2+D3) ; theta3= 0;
a4 = 0; alpha4 = -pi/2 ; d4 = -e2      ; theta4 =0;
a5 = 0; alpha5 = -pi/2 ; d5 = -(D4+D5) ; theta5 =0;
a6 = 0; alpha6 = -pi/2 ; d6 = 0        ; theta6 =0;
a7 = 0; alpha7 = -pi   ; d7 = -(D6+D7) ; theta7= 0;

%% Swivel or elbow angle formulation
%Find the position vector of the wrist pw
pw = Tgoal(1:3,4) - ((D6+D7).*Tgoal(1:3,3));
% Shoulder position vector ps
ps = D1 .* [0 0 1]';

% Charateristic lengths in shoulder-elbow-wrist plane
U = D2 + D3; % (m)
L = D4 + D5; % (m)
d = pw - ps;   
ddp = sqrt(norm(d)^2 - e2^2);  
Lp = sqrt(L^2 + e2^2);
alpha = (acos(((U.^2) + (norm(d).^2) - (Lp.^2))/(2 .* norm(d) .* U)));

% joint angle 4 is immediately calculated as psi angle of the swivel
% formulation
cpsi = (L^2 + U^2 - ddp^2)/(2*U*L);       %cosine of psi angle
spsi = sqrt(1 - cpsi^2);                  %sine of psi angle

%normal unit vector to the sviwel circumference
n = (d)./norm(d);
%to find a orthonormal base, define an arbitrary unit vector a
a = [1 0 0]';
% extract part of a that is the perpendicular to n
u = (a - (dot(a,n) * n))/norm(a - (dot(a,n) * n));  % dot(a,n) * n is the component of a // to n . So a - dot(a,n)*n is the remaining part of a perpendicular to n
% complete the triplet with a third vector that is perpendicular to n and v
v = cross(n,u); 

% The position vector of the center of swivel circunference pc can be
% calculated and also its radius R
pc = ps + ((U * cos(alpha)) * n) ;
R = U * sin(alpha);

%% Conditional part to choose between fixed or optimize option

if strcmp(swivelIKOption, 'fixed') 
    % If phi is fixed, the position vector of the elbow point pe can be
    % calculated starting from pc and moving along a direction defined by
    % phi of a quantity that is equal to R
    pe = pc + (R.*cos(phi).*u) + (R.*sin(phi).*v); 
    % Now the solveq function can be used to find q. Please go to the
    % solveq description to find more info about it
    q = Solveq(pe, pw, Tb0, Tgoal, spsi, cpsi, D1, L, e2, a1, alpha1, d1, theta1, a2, alpha2,d2, theta2, a3, alpha3, d3, theta3, a4, alpha4, d4, theta4);
    
    % Define the manipulator joint limits as a 3x2 matrix (there're
    % actually limits on joints 2,4,6)
    jnt2PositionLimits = (deg2rad([47, 313]));
    jnt4PositionLimits = (deg2rad([30, 330]));
    jnt6PositionLimits = (deg2rad([65, 295]));
    jntPositionLimits = [jnt2PositionLimits; jnt4PositionLimits; jnt6PositionLimits];
    grad_h = @(xx) (((jntPositionLimits(:,2)-jntPositionLimits(:,1)).^2) .* (2.*xx - jntPositionLimits(:,2) - jntPositionLimits(:,1)))./(4.*(jntPositionLimits(:,2) - xx).^2 .* (xx - jntPositionLimits(:,1)).^2); 
    TempJacobian = Jaco2GeometricJacobianv4(q);
    % Calculate the penalization factors for each solution due to the joint limits 
    Pen_Factor_jointlimits = 1./(1 + abs(grad_h(pagetranspose([reshape(q(2,:), 1, 1, size(q,2)) reshape(q(4,:), 1, 1, size(q,2)) reshape(q(6,:), 1, 1, size(q,2))])))).^0.5;
    % Modify the Jacobian according to the penalization factors.
    TempJacobianMod = TempJacobian .* [ones(6,1, size(q,2)), Pen_Factor_jointlimits(1,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(2,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(3,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2))];
    
    cmod = 1./cond(TempJacobianMod(1:3, :, 1));

    % Set cmod = 0 if the solution is not feasible due to physical joint
    % limit constraints
    % Please note that it is not important for the 'optimize' option
    % because the penalization factors exlude the solutions that are not
    % possible
    if q(2) < deg2rad(47) || q(2)>deg2rad(313) || q(4) < deg2rad(30) || q(4)>deg2rad(330) || q(6) < deg2rad(65) || q(6)>deg2rad(295)
    cmod = 0;
    end


    % Optional: add a warning if the solution is not feasible due to the
    % joint limits
    % if q(2) < deg2rad(47) || q(2)>deg2rad(313)
    %     warning('Joint 2 exceeds joint limits')
    % elseif q(4) < deg2rad(30) || q(4)>deg2rad(330)
    %     warning('Joint 4 exceeds joint limits')
    % elseif q(6) < deg2rad(65) || q(6)>deg2rad(295)
    %     warning('Joint 6 exceeds joint limits')
    % end
    % Please note that it is not important for the 'optimize' option
    % because the penalization factors exlude the solutions that are not
    % possible
elseif strcmp(swivelIKOption, 'optimize')
    % If phi can be optimized to find the solution with the highest
    % manipulability index, we can define a discretization for phi 
    phi = 0:1e-3:2*pi;
    % The position vector of the elbow can be found as a function of phi
    pe = pc + (R.*cos(phi).*u) + (R.*sin(phi).*v);  
    
    % Define the manipulator joint limits as a 3x2 matrix (there're
    % actually limits on joints 2,4,6)
    jnt2PositionLimits = (deg2rad([47, 313]));
    jnt4PositionLimits = (deg2rad([30, 330]));
    jnt6PositionLimits = (deg2rad([65, 295]));
    jntPositionLimits = [jnt2PositionLimits; jnt4PositionLimits; jnt6PositionLimits];
    % define the gradient funciton that can be used to penalize the
    % jacobian matrix
    grad_h = @(xx) (((jntPositionLimits(:,2)-jntPositionLimits(:,1)).^2) .* (2.*xx - jntPositionLimits(:,2) - jntPositionLimits(:,1)))./(4.*(jntPositionLimits(:,2) - xx).^2 .* (xx - jntPositionLimits(:,1)).^2); 
    
    % declare q as a zeros matrix
    q = zeros(7, size(pe,2));
    % declare the manipulability index klin3
    cmod = zeros(size(pe,2), 1);
    % Use the function solveq to find all the solution associated to the
    % phi range. q is a 7x(length(phi) matrix.
    % Please go to the solveq function description to find more info about
    % it.
    q = Solveq(pe, pw, Tb0, Tgoal, spsi, cpsi, D1, L, e2, a1, alpha1, d1, theta1, a2, alpha2,d2, theta2, a3, alpha3, d3, theta3, a4, alpha4, d4, theta4);
    % Evaluate the geometric jacobian matrix of each solution as a
    % 6x7x(length(phi)) matrix. Each layer of the multi-dimensional matrix
    % corresponds to the jacobian matrix of a solution
    TempJacobian = Jaco2GeometricJacobianv4(q);
    % Calculate the penalization factors for each solution due to the joint limits 
    Pen_Factor_jointlimits = 1./(1 + abs(grad_h(pagetranspose([reshape(q(2,:), 1, 1, size(q,2)) reshape(q(4,:), 1, 1, size(q,2)) reshape(q(6,:), 1, 1, size(q,2))])))).^0.5;
    % Modify the Jacobian according to the penalization factors.
    TempJacobianMod = TempJacobian .* [ones(6,1, size(q,2)), Pen_Factor_jointlimits(1,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(2,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(3,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2))];
    % Evaluate the 2-norm condition number of the linear twists part of the jacobian matrix. 
    for count_pe = 1 : size(pe, 2)
    cmod(count_pe) = 1./cond(TempJacobianMod(1:3, :, count_pe));
    end
    % Find the max value of the cmod factor
    [cmod, index] = max(cmod);
    % Find the associated best solution
    q = q(:,index);

elseif strcmp(swivelIKOption, 'all')
    % If phi can be optimized to find the solution with the highest
    % manipulability index, we can define a discretization for phi 
    phi = 0:1e-3:2*pi;
    % The position vector of the elbow can be found as a function of phi
    pe = pc + (R.*cos(phi).*u) + (R.*sin(phi).*v);  
    
    % Define the manipulator joint limits as a 3x2 matrix (there're
    % actually limits on joints 2,4,6)
    jnt2PositionLimits = (deg2rad([47, 313]));
    jnt4PositionLimits = (deg2rad([30, 330]));
    jnt6PositionLimits = (deg2rad([65, 295]));
    jntPositionLimits = [jnt2PositionLimits; jnt4PositionLimits; jnt6PositionLimits];
    % define the gradient funciton that can be used to penalize the
    % jacobian matrix
    grad_h = @(xx) (((jntPositionLimits(:,2)-jntPositionLimits(:,1)).^2) .* (2.*xx - jntPositionLimits(:,2) - jntPositionLimits(:,1)))./(4.*(jntPositionLimits(:,2) - xx).^2 .* (xx - jntPositionLimits(:,1)).^2); 
    
    % declare q as a zeros matrix
    q = zeros(7, size(pe,2));
    % declare the manipulability index klin3
    cmod = zeros(size(pe,2), 1);
    % Use the function solveq to find all the solution associated to the
    % phi range. q is a 7x(length(phi) matrix.
    % Please go to the solveq function description to find more info about
    % it.
    q = Solveq(pe, pw, Tb0, Tgoal, spsi, cpsi, D1, L, e2, a1, alpha1, d1, theta1, a2, alpha2,d2, theta2, a3, alpha3, d3, theta3, a4, alpha4, d4, theta4);
    % Evaluate the geometric jacobian matrix of each solution as a
    % 6x7x(length(phi)) matrix. Each layer of the multi-dimensional matrix
    % corresponds to the jacobian matrix of a solution
    TempJacobian = Jaco2GeometricJacobianv4(q);
    % Calculate the penalization factors for each solution due to the joint limits 
    Pen_Factor_jointlimits = 1./(1 + abs(grad_h(pagetranspose([reshape(q(2,:), 1, 1, size(q,2)) reshape(q(4,:), 1, 1, size(q,2)) reshape(q(6,:), 1, 1, size(q,2))])))).^0.5;
    % Modify the Jacobian according to the penalization factors.
    TempJacobianMod = TempJacobian .* [ones(6,1, size(q,2)), Pen_Factor_jointlimits(1,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(2,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2)), Pen_Factor_jointlimits(3,1,:).*ones(6,1, size(q,2)),...
                                                ones(6,1, size(q,2))];
    % Evaluate the 2-norm condition number of the linear twists part of the jacobian matrix. 
    for count_pe = 1 : size(pe, 2)
    cmod(count_pe) = 1./cond(TempJacobianMod(1:3, :, count_pe));

    % Impose cmod = 0 if the solution is forbidden due to physical joint
    % limit constraints
    % Please note that it is not important for the 'optimize' option
    % because the penalization factors exlude the solutions that are not
    % possible
    if q(2, count_pe) < deg2rad(47) || q(2, count_pe)>deg2rad(313)
         cmod(count_pe) = 0;
    elseif q(4, count_pe) < deg2rad(30) || q(4, count_pe)>deg2rad(330)
         cmod(count_pe) = 0;
    elseif q(6, count_pe) < deg2rad(65) || q(6, count_pe)>deg2rad(295)
         cmod(count_pe) = 0;
    end
    end

else 
    error('Select a valid option entry')
end
end
