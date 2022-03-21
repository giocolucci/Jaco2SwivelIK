function T = DHv3(a,alpha,d,q)
%% Compute the forward kinematics according to the Denavit-Hartenberg convention

% -------------------------------------------------------------------------
% INPUT

% a       : a parameter of classic DH convention
% alpha   : alpha parameter of classic DH convention 
% d       : d parameter (all revolute joints) of classic DH convention
% q       : q variable (all revolute joints)

% -------------------------------------------------------------------------
% OUTPUT

%T       : Homogeneous transformation matrix 4x4xlength(phi). phi is the
%          swivel or elbow array, please find more info in Jaco2SwivelIK
%          documentation

% -------------------------------------------------------------------------

% Author : Giovanni Colucci & Luigi Tagliavini - Politecnico di Torino -
%          Torino (TO), Italy
% Last edit : 03/21/2022 

% -------------------------------------------------------------------------

% Resize the j-th joint angle into layers
qtmp(1,1,:) = q;
% Calculate the 4x4xlength(phi) multi-dimensional matrix
T = [cos(qtmp),    (-sin(qtmp))*(cos(alpha)),          sin(qtmp)*sin(alpha),     a*cos(qtmp);
 sin(qtmp),         cos(qtmp)*cos(alpha),     (-cos(qtmp))*(sin(alpha)),     a*sin(qtmp);
      zeros(size(qtmp)),                sin(alpha)*ones(size(qtmp)),                 cos(alpha)*ones(size(qtmp)),            d*ones(size(qtmp));
      zeros(size(qtmp)),                         zeros(size(qtmp)),                          zeros(size(qtmp)),            ones(size(qtmp))];


end

