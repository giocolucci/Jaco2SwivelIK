function [q] = Solveq(pe, pw, Tb0, Tb7, spsi, cpsi, D1, L, e2, a1, alpha1, d1, theta1, a2, alpha2,d2, theta2, a3, alpha3, d3, theta3, a4, alpha4, d4, theta4)

%% Function related to the Jaco2SwivelIK alghoritm
% It finds the solution in terms of q starting from the elbow and wrist
% position vector, respectively pe and pw.

% -------------------------------------------------------------------------
% INPUT

% pe    : position vector of the elbow (m)
% pw    : position vector of the wrist (m)
% Tb0   : homogeneous transformation matrix from the base frame to link 0. It
%         is defined according to the Kinova user guide.
% Tgoal : Desired pose, expressed as a homogeneous transformation matrix
%         4x4
%[...]  : All the other terms are defined inside the Jaco2SwivelIK function

% -------------------------------------------------------------------------
% OUTPUT

% q : joint angles vector 7xlength(phi) (rad). phi is the swivel angle
%     vector, please go to the Jaco2SwivelIK function for more info about it

% -------------------------------------------------------------------------

% Author : Giovanni Colucci & Luigi Tagliavini - Politecnico di Torino -
%          Torino (TO), Italy
% Last edit : 03/21/2022 

% -------------------------------------------------------------------------
    

% Declare q as a zero matrix
q = zeros(7,size(pe,2));
% find q1
q(1, :) = - atan2(pe(2, :), pe(1, :));
% find q2
q(2, :) = atan2((sqrt(pe(1, :).^2 + pe(2, :).^2)),(D1-pe(3, :))); 
% find q4, that was also solved and defined as psi
q(4, :) = atan2(spsi, cpsi)*ones(1, size(pe,2));

% some useful notation
s1 = sin(q(1, :)); 
s2 = sin(q(2, :)); 
s4 = sin(q(4, :)); 
c1 = cos(q(1, :)); 
c2 = cos(q(2, :)); 
c4 = cos(q(4, :));

% find q3
c3 = -(e2.*s2.*pw(1) - e2.*s2.*pe(1, :) + c1.*c2.*e2.*pe(3, :) - c1.*c2.*e2.*pw(3) + L.*s1.*s4.*pe(3, :) - L.*s1.*s4.*pw(3) + L.*c1.*c2.^2.*c4.*e2 + L.*c1.*c4.*e2.*s2.^2 + L.^2.*c2.*c4.*s1.*s4)./(s2.*(s1.*L.^2.*s4.^2 + s1.*e2.^2));
s3 = (e2.*s1.*pw(3) - e2.*s1.*pe(3, :) - L.*s2.*s4.*pe(1, :) + L.*s2.*s4.*pw(1) + L.^2.*c1.*c2.^2.*c4.*s4 + L.^2.*c1.*c4.*s2.^2.*s4 - L.*c2.*c4.*e2.*s1 + L.*c1.*c2.*s4.*pe(3, :) - L.*c1.*c2.*s4.*pw(3))./(s2.*(s1.*L.^2.*s4.^2 + s1.*e2.^2));
q(3,:) = atan2(s3,c3);

% Forward kinematics to find T47. T01 and all the others are 4x4xlength(phi) 
% matrix and are computed with the DHv3 function. Please go to the related
% function for more info about it
T01 = DHv3(a1,alpha1,d1,theta1+q(1,:));
T12 = DHv3(a2,alpha2,d2,theta2+q(2,:));
T23 = DHv3(a3,alpha3,d3,theta3+q(3,:));
T34 = DHv3(a4,alpha4,d4,theta4+q(4,:));

Tb1 = pagemtimes(Tb0.*ones(size(T01)), T01);
Tb2 = pagemtimes(Tb1, T12);
Tb3 = pagemtimes(Tb2,T23);
Tb4 = pagemtimes(Tb3,T34);
T47 = pagemtimes(Tb2,Tb7);

% inv4v3 is a custom function that computes, for each layer, the inverse
% homogeneous transformation matrix. Please go to the related function for
% more info about it
T47 = pagemtimes(inv4v3(Tb4),Tb7.*ones(size(T01)));
% find q5, q6, and q7. 
q(5,:) = atan2(T47(2,3,:),T47(1,3,:));
q(7,:) = atan2(-T47(3,2,:),-T47(3,1,:));
q(6,:) = atan2(-reshape(T47(3,2,:), 1, size(T47,3))./sin(q(7,:)),reshape(T47(3,3,:), 1, size(T47,3)));

q = wrapTo2Pi(q);
end