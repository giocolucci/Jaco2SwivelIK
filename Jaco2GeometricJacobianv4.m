function [J] = Jaco2GeometricJacobianv4(q)
%% Geometric Jacobian matric for Kinova Jaco2 manipulator

% -------------------------------------------------------------------------
%INPUT

% J       : Jacobian matrix 6x7xlength(q) composed, in each layer, as follows:
%           [Jp ; Jo] (first threevrows are about translation and the last 
%           three about rotation

% -------------------------------------------------------------------------
% OUTPUT

% q  : Configuration of the manipulator 7xlength(phi). phi is the swivel angle
%      vector, please go to the Jaco2SwivelIK function for more info about it

% -------------------------------------------------------------------------

% Author : Giovanni Colucci & Luigi Tagliavini
% Last Edit : 03/21/2022

% -------------------------------------------------------------------------

% Reshape q to compute J
qtmp = reshape(q, 7,1,size(q,2));

% Calculate J as a complex symbolic function
J = [(1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))))./5000 - (41.*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)))./100 - (1319.*cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) + cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:))))./5000 - (49.*cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)))./5000 + (3111.*sin(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))))./10000 - (49.*cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))./5000 + (3111.*cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)))./10000,                                                     cos(qtmp(1,1,:)).*((41.*cos(qtmp(2,1,:)))./100 - (3111.*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)))./10000 - (49.*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)))./5000 + (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:))))./5000 + (1319.*cos(qtmp(6,1,:)).*(cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) + cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))))./5000 - (3111.*cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./10000), (49.*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))./5000 + (49.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)))./5000 + (3111.*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(4,1,:)))./10000 - (3111.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./10000 - (1319.*cos(qtmp(3,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(4,1,:)))./5000 + (1319.*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./5000 + (1319.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(6,1,:)))./5000, (3111.*cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./10000 + (3111.*cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))./10000 + (3111.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)))./10000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./5000 - (1319.*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))./5000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000, -(1319.*sin(qtmp(6,1,:)).*(cos(qtmp(3,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(5,1,:)) + cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(5,1,:))))./5000,   (sin(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) + cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:))).*((1319.*cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) + cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:))))./5000 - (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))))./5000) + (sin(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) + cos(qtmp(5,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)))).*((1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:))))./5000 + (1319.*cos(qtmp(6,1,:)).*(cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) + cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))))./5000),                                                                                                                                                                                                                                                 zeros(1,1,size(q,2));
     (49.*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)))./5000 - (41.*cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)))./100 + (1319.*cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) - cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))))./5000 - (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)))))./5000 - (3111.*sin(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))))./10000 - (49.*cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)))./5000 + (3111.*cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:)))./10000,                                                    -sin(qtmp(1,1,:)).*((41.*cos(qtmp(2,1,:)))./100 - (3111.*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)))./10000 - (49.*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)))./5000 + (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:))))./5000 + (1319.*cos(qtmp(6,1,:)).*(cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) + cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))))./5000 - (3111.*cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./10000), (49.*cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)))./5000 - (49.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)))./5000 + (3111.*cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./10000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(4,1,:)))./5000 + (3111.*cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./10000 + (1319.*cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(2,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./5000 - (1319.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(6,1,:)))./5000, (3111.*cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(3,1,:)))./10000 - (3111.*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./10000 - (3111.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)))./10000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(3,1,:)))./5000 + (1319.*cos(qtmp(6,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./5000 + (1319.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(1,1,:)))./5000 - (1319.*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(1,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000, -(1319.*sin(qtmp(6,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(5,1,:)) + cos(qtmp(2,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(5,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(5,1,:))))./5000, - (sin(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) + cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:))).*((1319.*cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) - cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))))./5000 - (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)))))./5000) - (sin(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) + cos(qtmp(5,1,:)).*(cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)))).*((1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:))))./5000 + (1319.*cos(qtmp(6,1,:)).*(cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) + cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))))./5000),                                                                                                                                                                                                                                                 zeros(1,1,size(q,2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  zeros(1,1,size(q,2)), (41.*sin(qtmp(2,1,:)))./100 + (49.*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)))./5000 - (3111.*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:)))./10000 + (3111.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(4,1,:)))./10000 + (1319.*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(2,1,:)))./5000 - (1319.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(4,1,:)))./5000 + (1319.*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000,                                                                                                                                                                                                                                                                                                                        (sin(qtmp(2,1,:)).*(98.*cos(qtmp(3,1,:)) - 3111.*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)) + 2638.*cos(qtmp(6,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(4,1,:)) + 2638.*cos(qtmp(3,1,:)).*sin(qtmp(5,1,:)).*sin(qtmp(6,1,:)) - 2638.*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(6,1,:))))./10000,                                                                                                                                                                                            (3111.*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:)))./10000 - (3111.*cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)))./10000 + (1319.*cos(qtmp(2,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(4,1,:)))./5000 - (1319.*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(2,1,:)))./5000 - (1319.*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(3,1,:)).*cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000,                                                                                      (1319.*sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(5,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(5,1,:))))./5000,                                                                                                                                                                                                                                                                                                                                                                                                                                                     (1319.*cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000 - (1319.*cos(qtmp(2,1,:)).*cos(qtmp(5,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(4,1,:)))./5000 + (1319.*cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)).*sin(qtmp(6,1,:)))./5000 + (1319.*cos(qtmp(6,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:)))./5000 + (1319.*cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*cos(qtmp(5,1,:)).*cos(qtmp(6,1,:)).*sin(qtmp(2,1,:)))./5000,                                                                                                                                                                                                                                                 zeros(1,1,size(q,2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  zeros(1,1,size(q,2)),                                                                                                                                                                                                                                                                                                                                                 -sin(qtmp(1,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                -cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                              cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:)),                                                                                                                  cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:)) - sin(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         - sin(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - cos(qtmp(5,1,:)).*(cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:))), cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) - cos(qtmp(1,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)) + cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*cos(qtmp(3,1,:))) + cos(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(3,1,:)).*sin(qtmp(1,1,:)) - cos(qtmp(1,1,:)).*cos(qtmp(2,1,:)).*sin(qtmp(3,1,:))));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  zeros(1,1,size(q,2)),                                                                                                                                                                                                                                                                                                                                                 -cos(qtmp(1,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                 sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                              cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:)),                                                                                                                - sin(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         - sin(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - cos(qtmp(5,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:))), cos(qtmp(6,1,:)).*(sin(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) + cos(qtmp(4,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(4,1,:)).*(cos(qtmp(1,1,:)).*sin(qtmp(3,1,:)) - cos(qtmp(2,1,:)).*cos(qtmp(3,1,:)).*sin(qtmp(1,1,:))) - sin(qtmp(1,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:))) - sin(qtmp(5,1,:)).*(cos(qtmp(1,1,:)).*cos(qtmp(3,1,:)) + cos(qtmp(2,1,:)).*sin(qtmp(1,1,:)).*sin(qtmp(3,1,:))));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 -ones(1,1,size(q,2)),                                                                                                                                                                                                                                                                                                                                                        zeros(1,1,size(q,2)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                         cos(qtmp(2,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                                                       -sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)),                                                                                                                                                    - cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           sin(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) + cos(qtmp(5,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)),                                                                                                     sin(qtmp(6,1,:)).*(cos(qtmp(5,1,:)).*(cos(qtmp(2,1,:)).*sin(qtmp(4,1,:)) - cos(qtmp(3,1,:)).*cos(qtmp(4,1,:)).*sin(qtmp(2,1,:))) - sin(qtmp(2,1,:)).*sin(qtmp(3,1,:)).*sin(qtmp(5,1,:))) + cos(qtmp(6,1,:)).*(cos(qtmp(2,1,:)).*cos(qtmp(4,1,:)) + cos(qtmp(3,1,:)).*sin(qtmp(2,1,:)).*sin(qtmp(4,1,:)))];
 


end