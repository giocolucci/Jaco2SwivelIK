function invT = inv4v3(T)
%% Inverse matrix of homogeneous transformation matrix 4x4 T

% -------------------------------------------------------------------------
% INPUT

% T   : Homogeneous Transformation Matrix 4x4xm , where m is the number of
%       layers

% -------------------------------------------------------------------------
% OUTPUT

% invT : Inverse transformation matrix 4x4xm

% -------------------------------------------------------------------------

% Author : Luigi Tagliavini - Politecnico di Torino -
%          Torino (TO), Italy
% Last edit : 03/21/2022 

% -------------------------------------------------------------------------

 invT = [pagetranspose(T(1:3,1:3,:)) -pagemtimes(pagetranspose(T(1:3,1:3,:)),T(1:3,4,:)); [0 0 0 1].*ones(1,1,size(T,3))];
end

