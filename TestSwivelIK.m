clear all 
close all 
clc

%% Inverse Kinematics alghoritm
angle = [pi/2 0 pi/2];
Tb7_goal = eul2tform(angle, 'XYZ');
Tb7_goal(1,4) =  .3 ;  Tb7_goal(2,4) = -.4 ;  Tb7_goal(3,4) =  0.8;

q = Jaco2SwivelIK(Tb7_goal, 0, 'optimize');


%% Visualization, please install the Robotic System add-on from Mathworks
robot = LoadJaco2;
show(robot, q, 'Frames','Off');