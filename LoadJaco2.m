function [robot] = LoadJaco2()
%% Build the Kinova Jaco2 manipulator model as a rigidbodytree
% The dh convention was adopted starting from the manual
% and adjusting the values according to the real joint measures.
% Thanks to this, the model can be used to send messages to the real robot.

% -------------------------------------------------------------------------

% Author       : Giovanni Colucci - Politecnico di Torino - Torino (TO), Italy
% Last edit    : 03/21/2022 
% Used toolbox : Robotic Toolbox
% -------------------------------------------------------------------------

% Manipulator parameters
D1 = 0.2755; %m
D2 = 0.2050;
D3 = 0.2050;
D4 = 0.2073;
D5 = 0.1038;
D6 = 0.1038;
D7 = 0.1600;
e2 = 0.0098;

% D.H. parameters
            %a    alpha      d       theta
dhparams =[ 0,       -pi/2,      -D1,    0;
            0,       -pi/2,        0,    0;
            0,       -pi/2, -(D2+D3),    0;
            0,       -pi/2,      -e2,    0;
            0,       -pi/2, -(D4+D5),    0;
            0,       -pi/2,        0,    0;
            0,         -pi, -(D6+D7),    0
          ];

% Define the home config
homejnt1=wrapToPi(deg2rad(-76.6854476928711));
homejnt2=wrapToPi(deg2rad(162.86973571777344));
homejnt3=wrapToPi(deg2rad(0.07373046875));
homejnt4=wrapToPi(deg2rad(43.469825744628906));
homejnt5=wrapToPi(deg2rad(-94.39232635498047));
homejnt6=wrapToPi(deg2rad(257.2372131347656));
homejnt7=wrapToPi(deg2rad(-71.64105224609375));


% Initialize the model
robot = rigidBodyTree("DataFormat",'column');

% Manipulator body
body0 = rigidBody('body0');
body1 = rigidBody('body1');
body2 = rigidBody('body2');
body3 = rigidBody('body3');
body4 = rigidBody('body4');
body5 = rigidBody('body5');
body6 = rigidBody('body6');
body7 = rigidBody('body7');

% Fingers
body8 = rigidBody('body8');
body9 = rigidBody('body9');
body10 = rigidBody('body10');
body11 = rigidBody('body11');

% Manipulator joints
jnt0 = rigidBodyJoint('jnt0', 'fixed');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
jnt4 = rigidBodyJoint('jnt4', 'revolute');
jnt5 = rigidBodyJoint('jnt5', 'revolute');
jnt6 = rigidBodyJoint('jnt6', 'revolute');
jnt7 = rigidBodyJoint('jnt7', 'revolute');

% Finger joints
jnt8 = rigidBodyJoint('jnt8', 'fixed');
jnt9 = rigidBodyJoint('jnt9', 'fixed');
jnt10 = rigidBodyJoint('jnt10', 'fixed');
jnt11 = rigidBodyJoint('jnt11', 'fixed');

% Apply rigid transformation according to dh convention
setFixedTransform(jnt0, eul2tform([0 0 pi]))
setFixedTransform(jnt1, dhparams(1,:), 'dh');
setFixedTransform(jnt2, dhparams(2,:), 'dh');
setFixedTransform(jnt3, dhparams(3,:), 'dh');
setFixedTransform(jnt4, dhparams(4,:), 'dh');
setFixedTransform(jnt5, dhparams(5,:), 'dh');
setFixedTransform(jnt6, dhparams(6,:), 'dh');
setFixedTransform(jnt7, dhparams(7,:), 'dh');

setFixedTransform(jnt8, [eul2rotm([0 pi/2 0.72*pi], 'ZYZ'), [0 0.03 -0.044]'; [0 0 0 1]])
setFixedTransform(jnt9, [eul2rotm([0 -pi/2 -.28*pi], 'ZYZ'), [0 -0.03 -0.044]'; [0 0 0 1]])
setFixedTransform(jnt10, [eul2rotm([0 0 0.0], 'ZYZ'), [0.043 0 0]'; [0 0 0 1]])
setFixedTransform(jnt11, [eul2rotm([0 0 0.0], 'ZYZ'), [0.043 0 0]'; [0 0 0 1]])

% Add the home configs
jnt1.HomePosition = homejnt1;
jnt2.HomePosition = homejnt2;
jnt3.HomePosition = homejnt3;
jnt4.HomePosition = homejnt4;
jnt5.HomePosition = homejnt5;
jnt6.HomePosition = homejnt6;
jnt7.HomePosition = homejnt7;

% joint limits
jnt2.PositionLimits = (deg2rad([47, 313]));
jnt4.PositionLimits = (deg2rad([30, 330]));
%jnt6.PositionLimits = (deg2rad([65, 295]));

% add the joint
body0.Joint = jnt0;
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

body8.Joint = jnt8;
body9.Joint = jnt9;
body10.Joint = jnt10;
body11.Joint = jnt11;

%Add visual and collision
addVisual(body0,'Mesh','./Robot Description/base.dae', eul2tform([pi pi 0]))
addVisual(body1,'Mesh','./Robot Description/shoulder.dae', [eul2rotm([0 pi pi/2]), [0 -0.122639+0.005 0]'; [0 0 0 1]])
addVisual(body2,'Mesh','./Robot Description/arm_half_1.dae', eul2tform([0 0 pi/2]))
addVisual(body3,'Mesh','./Robot Description/arm_half_2.dae', [eul2rotm([0 0 pi/2]), [0 -D3 0]'; [0 0 0 1]])
addVisual(body4,'Mesh','./Robot Description/forearm.dae', [eul2rotm([0 pi pi/2]), [0 -.013+0.001 0]'; [0 0 0 1]])
addVisual(body5,'Mesh','./Robot Description/wrist_spherical_1.dae', [eul2rotm([0 0 pi/2]), [0 -D5 0]'; [0 0 0 1]])
addVisual(body6,'Mesh','./Robot Description/wrist_spherical_2.dae', [eul2rotm([0 pi pi/2]), [0 0 0]'; [0 0 0 1]])
addVisual(body7,'Mesh','./Robot Description/hand_2finger.dae', [eul2rotm([0 pi 0]), [0 0 -D7]'; [0 0 0 1]])

addVisual(body8,'Mesh','./Robot Description/finger_proximal.dae', [eul2rotm([0 0 0], 'ZYZ'), [0 0 0]'; [0 0 0 1]])
addVisual(body9,'Mesh','./Robot Description/finger_proximal.dae', [eul2rotm([0 0 0], 'ZYZ'), [0 0 0]'; [0 0 0 1]])
addVisual(body10,'Mesh','./Robot Description/finger_distal.dae', [eul2rotm([0 0 0], 'ZYZ'), [0 0 0]'; [0 0 0 1]])
addVisual(body11,'Mesh','./Robot Description/finger_distal.dae', [eul2rotm([0 0 0], 'ZYZ'), [0 0 0]'; [0 0 0 1]])

addCollision(body0,'Mesh','./Robot Description/base.dae', eul2tform([pi pi 0]))
addCollision(body1,'Mesh','./Robot Description/shoulder.dae', [eul2rotm([0 pi pi/2]), [0 -0.122639+0.005 0]'; [0 0 0 1]])
addCollision(body2,'Mesh','./Robot Description/arm_half_1.dae', eul2tform([0 0 pi/2]))
addCollision(body3,'Mesh','./Robot Description/arm_half_2.dae', [eul2rotm([0 0 pi/2]), [0 -D3 0]'; [0 0 0 1]])
addCollision(body4,'Mesh','./Robot Description/forearm.dae', [eul2rotm([0 pi pi/2]), [0 -.013+0.001 0]'; [0 0 0 1]])
addCollision(body5,'Mesh','./Robot Description/wrist_spherical_1.dae', [eul2rotm([0 0 pi/2]), [0 -D5 0]'; [0 0 0 1]])
addCollision(body6,'Mesh','./Robot Description/wrist_spherical_2.dae', [eul2rotm([0 pi pi/2]), [0 0 0]'; [0 0 0 1]])
addCollision(body7,'Mesh','./Robot Description/hand_2finger.dae', [eul2rotm([0 pi 0]), [0 0 -D7]'; [0 0 0 1]])

addCollision(body8,'cylinder', [0.017 0.033], [eul2rotm([0 pi/2 0], 'ZYZ'), [0.033/2 + 0.007 -0.02 0]'; [0 0 0 1]])
addCollision(body9,'cylinder', [0.017 0.033], [eul2rotm([0 pi/2 0], 'ZYZ'), [0.033/2 + 0.007 -0.02 0]'; [0 0 0 1]])
addCollision(body10,'cylinder', [0.017 0.04], [eul2rotm([0 pi/2 0], 'ZYZ'), [0.021 -0.02 0]'; [0 0 0 1]])
addCollision(body11,'cylinder', [0.017 0.04], [eul2rotm([0 pi/2 0], 'ZYZ'), [0.021 -0.02 0]'; [0 0 0 1]])



% Finally add bodies to the model
addBody(robot, body0,'base');
addBody(robot, body1,'body0');
addBody(robot, body2,'body1');
addBody(robot, body3,'body2');
addBody(robot, body4,'body3');
addBody(robot, body5,'body4');
addBody(robot, body6,'body5');
addBody(robot, body7,'body6');

addBody(robot, body8, 'body7');
addBody(robot, body9, 'body7');
addBody(robot, body10, 'body8');
addBody(robot, body11, 'body9');
end

