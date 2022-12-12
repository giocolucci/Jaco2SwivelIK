# Jaco2SwivelIK
Closed form Inverse Kinematics alghoritm based on swivel (or elbow) formulation for the Kinova Jaco2 7 d.o.f. (degrees of freedom) manipulator.
List of references:
1. G. Colucci, A. Botta, L. Tagliavini, P. Cavallone, L. Baglieri, & G. Quaglia, Kinematic Modeling and Motion Planning of the Mobile Manipulator Agri. Q for Precision Agriculture. Machines, 10 (2022) 321.
2. G. Colucci, L. Baglieri, A. Botta, P. Cavallone, & G. Quaglia, Optimal Positioning of Mobile Manipulators Using Closed Form Inverse Kinematics. International Conference on Robotics in Alpe-Adria Danube Region (Springer, 2022), pp. 184–191.

The reference frame of the serial kinematic chain are chosen according to the Kinova manual, that can be found at:
https://drive.google.com/file/d/1xQbkx1-v3SfAentKR9f3p3c2SVdViyQl/view.

The joint angles are consistent with the real joint angles, then the alghoritm can be used to directly control the real robot.
The alghoritm is presented as a matlab function with all the related sub-function and files.

Authors : Giovanni Colucci and Luigi Tagliavini

Affiliation : Politecnico di Torino, Torino (To), Italy

Tested on : Dell XPS Intel® Core™ i7-10510U CPU @ 1.80GHz × 8 
OS : Ubuntu 20.04 LTS

User can find all the explanation of the alghoritm inside the scripts. The main function and central functions are :

- Jaco2SwivelIK computes the inverse kinematics solution. the sub-functions are:
  - DHv3
  - inv4v3
  - Jaco2GeometricJacobianv4
  - Solveq 
- LoadJaco2 load the rigidbodytree of the manipulator according to its real joint angles. The funciton can be used to visualize the solution in terms of robot configuration. The related documents are:
  - Robot Description/ . The whole folder is copied from the Kinova ROS project for the Jaco 2: https://github.com/Kinovarobotics/kinova-ros.
 
  Please note that LoadJaco2 loads the robot geometries from the Robot Description folder, so an error might occur if the repository structure is compromised.
  
## Run the example

Users can run the simple example TestSwiveIK.m to solve the IK problem for:
- a specific goal posture ("fixed" swivelIKOption), where the user must specify the corresponding elbow angle;
- extract the best posture for a specified pose ("optimize" swivelIKOption) according to a modified version of the Togai manipulability index;
- compute the whole set of possible solutions with a discretization step ("all" swivelIKOption). 

The script also contains a useful method to show the robot configuration, based on the LoadJaco2 function, that loads the Kinova Jaco2 manipulator as a rigidbodytree object and plot it in a graph. To successfully run the LoadJaco2 function, please install the Robotic System Toolbox add-on from Mathworks.

### Install the Robotic System Toolbox

On Matlab window, open the add-on explorer
![Install1](/Images/InstallAddOn.png)

Then, search the Robotic System Toolbox and install it
![Install1](/Images/RST.png)

