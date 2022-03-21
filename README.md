# Jaco2SwivelIK
Closed form Inverse Kinematics alghoritm based on swivel (or elbow) formulation for the Kinova Jaco2 7 d.o.f. (degrees of freedom) manipulator.
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
- LoadJaco2 load the rigidbodytree of the manipulator according to its real joint angles. The funciton can be used to visualize the solution in terms of robot configuration
## Run the example

Users can run the simple example TestSwiveIK.m to solve the IK problem for a generic goal pose. The script also contains a useful method to show the robot coniguration, base on the LoadJaco2 function, that loads the Kinova Jaco2 manipulator as a rigidbodytree object and plot it in a graph. To successfully run the LoadJaco2 function, please install the Robotic System Toolbox add-on from Mathworks


### Install the Robotic System Toolbox

On Matlab window, open the add-on explorer
![Install1](/Images/InstallAddOn.png)

Then, search the Robotic System Toolbox and install it
![Install1](/Images/RST.png)

