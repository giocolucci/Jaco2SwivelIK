# Jaco2SwivelIK
Closed form Inverse Kinematics alghoritm based on swivel (or elbow) formulation for the Kinova Jaco2 7 d.o.f. (degrees of freedom) manipulator
The alghoritm is presented as a matlab function with all the related sub-function and files.

Authors : Giovanni Colucci and Luigi Tagliavini 
Affiliation : Politecnico di Torino, Torino (To), Italy

Tested on : Dell XPS Intel® Core™ i7-10510U CPU @ 1.80GHz × 8 
OS : Ubuntu 20.04 LTS

## Run the example

Users can run the simple example TestSwiveIK.m to solve the IK problem for a generic goal pose. The script also contains a useful method to show the robot coniguration, base on the LoadJaco2 function, that loads the Kinova Jaco2 manipulator as a rigidbodytree object and plot it in a graph. To successfully run the LoadJaco2 function, please install the Robotic System Toolbox add-on from Mathworks


### Install the Robotic System Toolbox

![Install1](/Images/InstallAddOn.png)
