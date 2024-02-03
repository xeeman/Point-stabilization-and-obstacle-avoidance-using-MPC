
**********This folder contains Matlab code for solving point stabilization problems with static and dynamic obstacles avoidance.*************
Matlab version 2018b
casadi toolbox (casadi-windows-matlabR2016a-v3.4.5) must be downloaded here or from the Casadi website(https://web.casadi.org).

1. For static obstacles avoidance
Run the m-file Main_static_obs.m: it will call the simulnk file (Sim_PS_StaticOBS) which ultemately call the control file (single_robot_obs).
2. For Dynamic obstacles avoidance
Run the m-file Main_dynamic_obs.m: it will call the simulnk file (Sim_PS_DynamicOBS) which ultemately call the control file (single_robot_obs).

NB: The control file is matlab system function block, it must be simulated 
using "interpreted execution" not "code generation" by right clicking on the block.

2. Don't forget to cite my paper titled 
   "Dynamic obstacles avoidance using NMPC", "Real-time Dynamic obstacles avoidance using NMPC"
    By Mukhtar Sani et al (mukhtar.sani2707@gmail.com); Google scholar, researchgate, IeeeExplore e.t.c
