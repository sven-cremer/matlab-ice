Intelligent Control and Estimation (ICE) Library.
===

MATLAB scripts being developed for the Intelligent Control and Estimation Library.  

# Setup
First create a fork on Bitbucket (type x then f). Download the repository using git clone, i.e.

    git clone https://<user-name>@bitbucket.org/<user-name>/matlab-ice.git

Navigate to the folder in MATLAB.

*Note: The Puma560 demo requires Peter Corke's Robotics Toolbox.*

# Running
### Two-link arm
To run simulation, execute

    run_NeuroAdaptiveCtrl_2DOF.m

Plot the data using

    plot_NeuroAdaptiveCtrl_2DOF.m

or play the animation with

    animate_Robot.m

### Puma560 robot

    run_NAC_Puma560.m