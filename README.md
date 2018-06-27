# UWB-Aided-UAV-lacalization

This repository includes MATLAB codes used for CSRS 2018.

The codes are for  the implementation of EKF for UAV localization using UWB and IMU.

The codes were tested on MATLAB R2015a.

Main Code: 
UWB_AIDED_EKF_based_localization.m

Function:
transition_function.m - Transition from t to t+1
transition_function2.m - Same code with transition_function.m (duplicated to apply NumJacob.m)
Rotation_e2i.m - Tait-Bryan angle to Rotation matrix
NumJacob.m - Numerical Jacobian matrix
