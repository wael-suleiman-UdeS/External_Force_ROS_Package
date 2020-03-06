## License
GNU GENERAL PUBLIC LICENSE Version 2

## Reference

If you use this package, we would appreciate that you cite the following paper:  

@article{Hawley:2019yg,
	Author = {Louis Hawley and R{\'e}my Rahem and Wael Suleiman},
	Journal = {International Journal of Humanoid Robotics},
	Number = {06},
	Pages = {1-25},
	Title = {External Force Observer for Small- and Medium-Sized Humanoid Robots},
	Volume = {16},
	Year = {2019}}


## Summary

This project regroups ROS package to estimate a force applied to a robot equipped with force sensing resistors.   
The repository overall structure:

Matlab folder contains the Matlab files for simulation and testing (please refer to "Matlab/ReadMe.txt" for more details).

Source folder contains the project source code.

## Installation

This project works with ROS indigo under Ubuntu 14.04 LTS.  

install ros indigo (using the ros indigo installation guide on the ros wiki: http://wiki.ros.org/indigo/Installation/Ubuntu)
	
install all dependencies (in folder "source"):  
	rosdep install --from-paths src --ignore-src -r -y
	
Build the project (in folder "source"):  
    To clean then build:  ". BUILDPROJECT 1"
    To only build :       ". BUILDPROJECT"

## Requirements

* Eigen  = v. 3
* cmake >= v. 2.8.3
* ROS Indigo
* Boost >= 1.54

## Project Structure
	
force_observer : A kalman-based force observer for humanoid robots.

force_observer_msgs : Messages required by the kalman-based force observer.

Third-Party : Necessary third party packages.

## Usage

Launch the kalman force observer  
    roslaunch force_observer kalman_force_observer.launch

Reset the observer state by calling "/Reset_observer" service  
    run rqt_service_caller with "rosrun rqt_service_caller rqt_service_caller"  
    Choose "/Reset_observer" in the list  
    Press the "Call" button.
