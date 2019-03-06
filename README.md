Advanced Robotics Course Material

Robot arm control tutorial with ros-control, moveit, etc.

This package is developed based on the general robot arm controller package forked from [4]. Several controllers are added for instance: motion controller, admittance controller and direct force controller. These will be explained in detail in the section below.

Elfin is 6-dof manipulator. elfin_description, elfin_gazebo is forked from [3]. elfin_control and elfin_launch are added. elfin_control has control launch file and gain yaml file.

## Introduction
Implemented various control algorithm on 6-dof Elfin manipulator simulation using ros-control frameworks.

## How to run 
### Prerequisite
Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-kinetic-effort-controllers

### Download and build 

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/trannguyenle95/arm-tutorial.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Run
Depending on the controller you want to run, use suitable launch file.
If you want to use motion controller in joint space, then you may choose this controllers as follows:

    $ roslaunch elfin_launch elfin_gravity_comp_control.launch
    or
    $ roslaunch elfin_launch elfin_time_delay_control.launch
    or
    $ roslaunch elfin_launch elfin_computed_torque_control.launch

If you want to use motion controller in task space, then you may choose this controllers as follows:
(Please refer to slide #32 IHA_4506_Advanced_Robotics_RobotMotionControl.pdf for more information)
    
    $ roslaunch elfin_launch elfin_computed_torque_control_clik.launch
    (1st option: Motion control with inverse kinematics)
    or
    $ roslaunch elfin_launch elfin_motion_control.launch
    (2nd option: Motion control without inverse kinematics)
    
    How they works?
        - You need to specify the goal for the robot to move there by publishing the goal location to          
        /elfin/controller_name/command topic
        
        - We have already made a trajectory publisher that publish a trajectory in y-axis to the mentioned topic.
        $ cd ~/catkin_ws/src
        $ git clone https://github.com/trannguyenle95/trajectory.git
        $ rosrun trajectory trajectory 
        Robot should move now !!
        
If you want to use motion and force controller in task space, then you may choose this controllers as follows:
(Please refer to IHA 4506 Advanced Robotics_Force control.pdf for more information )

    $ roslaunch elfin_launch elfin_adaptive_variable_impedance_control.launch 
    or
    $ roslaunch elfin_launch elfin_admittance_control.launch  
    (Admittance controller)
    or
    $ roslaunch elfin_launch elfin_force_control.launch 
    (Direct force controller)
        Note: The default reference force is 20 N. 
             - For visualization, run
                $ rqt_plot
                $ Add: /elfin/force_controller/publish_force/data[4] (Real force value measured at EE)
                       /elfin/force_controller/publish_force/data[5] (Desired force value)
             - To change the reference force value. Please change the value of this element:
                       /elfin/force_controller/force_gains/data[3] (Refernce force value)
                You can simply change it using rostopic pub or using rqt_ez_publisher
                $ rosrun rqt_ez_publisher rqt_ez_publisher    
           
### Tip
 If you face the problems such as robot collapses, robot behaves weird when it touch the table (force-related controller),    please do:
 
    $ cd ~/catkin_ws
    $ catkin_make clean
    $ catkin_make
    
## Reference
1. [ros-control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin manipulator](http://wiki.ros.org/Robots/Elfin)
4. [arm-control package](https://github.com/modulabs/arm-control)
