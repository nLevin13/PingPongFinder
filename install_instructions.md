# How to install PingPongFinder package:  
## Set up a new catkin_workspace  
$ mkdir -p ~/ros_workspaces/project/src  
$ cd ~/ros_workspaces/project/src  
$ catkin_init_workspace  
$ git clone <THIS REPO>  
$ git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git  
  Copy stdr_teleop package from lab 4 to here
$ cd ~/ros_workspaces/project  
$ catkin_make  
$ source devel/setup.bash  

Now, STDR should be properly installed on your machine. Try 



KYLES NOTES
For AMCL:

Install geometry2, https://github.com/ros/geometry2/tree/indigo-devel  
$ git checkout indigo-devel  
Install navigation, https://github.com/ros-planning/navigation/tree/kinetic-devel  
$ git checkout kinetic-devel  

