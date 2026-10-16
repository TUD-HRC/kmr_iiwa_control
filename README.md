# kmr_iiwa_ros2_java
Java app for the Sunrise cabinet to control the KUKA KMP200 mobile base in conjunction with the IIWA arm. Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise. 

## Requirments
- Sunrise Workbench 1.17
- [KUKA KMR_IIWA robot](https://www.kuka.com/en-us/products/amr-autonomous-mobile-robotics/mobile-robot-systems/kmr-iiwa), the mobile manipulator combining the KUKA Mobile Platform 200 (KMP-200) and the LBR iiwa 14 manipulator.
  It may work on the LBR iiwa 7 version, some configurations may need changes.
- ROS2 installation is not required in the deployment pc, but it will be necessary on the server pc communicating with the robot through the running app. Tested on Jazzy, it may work on different ROS2 versions

## How To (TODO) 
- copy the files 'log4j_*.jar from the plugins folder in your Sunrise Workbench installation directory.
- In Sunrise Workbench, create new project
- Copy the content of src into the src folder of the Sunrise project. 
- Check the configuration file for IP addresses (StationSetup.cat, created when you create a Sunrise Project)
- Make the KMRiiwaRos2App default application (is this necessary?)
- Deploy to the robot ("Syncronize Project" icon in the Workbench toolbar at the top)
  - If syncronizatiom (i.e. deploy on the robot) fails, you can try import the project from the robot, add the files, and the syncro. It worked for us

## How it works
- The code here is a TCP (or UDP if you want to undertand where to touch the code) client that connect to a server on different ports (i.e. the ROS2 nodes) for receiving commands as strings. Such strings are then interpreted to generate commands for the KUKA API in order to control the arm and the base

## Notes
- Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise, but adapted since I can not figure out how to install the requirments (are those for an old sunrise version?)
- Other resource: https://github.com/stoic-roboticist/kmriiwa_ros_java - ROS1, but better instructions that are useful
- 
