# kmr_iiwa_ros2_java
Java app for the Sunrise cabinet to control the KUKA KMP200 mobile base in conjunction with the IIWA arm. Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise. 

## Requirments
- Sunrise Workbench 1.17. Windows is required, but I managed to run it on ubuntu 24 with WineHQ
- [KUKA KMR_IIWA robot](https://www.kuka.com/en-us/products/amr-autonomous-mobile-robotics/mobile-robot-systems/kmr-iiwa), the mobile manipulator combining the KUKA Mobile Platform 200 (KMP-200) and the LBR iiwa 14 manipulator.
  It may work on the LBR iiwa 7 version, some configurations may need changes.
- ROS2 installation is not required in the deployment pc, but it will be necessary on the server pc communicating with the robot through the running app. Tested on Jazzy, it may work on different ROS2 versions

## How To (TODO) 
- In Sunrise Workbench, create new project
  - Better, create new importing the project existent on the robot, it will avoid configurations mismatch issues
- Copy/merge application folder and copy kmr_iiwa_control folder into the src folder of the Sunrise project.
  - **Important** Do not ever think for a second to create a git repo inside the sunrise src folder. Sunrise alredy use git in the root of the project. Wheny deploying on the robot if another git submodule is found the syncro will be stuck forever (and also the syncro back from the robot), forcing you to manually delete project folders inside the robot
- Copy the file 'log4j_*.jar from the <sunrise_workbench_install_folder>/plugins folder to the <project_directory>/KUKAJavaLib directory through sunrise workbench.
- [Mobile base sensors]. Copy also `com.kuka.nav.provider-*.jar`, `com.kuka.nav.request.provider-*.jar`, `com.kuka.nav.robot.manager.provider*.jar`. I found these libraries in the pc of the mobile base itself, in `/opt/nav/KUKAJavaLib` (default: 172.31.1.102). I could not find these in the Sunrise plugin, nor anywhere else. They seem part of the KUKA Navigation library. If you do not have these, you can still use the robot but without access to mobile base sensors. Just comment the parts in the application that you do not have access to and the whole mrb_status_commander file.
- In Sunrise workbench, right click on the log4j lib, build->add to project path. It should be present in the `Referenced Libraries` folder now
- Check the configuration file for IP addresses (StationSetup.cat, created when you create a Sunrise Project)
- [necessary?] Make the KMRiiwaControlApp default application
- [TODO]. Server pc is hardcoded for now (`String remotePC = "172.31.1.149";`), you have to change it in the TCPSocket.java file
- Deploy to the robot ("Syncronize Project" icon in the Workbench toolbar at the top)
  - If syncronizatiom (i.e. deploy on the robot) fails, you can try import the project from the robot, add the files, and the syncro. It worked for us

## How it works
- The code here is a TCP (or UDP if you want to undertand where to touch the code) client that connect to a server on different ports (i.e. the ROS2 nodes) for receiving commands as strings. Such strings are then interpreted to generate commands for the KUKA API in order to control the arm and the base

## Notes
- Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise, but adapted since I can not figure out how to install the requirments (are those for an old sunrise version?)
- Other resource: https://github.com/stoic-roboticist/kmriiwa_ros_java - ROS1, but better instructions that are useful

## Troubleshoot
### Accessing the sunrise cabinet pc
- IP default seems to be 172.31.1.147, but here we use 172.31.1.10. Default username is `kukauser`, password `kuka` (according to internet). At HRC lab we use a different password, check the reserved instruction for this.
### Syncro failures
- "Git submodules present". You put git in the project (or a subfolder) on the robot. I solved deleting manually the project folder after accessing the robot files through its ip
- "Security configuration differ". Either solve properly this, or just import the project existing in the robot, and add the file from this repo to it, then syncro it back
- Some error related to tool library. Internet said it is about a corrupted roboticsAPI config file. This file is regenerated when you modify the stationsetup.cat and you save such modification, so you can try to do this. Sometimes also rebooting the robot helped me.