# kmr_iiwa_control

Java application that runs on the KUKA Sunrise cabinet to control the KUKA KMP 200 mobile platform together with the LBR iiwa arm. The code was taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise.

## Description

The application spawns several threads, each acting as a TCP/UDP client that connects to an external server on a different port. Every thread uses the KUKA API for a specific robot functionality. The app is standalone and ROS agnostic, so in principle any client can communicate with it. Most likely, however, you are here because you want to use the ROS 2 nodes from https://github.com/TUD-HRC/kmr_iiwa_ros2.

This code is the client side of the communication: it connects to a server listening on each port (typically the ROS 2 nodes) and exchanges packets as strings. Incoming packets are interpreted and translated into KUKA API commands that control the arm and the mobile base.

## Requirements

- Sunrise Workbench 1.17. Windows is required, although I managed to run it on Ubuntu 24 with WineHQ. Some troubleshooting steps may still require Windows.
- [KUKA KMR iiwa robot](https://www.kuka.com/en-us/products/amr-autonomous-mobile-robotics/mobile-robot-systems/kmr-iiwa), the mobile manipulator that combines the KUKA Mobile Platform 200 (KMP 200) and the LBR iiwa 14 manipulator.
  It may also work with the LBR iiwa 7, but some configurations may need to be changed.
- For ROS 2 use, https://github.com/TUD-HRC/kmr_iiwa_ros2 must be installed on the external PC.

## Installation

- **Important**: Back up the existing project on the robot first (for example by importing it from Sunrise Workbench), if there is one. Sunrise is fragile and file corruption is common.
- In Sunrise Workbench, create a new project.
  - Better: create it by importing the project that already exists on the robot; this avoids configuration mismatch issues.
- Copy/merge the `kmr_iiwa_control/application` folder and copy the `kmr_iiwa_control/kmr_iiwa_control` folder into the `src` folder of the Sunrise project.
  - **Important**: Never create a Git repository inside the Sunrise `src` folder. Sunrise already uses Git in the project root. If another Git submodule is found, synchronization to the robot will hang indefinitely (and so will synchronization back from the robot), forcing you to manually delete project folders inside the robot.
- Copy the `log4j_*.jar` file from the `<sunrise_workbench_install_folder>/plugins` folder into the `<project_directory>/KUKAJavaLib` directory through Sunrise Workbench.
- **Mobile base sensors**: also copy `com.kuka.nav.provider-*.jar`, `com.kuka.nav.request.provider-*.jar` and `com.kuka.nav.robot.manager.provider*.jar`. I found these libraries on the mobile base PC itself, in `/opt/nav/KUKAJavaLib` (default: 172.31.1.102). I could not find them in the Sunrise plugin or anywhere else; they seem to be part of the KUKA Navigation library. Without them you can still use the robot, but you will not have access to the mobile base sensors. In that case you have to comment out the parts of the application that use them, as well as the whole `KMP_status_reader` file.
- In Sunrise Workbench, right-click the libraries copied in the previous steps and select Build > Add to Project Path. They should now appear in the `Referenced Libraries` folder.
- Check the official configuration file for IP addresses (`StationSetup.cat`, created when you create a Sunrise project).
- Check the specific configuration file in `kmr_iiwa_control/kmr_iiwa_control/config/Config.java`.
- [necessary?] Make `KMRiiwaControlApp` the default application.
- Deploy to the robot ("Synchronize Project" icon in the Workbench toolbar at the top).

## Notes

- Code taken from https://github.com/MortenMDahl/kmriiwa_ws_devel/tree/foxy/src/kmr_sunrise, but adapted because I could not figure out how to install the requirements (are those for an old Sunrise version?).
- Other resource: https://github.com/stoic-roboticist/kmriiwa_ros_java - ROS 1, but with better installation instructions that I find useful.

## Troubleshooting

### Accessing the Sunrise cabinet PC

- The default IP seems to be 172.31.1.147, but here we use 172.31.1.10. The default username is `kukauser` and the password is `kuka` (according to the internet). At the HRC lab we use a different password; check the reserved instructions for this.

### Sync failures

- Errors about a different safety (or other) configuration than the one actually present on the robot:
  try importing the project from the robot, adding the files, and then syncing. It worked for us.
- "Git submodules present": you put a Git repository in the project (or a subfolder) on the robot. I solved this by manually deleting the project folder after accessing the robot files through its IP.
- "Security configuration differs": either solve it properly, or import the project that already exists on the robot, add the files from this repo to it, and sync it back.
- "Deployment failed! Data source must not be (re)opened by clients. Error while parsing ...RoboticsAPI.data.xml! Cause: Content is not allowed in prolog.", or a similar error related to the tool library: according to the internet, this is caused by a corrupted RoboticsAPI config file. This file is regenerated when you modify `stationsetup.cat` and save the modification, so try doing that. Sometimes rebooting the robot also helped me. As a last resort, access the robot and delete the project manually as in the problem above, then reimport your backup (you do have a backup, right?).
