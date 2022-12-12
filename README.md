# Documentation for On-Rover Setup

## Setting up the NUC

Install Ubuntu (username: rover, hostname: MRT-NUC), Open-SSH, Git, ROS-Noetic and Alias python3 as python

### Setting up SSH

* Connect the NUC via wired network (LAN) to the Router
* Setup the IP Address as 192.168.2.2 and Netmask as 255.255.255.0
* Add it to /etc/hosts file for hostname resolution (required for ROS setup)
* Add the IP Address of base station as well (for example: 192.168.2.3 iitbmartian-H310M-S2)
* Reboot the NUC

### Creating Workspace

* Create a new directory `rover_ws`, create directory `src`, and run `catkin_make`
* Git pull `rover-mobility` in src, and `catkin_make`

## Roboclaws Setup

### Roboclaw Documentation
All 6 roboclaws are to be setup in packet serial modes. <br/>
* [Roboclaw User Manual](https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf)
* [Roboclaw Datasheet](http://downloads.basicmicro.com/docs/roboclaw_datasheet_2x45A.pdf)

### Locking the Roboclaws
* After powering on all the roboclaws and connecting to the NUC, use `cd /dev` to move into the dev directory and use `ls -ll` to display all the plugged in connections. Ensure there exists 6 `/ttyACM*` connections. (* - 0 to 5)
* To find the devpath for each roboclaw, use  <br/>
   `udevadm info -a -n /dev/ttyACM* | grep '{devpath}'`<br/> (do it for all * - 0 to 5)
* Make a .rules file in `/etc/udev/rules.d` starting with numbers above 50 (eg 72-\<filename\>.rules)
* The six Roboclaws are to be given the following SYMLINK names: <br/>
  Fdrive, Bdrive, Cdrive, shoulder_elbow_actuators, base_finger_motors, wrist_rotation_motors
* Current rules files is as shown, and modify parameters accordingly

`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="1.1" ,SYMLINK+="shoulder_elbow_actuators", OWNER="rover", GROUP="rover"`<br/>
`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="1.2" ,SYMLINK+="base_finger_motors", OWNER="rover", GROUP="rover"`<br/>
`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="1.3" ,SYMLINK+="wrist_rotation_motors", OWNER="rover", GROUP="rover"`<br/>
`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="2.1" ,SYMLINK+="Fdrive", OWNER="rover", GROUP="rover"`<br/>
`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="2.3" ,SYMLINK+="Bdrive", OWNER="rover", GROUP="rover"`

* Change devpath/product etc to get your device running. Don't forget to `sudo udev restart` after publishing new rules for a device

## Description of the packages

| Name of Package | Description | Subscribed rostopics                                                                            | Published rostopics       |
| --- | --- |-------------------------------------------------------------------------------------------------|---------------------------|
| rover_drive | Drive Package: Running RoboClaws Fdrive, Bdrive, Cdrive | `/rover/drive_directives/manual` `/rover/drive_directives/autonomous` `/rover/drive_directives` | `/rover/drive_directives` |
| rover_arm | Arm Package: Running RoboClaws shoulder_elbow_actuators, base_finger_motors, wrist_rotation_motors | `/rover/arm_directives`                                                                         | -                         |
| rover_light | For LED Lights | `/rover/drive_directives` `/rover/tasks_status` `/rover/light`          | `/rover/light`                  |
| rover_measure | Measuring voltage and current supply | -                                                                                               | `/rover/measure`          |

`rover_msg` contains the custom ROS message types `arm_msg` and `drive_msg`
