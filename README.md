# Rover Mobility Packages
Repository for Rover Mobility Codes, IITB Mars Rover Team 22-23

## Roboclaws Setup

### Roboclaw Documentation
All 6 roboclaws are to be setup in packet serial modes. <br/>
* [Roboclaw User Manual](http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf)
* [Roboclaw Datasheet](http://downloads.ionmc.com/docs/roboclaw_datasheet_2x45A.pdf)

### Locking the Roboclaws
* After powering on all the roboclaws and connecting to the NUC, use `cd /dev` to move into the dev directory and use `ls -ll` to display all the plugged in connections. Ensure there exists 6 `/ttyACM*` connections. (* - 0 to 5)
* To find the devpath for each roboclaw, use  <br/>
   `udevadm info -a -n /dev/ttyACM* | grep '{devpath}'`<br/> (do it for all * - 0 to 5)
* Make a .rules file in `/etc/udev/rules.d` starting with numbers above 50 (eg 72-\<filename\>.rules)
* The six Roboclaws are to be given the following SYMLINK names: <br/>
  Ldrive, Rdrive, Cdrive, shoulder_elbow_actuators, base_finger_motors, wrist_rotation_motors
* Current rules files is as shown, and modify parameters accordingly

`ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="5.1" ,SYMLINK+="shoulder_elbow_actuators", OWNER="rover", GROUP="rover"
ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="5.2" ,SYMLINK+="base_finger_motors", OWNER="rover", GROUP="rover"
ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="5.3" ,SYMLINK+="wrist_rotation_motors", OWNER="rover", GROUP="rover"
ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="6.1" ,SYMLINK+="Ldrive", OWNER="rover", GROUP="rover"
ACTION=="add", ATTRS{product}=="USB Roboclaw 2x45A" ,ATTRS{devpath}=="6.3" ,SYMLINK+="Rdrive", OWNER="rover", GROUP="rover"`

*Change devpath/product etc to get your device running. Don't forget to `sudo udev restart` after publishing new rules for a device




