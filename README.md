# FRC 2016

The 2016 FRC robot code for Dropshot. Dropshot's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains the function of each package, some of the variable naming conventions used, and setup instructions. Additional information about each specific class can be found in that class's java file.

## How to Write code in IntelliJ
- Create a new directory to be the "top level" of your IntelliJ project. I call mine `~/pofs/robot`
- Check out this repo into that directory:
```
~ $ cd ~/pofs/robot/
~/pofs/robot $ git clone https://github.com/Team254/FRC-2016.git
~/pofs/robot $ ls
FRC-2016
```
- In IntelliJ, create a new empty project (not a java project, just an empty project) at your "top level". This Should create a `.idea` folder if you did it in the right spot
```
# after making the project:
~/pofs/robot $ ls -a
.  ..  .idea  FRC-2016
```
- In intelliJ, you should be in the "project settings" window. Create a new module from existing sources on the `FRC-2016` folder, IntelliJ should pick up `src/` as the content root.
- In the project settings window, add a new Java library for wiplib. Select all the jars in `~/wpilib/java/current/libs/`. It'll give it a wonky name like "networktables", but that doesn't matter. Choose to include it in the `FRC-2016` project.
- In project settings, under the "prject section" set your JDK to Java 1.8
- You can now write code with auto-complete in IntelliJ, but not build/deploy
- In IntelliJ, in the "ant build" pane, add `FRC-2016/build.xml`. To deploy code to the robot, double click `athena-project-build.build`

## How to run CheesyLogger

You'll need to install Mosquitto on your machine. On osx that's:

```
$ brew install mosquitto
```

There's probably a compatible apt-get package for linux (I haven't looked at time of writing).

In CheesyLogger.java, change the mqtt server url to be your laptop. You can use your IP, or your mDNS record. Mine is `tcp://leighpauls-mbp.synergy.bcp.org:1883`.

Run the mqtt broker server:
```
~/pofs/robot/FRC-2016 $ ./logger/run_mqtt_server.sh
1453174940: mosquitto version 1.4.5 (build date 2015-11-09 14:23:18-0800) starting
1453174940: Config loaded from ./logger/mosquitto.conf.
1453174940: Opening ipv4 listen socket on port 1883.
1453174940: Opening ipv6 listen socket on port 1883.
1453174940: Opening websockets listen socket on port 11883.
```

Point your browser at `file:///Users/*YOUR_USERNAME*/pofs/robot/FRC-2016/logger/logger.html` (replacing the path with wherever your repo lives).

##Package Functions
- com.team254.frc2016

	Contains the robot's central functions and holds a file with all numerical constants used throughout the code. For example, the Robot member class controls all routines depending on the robot state.

- com.team254.frc2016.auto

	Controls the robot's autonomous routine for the autonomous state. See the auto.modes package for the autonomous routines.
	
- com.team254.frc2016.auto.actions

	Contains all actions used during the autonomous period, which all share a common interface, Action (also in this package). Examples include getting the hood low.Routines interact with the Subsystems, which interact with the hardware.
	
- com.team254.frc2016.auto.modes
	
	Contains all autonomous routines. Autonomous routines typically move the robot in their class and call one or more Actions.
	
- com.team254.frc2016.loops

	Loops are routines that run periodically on the robot, such as calibrating the gyroscope and checking the turret's limit switches. This runs at 100 Hz. All Loops implement the Loop interface and are handled (started, stopped, added) by the Looper class.
	The Robot class has two Loopers, mEnabledLooper and mDisabledLooper, which run when the robot is enabled and disabled, respectively. The enabled and disabled states require different Loops to be run, thus the need for two Loopers.
	
- com.team254.frc2016.subsystems
	
	Subsystems are consolidated into one central class per subsystem, all of which implement the Subsystem abstract class. Each Subsystem uses enums, whether at the local or global scope, to set one of multiple states. For example, the HoodRoller has "Stop", "Intake", "Reverse", and "Shoot" states.
	There is only one instance of each Subsystem class (one cannot have two Drivebases on one robot or in the code). To modify a subsystem, one would get the instance of the subsystem and change its desired state. The Subsystem class will work on setting the desired state.
	
- com.team254.frc2016.vision

	Handles the Android vision tracking system. This includes handling all ADB (Android Debug Bridge) communications with the phone and creating VisionUpdate data with the data from the phone.
	VisionUpdates consist of TargetInfo objects (the target's coordinates in 3D space), a timestamp, and a "valid" value (if the update is valid). This represents the target data from each frame processed by the phone.
	The VisionServer class unifies the vision system. Like the Subsystems, there is only one instance of the VisionServer.

- com.team254.frc2016.vision.messages

	Contains messages used by the vision system: a heartbeat signal that's regularly sent out and a "camera mode" message that contains information about the camera's state.
	All Messages implement the VisionMessage abstract class.
	
- com.team254.lib.util

	A collection of assorted utilities used in the robot code. This includes custom classes for hardware devices (encoders, gyroscopes, etc.) as well as mathematical helper functions, especially regarding translations and rotations. Check each .java file for more information.

## Variable Naming Conventions

- k_*** (i.e. kMinHoodAngle)    : Final constants, especialy those found in the Constants.java file
- K_*** (i.e. K_VISION_MODE)    : Static constants
- m***  (i.e. mStartingDistance): Private instance variables
