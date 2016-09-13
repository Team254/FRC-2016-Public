# FRC-2016
Code for 2016 FRC Robot

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

Point your browser at `file:///Users/leighpauls/pofs/robot/FRC-2016/logger/logger.html` (replacing the path with wherever your repo lives).
