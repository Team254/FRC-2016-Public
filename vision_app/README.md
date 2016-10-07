# CheezDroid
Android-based Vision System


## To provision a device for robot use
* Enable device admin
1. Settings App > Security > Device Administrators > Click box 'on' for CheezDroid

* Enable device owner
1. adb shell
2. dpm set-device-owner com.team254.cheezdroid/.ChezyDeviceAdminReceiver

## How to Install ADB on the RoboRIO

Download and run the script from [here] (https://github.com/Team254/FRC-2016-Public/blob/master/installation/install.osx.sh). Note that this script has only been tested on Mac OS X; it hasn't been tested on Windows or Linux.
