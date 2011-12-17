display: Android Teleop
description: Drive Maxwell from Android with a touch joystick and video feed.
platform: Maxwell
launch: maxwell_teleop/android_teleop.launch
interface: maxwell_teleop/android_teleop.interface
icon: turtlebot_teleop/android-lightning-turtlebot.png
clients:
 - type: android
   manager:
     api-level: 9
     intent-action: ros.android.teleop.Teleop
   app: 
     gravityMode: 0
     base_control_topic: /cmd_vel
