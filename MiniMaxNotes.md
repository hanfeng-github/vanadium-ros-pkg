# Introduction #
Mini Max is a custom mobile manipulator, leveraging the ArbotiX2, Dynamixel Servos, ROS, and an iRobot Create. The platform is open source.

<img src='https://vanadium-ros-pkg.googlecode.com/svn/wiki/watercolor.png' width='300'>

<h2>Parts List</h2>
<a href='https://spreadsheets.google.com/spreadsheet/pub?hl=en_US&hl=en_US&key=0AoEGbFgI5kOhdG9fRkFTVnc5VXkyZ0JYcW5laDE3NkE&single=true&gid=5&output=html'>Parts List</a> <i>(DRAFT)</i>

<h2>Arm Details</h2>
<pre><code>arm_shoulder_pan_joint id = 1<br>
arm_shoulder_lift_joint id = 2<br>
arm_elbow_flex_joint id = 3<br>
arm_wrist_flex_joint id = 4<br>
gripper_joint id = 5<br>
</code></pre>

<h2>Head details</h2>
<pre><code>pan servo id = 6<br>
tilt servo id = 7<br>
</code></pre>

<h2>Using Mini Maxwell</h2>
Has been moved to <a href='http://www.ros.org/wiki/mini_max/Tutorials'>Mini Maxwell Tutorials</a> on the ROS wiki.<br>
<br>
<h2>Software Roadmap</h2>
The following need to be addressed:<br>
<ul><li>mini_max_apps<br>
<ul><li>tuck_arm needs to be updated<br>
</li><li>create launch file for nav</li></ul></li></ul>

<h2>Documentation Roadmap</h2>
<ul><li>How to build, wire robot<br>
<ul><li>Installing Power Board<br>
</li><li><del>Setting servo IDs</del>
</li><li>Assembling the Frame<br>
</li><li>Assembling, Wiring the Arm<br>
</li><li>Attaching the Kinect</li></ul></li></ul>

<ul><li>Basic tutorials (on ROS.org):<br>
<ul><li>(DONE) How to setup laptop -- covers installation of packages.<br>
</li><li>(DONE) Starting up Mini Maxwell -- covers bringup.<br>
</li><li>(DONE) Using the Buttons -- covers reading the state of the switches.<br>
</li><li>Moving the base -- shows how to move the base by publishing cmd_vel. Students draw a square on the ground. Advanced: using TF to make it a nice square.<br>
</li><li>Pointing the Head -- shows how to compute joint angles to look at a point in space.<br>
</li><li>Moving the Arm -- shows how to use the simple_arm_server, move the arm, open/close the gripper.<br>
</li><li>Using the Camera -- shows how to capture image, do cv_bridge, and then blur the image.</li></ul></li></ul>

<ul><li>Advanced tutorials (on ROS.org):<br>
<ul><li>Working with Point Clouds (C++) -- shows how to capture point clouds with the Kinect. Shows perception aspects of block_manipulation.<br>
</li><li>Picking up Objects (C++) -- builds on perception of block_manipulation, picks up a recognized block.</li></ul></li></ul>

<h2>Hardware Revision History</h2>
Future Work:<br>
Laser cut, parallel jaw gripper<br>
<br>
Revision History:<br>
<ul><li>V0.2 Revisions (Siena batch)<br>
<ul><li>Add holes for ArbotiX mounting<br>
</li><li>Add right angle connectors on front upright, adjoining side<br>
</li><li>New Kinect mounting pieces<br>
</li><li>Add holes in base for wire pass through<br>
</li></ul></li><li>V0.3 Revisions (Fergy's)<br>
<ul><li>Add base ridge for increased stability of arm<br>
</li><li>Holes in base are now rounded squares (better USB cable access)<br>
</li></ul></li><li>V0.4 Revisions (HMC?)<br>
<ul><li>Move left-most Kinect mount just a bit for better alignment.<br>
</li></ul></li><li>V0.5 Revisions (Forthcoming)<br>
<ul><li>Add USB slots (HMC upgrade)