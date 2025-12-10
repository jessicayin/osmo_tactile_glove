## Installation
If not already installed, install the `example_interfaces` ROS2 package:

```
git clone -b humble git@github.com:ros2/example_interfaces.git
```
into this workspace's `src` folder.

Then run `./colcon_build.sh`.

Remember to source the workspace afterwards.

`source ./install/setup.bash`

## Run

```
#check which port the hand is connected to
ls /dev/tty*

#add permission to modify port
sudo chmod 666 /dev/ttyUSB0

#TODO: add command line input for COM port config. default is /dev/ttyUSB0

#run the node
ros2 run inspire_hand Hand_control_node
```

## Info
#TODO: verify functionality of these commands

(1)ros2 service call /Setpos service_interfaces/srv/Setpos "{pos0: num0,pos1: num2,pos2: num2,pos3: num3,pos4: num4,pos5: num5,hand_id: 1, status: 'set_pos'}"
Set six drive positions------parameter num0-num5 range 0-2000, default hand ID is 1

(2)ros2 service call /Setangle service_interfaces/srv/Setangle "{angle0: num0,angle1: num2,angle2: num2,angle3: num3,angle4: num4,angle5: num5,hand_id: 1, status: 'set_angle'}"
Set hand angle------parameter num0-num5 range -1-1000

(3)ros2 service call /Setforce service_interfaces/srv/Setforce "{force0: num0,force1: num2,force2: num2,force3: num3,force4: num4,force5: num5,hand_id: 1, status: 'set_force'}"
Set force control threshold ------ parameter num0-num5 range -1000

(4) ros2 service call /Setspeed service_interfaces/srv/Setspeed "{speed0: num0,speed1: num2,speed2: num2,speed3: num3,speed4: num4,speed5: num5,hand_id: 1, status: 'set_speed'}"
Set speed ------ parameter num0-num5 range -1000

(5) ros2 service call /Getposact service_interfaces/srv/Getposact "{hand_id: 1, status: 'get_posact'}"
Read the actual position value of the drive

(6) ros2 service call /Getangleact service_interfaces/srv/Getangleact "{hand_id: 1, status: 'get_angleact'}"
Read the actual angle value

(7)ros2 service call /Getforceact service_interfaces/srv/Getforceact "{hand_id: 1, status: 'get_forceact'}"
Read the actual force

(8)ros2 service call /Getposset service_interfaces/srv/Getposset "{hand_id: 1, status: 'get_posset'}"
Read the position value set by the driver

(9)ros2 service call /Getangleset service_interfaces/srv/Getangleset "{hand_id: 1, status: 'get_angleset'}"
Read the set angle value

(10)ros2 service call /Getforceset service_interfaces/srv/Getforceset "{hand_id: 1, status: 'get_forceset'}"
Read the set force control threshold

(11)ros2 service call /Geterror service_interfaces/srv/Geterror "{hand_id: 1, status: 'get_error'}"
Read fault information

(12)ros2 service call /Getspeedset service_interfaces/srv/Getspeedset "{hand_id: 1, status: 'get_speedset'}"
Read the set speed

(13)ros2 service call /Gettemp service_interfaces/srv/Gettemp "{hand_id: 1, status: 'get_temp'}"
Read temperature information

(14)ros2 service call /Getcurrentact service_interfaces/srv/Getcurrentact "{hand_id: 1, status: 'get_currentact'}"
Read current

(15)ros2 service call /Setgestureno service_interfaces/srv/Setgestureno "{gesture_no: 1,hand_id: 1, status: 'set_gesture_no'}"
Gesture sequence, gesture_no range is 1-13
