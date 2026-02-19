# Circuit Design
<img width="700" height="700" alt="image" src="https://github.com/user-attachments/assets/1a6f24a3-88ef-41a5-b630-a91bfdca63ae" />


# Arduino Motor Controller

This code turns an Arduino into a motor controller!
It provides a simple serial interface to communicate with a high-level computer (e.g. running ROS), and generates the appropriate PWM signals for a motor driver, to drive two motors.

This is a fork of the original code, with some changes, and removal of the ROS nodes (see [this repo](https://github.com/joshnewans/serial_motor_demo) for an alternative). Check out `README-orig.md` for the original README.

As I only have need for a subset of the functionality, I have no idea what does and doesn't work, beyond what is detailed below.
Feedback/improvements are welcome (though no promises on how quickly I'll respond). I currently only use the L298N driver, and the Arduino encoder mode.


## Functionality

The main functionality provided is to receive motor speed requests over a serial connection, and provide encoder feedback.
The original code has provisions for other features - e.g. read/write of digital/analog pins, servo control, but I've never used them.

The main commands to know are

- `e` - Motor responds with current encoder counts for each motor
- `r` - Reset encoder values
- `o <PWM1> <PWM2>` - Set the raw PWM speed of each motor (-255 to 255)
- `m <Spd1> <Spd2>` - Set the closed-loop speed of each motor in *counts per loop* (Default loop rate is 30, so `(counts per sec)/30`
- `p <Kp> <Kd> <Ki> <Ko>` - Update the PID parameters

## Code to run on robot
```
ros2 run serial_motor_demo driver --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud_rate:=57600 \
  -p encoder_cpr:=600 \
  -p loop_rate:=30
```


## Code to run on robot
```
ros2 run serial_motor_demo gui
```

## Gotchas

Some quick things to note

- There is an auto timeout (default 2s) so you need to keep sending commands for it to keep moving
- PID parameter order is PDI (?)
- Motor speed is in counts per loop
- Default baud rate 57600
- Needs carriage return (CR)
- Make sure serial is enabled (user in dialout group)
- Check out the original readme for more


## TODO (maybe)
- Document PID tuning
- Make the speed input counts per second
- Add/test more driver boards
- Add/test other functionality
