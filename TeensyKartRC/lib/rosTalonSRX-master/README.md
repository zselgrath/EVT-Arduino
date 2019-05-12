# rosTalonSRX
ROS packages for controlling a Talon SRX motor controller. The Talon SRX must be running the non-FRC firmware. FRC firmware checks for some tokens whose generation code is in a binary. Non-FRC firmware does not check for these tokens.

Uses an arduino and a seeed studio CAN-BUS Shield. Load the arduino_stuff/CANBridge sketch onto the arduino.

Start the following nodes:

    rosrun ardu_can_bridge ardu_can_bridge_node _port:=/dev/ttyACM0
    rosrun can_talon_srx can_talon_srx_node

Change /dev/ttyACM0 to whatever device the CANBridge arduino is.

This next node is for testing. It takes a joystick input and spits out a message can_talon_srx understands.

    rosrun joy joy_node
    rosrun can_talon_srx_test can_talon_srx_test_node

Talon SRX ID must be 1. It is hardcoded, along with some other stuff that should not be hardcoded.
