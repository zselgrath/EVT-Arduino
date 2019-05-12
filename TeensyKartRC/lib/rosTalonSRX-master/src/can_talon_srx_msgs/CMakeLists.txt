cmake_minimum_required(VERSION 2.8.3)
project(can_talon_srx_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)
add_message_files(
	FILES
	Status.msg

	Set.msg
	ConfigSetParameter.msg
	SetDemand.msg
	SetOverrideLimitSwitchEn.msg
	SetFeedbackDeviceSelect.msg
	SetRevMotDuringCloseLoopEn.msg
	SetOverrideBrakeType.msg
	SetModeSelect.msg
	SetModeSelectAndDemand.msg
	SetProfileSlotSelect.msg
	SetRampThrottle.msg
	SetRevFeedbackSensor.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime
)
