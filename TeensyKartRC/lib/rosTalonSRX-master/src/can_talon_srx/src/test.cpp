#include "HAL/CanTalonSRX.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include "FRC_NetworkCommunication/CANSessionMux.h"
#include <memory>

#include "can_talon_srx/GetParameter.h"

#include "can_talon_srx_msgs/Set.h"
#include "can_talon_srx_msgs/Status.h"
#include "can_talon_srx_msgs/ConfigSetParameter.h"

#include "can_talon_srx_msgs/SetDemand.h"
#include "can_talon_srx_msgs/SetOverrideLimitSwitchEn.h"
#include "can_talon_srx_msgs/SetFeedbackDeviceSelect.h"
#include "can_talon_srx_msgs/SetRevMotDuringCloseLoopEn.h"
#include "can_talon_srx_msgs/SetOverrideBrakeType.h"
#include "can_talon_srx_msgs/SetModeSelect.h"
#include "can_talon_srx_msgs/SetModeSelectAndDemand.h"
#include "can_talon_srx_msgs/SetProfileSlotSelect.h"
#include "can_talon_srx_msgs/SetRampThrottle.h"
#include "can_talon_srx_msgs/SetRevFeedbackSensor.h"

CanTalonSRX* motor;

void setCallback(const can_talon_srx_msgs::Set::ConstPtr& control) {
	motor->Set(control->set);
}

void configSetParameterCallback(const can_talon_srx_msgs::ConfigSetParameter::ConstPtr& msg) {
	motor->SetParam((CanTalonSRX::param_t) msg->param, msg->value);
}



void setDemandCallback(const can_talon_srx_msgs::			SetDemand::ConstPtr& msg) {
	motor->SetDemand(msg->param);
}
void setOverrideLimitSwitchEnCallback(const can_talon_srx_msgs::        SetOverrideLimitSwitchEn::ConstPtr& msg) {
	motor->SetOverrideLimitSwitchEn(msg->param);
}
void setFeedbackDeviceSelectCallback(const can_talon_srx_msgs::         SetFeedbackDeviceSelect::ConstPtr& msg) {
	motor->SetFeedbackDeviceSelect(msg->param);
}
void setRevMotDuringCloseLoopEnCallback(const can_talon_srx_msgs::      SetRevMotDuringCloseLoopEn::ConstPtr& msg) {
	motor->SetRevMotDuringCloseLoopEn(msg->param);
}
void setOverrideBrakeTypeCallback(const can_talon_srx_msgs::            SetOverrideBrakeType::ConstPtr& msg) {
	motor->SetOverrideBrakeType(msg->param);
}
void setModeSelectCallback(const can_talon_srx_msgs::                   SetModeSelect::ConstPtr& msg) {
	motor->SetModeSelect(msg->param);
}
void setModeSelectAndDemandCallback(const can_talon_srx_msgs::          SetModeSelectAndDemand::ConstPtr& msg) {
	motor->SetModeSelect(msg->param, msg->demand);
}
void setProfileSlotSelectCallback(const can_talon_srx_msgs::            SetProfileSlotSelect::ConstPtr& msg) {
	motor->SetProfileSlotSelect(msg->param);
}
void setRampThrottleCallback(const can_talon_srx_msgs::                 SetRampThrottle::ConstPtr& msg) {
	motor->SetRampThrottle(msg->param);
}
void setRevFeedbackSensorCallback(const can_talon_srx_msgs::            SetRevFeedbackSensor::ConstPtr& msg) {
	motor->SetRevFeedbackSensor(msg->param);
}


bool getParameter(can_talon_srx::GetParameter::Request &req, can_talon_srx::GetParameter::Response &res) {
	motor->RequestParam((CanTalonSRX::param_t)req.param);
	ros::Duration(0.04).sleep();
	motor->GetParamResponse((CanTalonSRX::param_t)req.param, res.value);
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "talontest");
	ros::NodeHandle n;

	ros::Publisher CANSend_pub;// = n.advertise<can_talon_srx::CANSend>("CANSend",100);
	ros::ServiceClient CANRecv_cli;// = n.serviceClient<can_talon_srx::CANRecv>("CANRecv");
	ros::Publisher status_pub = n.advertise<can_talon_srx_msgs::Status>("status", 10);

	init_CANSend(n);
	ros::NodeHandle privn("~");
	int talonNumber;
	privn.param("talon_number", talonNumber, 1);
	motor = new CanTalonSRX(talonNumber);
	std::cout << talonNumber << std::endl;
	
	ros::ServiceServer getParameter_srv = n.advertiseService("getParameter", getParameter);

	ros::Subscriber set_sub = n.subscribe("set", 1, setCallback);
	ros::Subscriber ConfigSetParameter_sub = n.subscribe("configSetParameter", 10, configSetParameterCallback);

	ros::Subscriber setDemandCallback_sub = n.subscribe("setDemand", 10, setDemandCallback);
	ros::Subscriber setOverrideLimitSwitchEnCallback_sub = n.subscribe("setOverrideLimitSwitchEn", 10, setOverrideLimitSwitchEnCallback);
	ros::Subscriber setFeedbackDeviceSelectCallback_sub = n.subscribe("setFeedbackDeviceSelect", 10, setFeedbackDeviceSelectCallback);
	ros::Subscriber setRevMotDuringCloseLoopEnCallback_sub = n.subscribe("setRevMotDuringCloseLoopEn", 10, setRevMotDuringCloseLoopEnCallback);
	ros::Subscriber setOverrideBrakeTypeCallback_sub = n.subscribe("setOverrideBrakeType", 10, setOverrideBrakeTypeCallback);
	ros::Subscriber setModeSelectCallback_sub = n.subscribe("setModeSelect", 10, setModeSelectCallback);
	ros::Subscriber setModeSelectAndDemandCallback_sub = n.subscribe("setModeSelectAndDemandCallback", 10, setModeSelectAndDemandCallback);
	ros::Subscriber setProfileSlotSelectCallback_sub = n.subscribe("setProfileSlotSelectCallback", 10, setProfileSlotSelectCallback);
	ros::Subscriber setRampThrottleCallback_sub = n.subscribe("setRampThrottleCallback", 10, setRampThrottleCallback);
	ros::Subscriber setRevFeedbackSensorCallback_sub = n.subscribe("setRevFeedbackSensorCallback", 10, setRevFeedbackSensorCallback);

	// setDemandCallback
	// setOverrideLimitSwitchEnCallback
	// setFeedbackDeviceSelectCallback
	// setRevMotDuringCloseLoopEnCallback
	// setOverrideBrakeTypeCallback
	// setModeSelectCallback
	// setModeSelectAndDemandCallback
	// setProfileSlotSelectCallback
	// setRampThrottleCallback
	// setRevFeedbackSensorCallback

	ros::Rate loop_rate(100); //in hertz

	ros::AsyncSpinner spinner(1);
	spinner.start();


	double dparam; //variable declaration needs to be outside or else segfault
	int param;
	while(ros::ok()) {
		can_talon_srx_msgs::Status status_msg;

		motor->GetFault_OverTemp(param);
		status_msg.Fault_OverTemp = param;
		motor->GetFault_UnderVoltage(param);
		status_msg.Fault_UnderVoltage = param;
		motor->GetFault_ForLim(param);
		status_msg.Fault_ForLim = param;
		motor->GetFault_RevLim(param);
		status_msg.Fault_RevLim = param;
		motor->GetFault_HardwareFailure(param);
		status_msg.Fault_HardwareFailure = param;
		motor->GetFault_ForSoftLim(param);
		status_msg.Fault_ForSoftLim = param;
		motor->GetFault_RevSoftLim(param);
		status_msg.Fault_RevSoftLim = param;
		motor->GetStckyFault_OverTemp(param);
		status_msg.StckyFault_OverTemp = param;
		motor->GetStckyFault_UnderVoltage(param);
		status_msg.StckyFault_UnderVoltage = param;
		motor->GetStckyFault_ForLim(param);
		status_msg.StckyFault_ForLim = param;
		motor->GetStckyFault_RevLim(param);
		status_msg.StckyFault_RevLim = param;
		motor->GetStckyFault_ForSoftLim(param);
		status_msg.StckyFault_ForSoftLim = param;
		motor->GetStckyFault_RevSoftLim(param);
		status_msg.StckyFault_RevSoftLim = param;
		motor->GetAppliedThrottle(param);
		status_msg.AppliedThrottle = param;
		motor->GetCloseLoopErr(param);
		status_msg.CloseLoopErr = param;
		motor->GetFeedbackDeviceSelect(param);
		status_msg.FeedbackDeviceSelect = param;
		motor->GetModeSelect(param);
		status_msg.ModeSelect = param;
		motor->GetLimitSwitchEn(param);
		status_msg.LimitSwitchEn = param;
		motor->GetLimitSwitchClosedFor(param);
		status_msg.LimitSwitchClosedFor = param;
		motor->GetLimitSwitchClosedRev(param);
		status_msg.LimitSwitchClosedRev = param;
		motor->GetSensorPosition(param);
		status_msg.SensorPosition = param;
		motor->GetSensorVelocity(param);
		status_msg.SensorVelocity = param;
		motor->GetCurrent(dparam);
		status_msg.Current = dparam;
		motor->GetBrakeIsEnabled(param);
		status_msg.BrakeIsEnabled = param;
		motor->GetEncPosition(param);
		status_msg.EncPosition = param;
		motor->GetEncVel(param);
		status_msg.EncVel = param;
		motor->GetEncIndexRiseEvents(param);
		status_msg.EncIndexRiseEvents = param;
		motor->GetQuadApin(param);
		status_msg.QuadApin = param;
		motor->GetQuadBpin(param);
		status_msg.QuadBpin = param;
		motor->GetQuadIdxpin(param);
		status_msg.QuadIdxpin = param;
		motor->GetAnalogInWithOv(param);
		status_msg.AnalogInWithOv = param;
		motor->GetAnalogInVel(param);
		status_msg.AnalogInVel = param;
		motor->GetTemp(dparam);
		status_msg.Temp = dparam;
		motor->GetBatteryV(dparam);
		status_msg.BatteryV = dparam;
		motor->GetResetCount(param);
		status_msg.ResetCount = param;
		motor->GetResetFlags(param);
		status_msg.ResetFlags = param;
		motor->GetFirmVers(param);
		status_msg.FirmVers = param;
		motor->GetPulseWidthPosition(param);
		status_msg.PulseWidthPosition = param;
		motor->GetPulseWidthVelocity(param);
		status_msg.PulseWidthVelocity = param;
		motor->GetPulseWidthRiseToFallUs(param);
		status_msg.PulseWidthRiseToFallUs = param;
		motor->GetPulseWidthRiseToRiseUs(param);
		status_msg.PulseWidthRiseToRiseUs = param;
		motor->IsPulseWidthSensorPresent(param);
		status_msg.IsPulseWidthSensorPresent = param;

		status_pub.publish(status_msg);

		loop_rate.sleep();
	}
}
