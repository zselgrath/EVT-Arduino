#include "ctre/PDP.h"
#include <iostream>
#include "ros/ros.h"
#include <vector>
#include "FRC_NetworkCommunication/CANSessionMux.h"
#include <memory>
#include "can_pdp_msgs/PDP.h"
#include "std_msgs/Empty.h"

PDP* pdp;

void clearStickyFaultsCallback(const std_msgs::Empty::ConstPtr& msg) {

}

void resetEnergyCallback(const std_msgs::Empty::ConstPtr& msg) {

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pdp");
	ros::NodeHandle n;
	
	ros::Publisher pdp_pub = n.advertise<can_pdp_msgs::PDP>("pdp", 10);
	ros::Subscriber clearStickyFaults_sub = n.subscribe("clearStickyFaults", 1, clearStickyFaultsCallback);
	ros::Subscriber resetEnergy_sub = n.subscribe("resetEnergy", 1, resetEnergyCallback);

	init_CANSend(n);

	pdp = new PDP(0);

	ros::Rate loop_rate(20);

	while(ros::ok()) {
		can_pdp_msgs::PDP pdp_msg;

		for(int i = 0; i < 16; i++) {
			pdp->GetChannelCurrent(i, pdp_msg.Current[i]);
		}
		pdp->GetVoltage(pdp_msg.Voltage);
		pdp->GetTemperature(pdp_msg.Temperature);
		pdp->GetTotalCurrent(pdp_msg.TotalCurrent);
		pdp->GetTotalPower(pdp_msg.TotalPower);
		pdp->GetTotalEnergy(pdp_msg.TotalEnergy);

		pdp_pub.publish(pdp_msg);

		loop_rate.sleep();
	}
}
