
#include "FRC_NetworkCommunication/CANSessionMux.h"
#include "ardu_can_bridge_msgs/CANData.h"
#include "ardu_can_bridge_msgs/CANSend.h"

#include <iostream>
#include <vector>
#include <map>
#include <iterator>

std::map<uint32_t, ardu_can_bridge_msgs::CANData> receivedCAN;

void CANRecvCallback(const ardu_can_bridge_msgs::CANData::ConstPtr& msg) {
	ardu_can_bridge_msgs::CANData data;
	data.arbID = msg->arbID;
	data.size = msg->size;
	data.bytes = msg->bytes;
	receivedCAN[msg->arbID] = data;
}

#ifdef __cplusplus
extern "C"
{
#endif  
	ros::Publisher CANSend_pub;
	ros::ServiceClient CANRecv_cli;
	ros::Subscriber CANRecv_sub;
	//ros::NodeHandle *n;

	void init_CANSend(ros::NodeHandle n) {
		//ros::NodeHandle privn("~");
		//std::string can_bridge_name;
		//privn.param<std::string>("can_bridge_name", can_bridge_name, "");
		//CANSend_pub = n.advertise<can_talon_srx::CANSend>(can_bridge_name + "/CANSend", 100);
		////CANRecv_cli = n.serviceClient<can_talon_srx::CANRecv>(can_bridge_name.append("CANRecv"));
		//CANRecv_sub = n.subscribe(can_bridge_name + "/CANRecv", 100, CANRecvCallback);
		//std::cout << "can_bridge_name:\t" << can_bridge_name <<std::endl;

		CANSend_pub = n.advertise<ardu_can_bridge_msgs::CANSend>("CANSend", 100);
		CANRecv_sub = n.subscribe("CANRecv", 100, CANRecvCallback);
	}

	void FRC_NetworkCommunication_CANSessionMux_sendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status) {
		ardu_can_bridge_msgs::CANSend msg;

		msg.data.arbID = messageID;
		msg.data.size = dataSize;
		std::vector<uint8_t> candata(data, data+dataSize);
		msg.data.bytes = candata;
		msg.periodMs = periodMs;
		CANSend_pub.publish(msg);

		//std::cout << "sendCAN " << messageID << "\t";
		//for(int i = 0; i < dataSize; i++) {
		//	std::cout << unsigned(data[i]) << "\t";
		//}
		//std::cout << "datsize:" << unsigned(dataSize) << "\tper:" << periodMs << std::endl;
	}

	void FRC_NetworkCommunication_CANSessionMux_receiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status) {
		//std::cout << "recvCAN " << *messageID << std::endl;
		//can_talon_srx::CANRecv srv;
		//srv.request.arbID = *messageID;

		std::map<uint32_t, ardu_can_bridge_msgs::CANData>::iterator i = receivedCAN.find(*messageID);
		if(i == receivedCAN.end()) {
			*status = 1;
		} else {
			*dataSize = receivedCAN[*messageID].size;
			*status = 0;
			std::copy(receivedCAN[*messageID].bytes.begin(), receivedCAN[*messageID].bytes.begin() + *dataSize, data);
		}



		//if(CANRecv_cli.call(srv)) {
		//	//ROS_INFO("sent CANRecv");
		//	//ROS_INFO("recv arbID: %ld", (long int)srv.response.data.arbID);
		//	*dataSize = srv.response.data.size;
		//	*status = srv.response.status;
		//	std::copy(srv.response.data.bytes.begin(), srv.response.data.bytes.begin()+*dataSize, data);
		//} else {
		//	ROS_ERROR("no CANRecv service");
		//	*status = 2;
		//}
		//for(int i = 0; i < *dataSize; i++) {
		//	std::cout << unsigned(data[i]) << "\t";
		//}
		//std::cout << "status " << *status << std::endl;
	}

//sessionHandle: set this integer to identify a session
//messageID: arbID of messages to cache
//messageIDMask: CanTalonSRX sets this to 0xFFFFFFFF, probably can ignore
//maxMessages: number of messages to cache
//status: set to zero if everything worked
	uint32_t _messageID = 0;
	void FRC_NetworkCommunication_CANSessionMux_openStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status) {
		ROS_WARN("FRC_NetworkCommunication_CANSessionMux_openStreamSession not fully implemented");
		*sessionHandle = 1; //set the session handle to something or else CanTalonSRX won't continue
		_messageID = messageID;
	}

	void FRC_NetworkCommunication_CANSessionMux_closeStreamSession(uint32_t sessionHandle) {
		ROS_WARN("FRC_NetworkCommunication_CANSessionMux_closeStreamSession not implemented");
	}

	void FRC_NetworkCommunication_CANSessionMux_readStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status) {
		ROS_WARN("FRC_NetworkCommunication_CANSessionMux_readStreamSession not fully implemented");
		uint8_t data[8];
		uint32_t timeStamp = 0;
		int32_t mystatus = 0;
		uint8_t tmpdata[8];
		uint8_t datasize;
		FRC_NetworkCommunication_CANSessionMux_receiveMessage(&_messageID, 0, &messages->data[0], &datasize, &timeStamp, &mystatus);
		messages->messageID = _messageID;
		*messagesRead = 1;
	}

	//void FRC_NetworkCommunication_CANSessionMux_getCANStatus(float *percentBusUtilization, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *receiveErrorCount, uint32_t *transmitErrorCount, int32_t *status);

#ifdef __cplusplus
}
#endif
