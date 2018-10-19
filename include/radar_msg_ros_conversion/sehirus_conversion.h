#ifndef SEHIRUS_CONVERSION_H
#define SEHIRUS_CONVERSION_H

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <pthread.h>
#include "ros/ros.h"
#include "radar_msg_ros_conversion/SehirusHeartbeat.h"

typedef boost::asio::ip::udp boostUdp;
typedef boost::asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO> rcv_timeout_option;

class SehirusConversion {
public:
	SehirusConversion();
	~SehirusConversion();
	void main();
private:
	ros::NodeHandle _nodeHandle;
	ros::Publisher	_heartBeatPublisher;
};



#endif /* SEHIRUS_CONVERSION_H */