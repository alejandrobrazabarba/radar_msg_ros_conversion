#ifndef SEHIRUS_CONVERSION_H
#define SEHIRUS_CONVERSION_H

#include <iostream>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <pthread.h>
#include "ros/ros.h"
#include "radar_msg_ros_conversion/SehirusHeartbeat.h"

typedef boost::asio::ip::udp boostUdp;

class SehirusConversion {
public:
	SehirusConversion();
	~SehirusConversion();
	void main();
private:
	void udp_handle_receive(const boost::system::error_code& error,
							std::size_t num_bytes);

	ros::NodeHandle _nodeHandle;
	ros::Publisher	_heartBeatPublisher;
	boost::array<uint8_t, 1> _send_buf;
	boost::array<uint8_t, 72> _recv_buf;
	boost::asio::io_service	_io_service;
	boostUdp::endpoint _server_endpoint;
	boostUdp::socket _socket;
};



#endif /* SEHIRUS_CONVERSION_H */