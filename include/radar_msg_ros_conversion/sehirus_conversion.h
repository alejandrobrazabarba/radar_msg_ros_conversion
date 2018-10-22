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
#include "radar_msg_ros_conversion/SehirusBasicTrackReport.h"
#include "radar_msg_ros_conversion/SehirusNormalTrackReport.h"
#include "radar_msg_ros_conversion/SehirusExtendedTrackReport.h"

typedef boost::asio::ip::udp boostUdp;
typedef radar_msg_ros_conversion::SehirusHeartbeat Heartbeat;
typedef radar_msg_ros_conversion::SehirusBasicTrackReport BasicTrackReport;
typedef radar_msg_ros_conversion::SehirusNormalTrackReport NormalTrackReport;
typedef radar_msg_ros_conversion::SehirusExtendedTrackReport ExtendedTrackReport;


#define HEARTBEATLISTENINGPORT 6500
#define TRACKREPORTLISTENINGPORT 6501
//#define BASICTRACKREPORTSIZE 76 // bytes
//#define NORMALTRACKREPORTSIZE 152 // bytes
//#define EXTENDEDTRACKREPORTSIZE 188 // bytes
#define FLOATSIZE 4

class SehirusConversion {
public:
	SehirusConversion();
	~SehirusConversion();
	void main();
private:
	void udp_handle_receive_heartbeat(const boost::system::error_code& error,
									  std::size_t num_bytes);
	void udp_handle_receive_trackreport(const boost::system::error_code& error,
									  	std::size_t num_bytes);

	ros::NodeHandle _nodeHandle;
	ros::Publisher	_heartBeatPublisher;
	ros::Publisher  _basicTrackReportPublisher;
	ros::Publisher  _normalTrackReportPublisher;
	ros::Publisher  _extendedTrackReportPublisher;
	//boost::array<uint8_t, 1> _sendBuf;
	boost::array<uint8_t, 72> _heartbeatRecvBuf;
	boost::array<uint8_t, 188> _trackReportRecvBuf;
	boost::asio::io_service	_io_service;
	boostUdp::endpoint _server_endpoint;
	boostUdp::socket _heartbeatSocket;
	boostUdp::socket _trackReportSocket;
};



#endif /* SEHIRUS_CONVERSION_H */