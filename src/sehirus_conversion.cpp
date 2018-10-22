#include "radar_msg_ros_conversion/sehirus_conversion.h"

// Example: How to access SehirusHeartbeat.msg
/*
radar_msg_ros_conversion::SehirusHeartbeat heartBeatMsg;
heartBeatMsg.msg_id
heartBeatMsg.command_listen
*/

/**
* SehirusConversion constructor
*/
SehirusConversion::SehirusConversion():
_nodeHandle(),
_heartbeatSocket(_io_service, boostUdp::endpoint(boostUdp::v4(), HEARTBEATLISTENINGPORT)),
_trackReportSocket(_io_service, boostUdp::endpoint(boostUdp::v4(), TRACKREPORTLISTENINGPORT))
{
}

/**
* SehirusConversion destructor
*/
SehirusConversion::~SehirusConversion()
{

}

void SehirusConversion::udp_handle_receive_heartbeat(const boost::system::error_code& error,
										   std::size_t num_bytes)
{
	if (!error)
	{
		// Heartbeat Message Size: 72 bytes
		ROS_INFO("Heartbeat Message Received");
		radar_msg_ros_conversion::SehirusHeartbeat heartBeatMsg;

		uint32_t msgIdentifier = (uint32_t)_heartbeatRecvBuf[3] << 24  |
      				  				 (uint32_t)_heartbeatRecvBuf[2] << 16 |
      				  				 (uint32_t)_heartbeatRecvBuf[1] << 8  |
      				  				 (uint32_t)_heartbeatRecvBuf[0];

      	float aux_float = 0;
      	// Heartbeat message
      	if ( (msgIdentifier == 0x01AB0101) && (num_bytes == Heartbeat::UDPMSGSIZE) )  
      	{
			/*
			//std::cout.write(_recv_buf.data(), len);
			//std::cout << "Something is received" << std::endl;
			// Si recibimos los datos en formato big-endian
			/*
			num = (uint32_t)buffer[0] << 24 |
      		  	(uint32_t)buffer[1] << 16 |
      	  		(uint32_t)buffer[2] << 8  |
      	  		(uint32_t)buffer[3];
    		// Si recibimos los datos en formato little-endian
			num = (uint32_t)buffer[3] << 24 |
      		  		(uint32_t)buffer[2] << 16 |
      		  		(uint32_t)buffer[1] << 8  |
      	  	  		(uint32_t)buffer[0];
			*/

    		char serverName[Heartbeat::SERVERNAMESIZE];
    		for(int i=0; i<=Heartbeat::SERVERNAMESIZE; i++) {
      			serverName[i] = _heartbeatRecvBuf[Heartbeat::SERVERNAMEPOS+i]; 
    		}
			heartBeatMsg.serverName = std::string(serverName);
			heartBeatMsg.commandPort = (uint16_t) _heartbeatRecvBuf[Heartbeat::CMDPORTPOS+1] << 8 |
								   		(uint16_t) _heartbeatRecvBuf[Heartbeat::CMDPORTPOS];
			heartBeatMsg.commandConnected = (uint8_t) _heartbeatRecvBuf[Heartbeat::CMDCONNECTEDPOS];
			heartBeatMsg.commandListen = (uint8_t) _heartbeatRecvBuf[Heartbeat::CMDLISTENPOS];
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::RAWDISTTPUTPOS, FLOATSIZE);
			heartBeatMsg.rawDistributionThroughput = aux_float;
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::PRODISTTPUTPOS, FLOATSIZE);
			heartBeatMsg.proDistributionThroughput = aux_float;
			std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::RAWRECTPUTPOS, FLOATSIZE);
			heartBeatMsg.rawRecordingThroughput = aux_float;
			heartBeatMsg.sourceStarted = (uint8_t) _heartbeatRecvBuf[Heartbeat::SRCSTARTEDPOS];
			heartBeatMsg.bufferFull = (uint8_t) _heartbeatRecvBuf[Heartbeat::BUFFERFULLPOS];
			heartBeatMsg.cpuLoad = (uint16_t) _heartbeatRecvBuf[Heartbeat::CPULOADPOS+1] << 8 |
								   (uint16_t) _heartbeatRecvBuf[Heartbeat::CPULOADPOS];
			heartBeatMsg.currentSourcePeriod = (uint16_t) _heartbeatRecvBuf[Heartbeat::CURRENTSRCPERIODPOS+1] << 8 |
								   			   (uint16_t) _heartbeatRecvBuf[Heartbeat::CURRENTSRCPERIODPOS];
			heartBeatMsg.nScans = (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+3] << 24  |
      				  			  (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+2] << 16 |
      				  			  (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS+1] << 8  |
      				  			  (uint32_t)_heartbeatRecvBuf[Heartbeat::NSCANSPOS];
			heartBeatMsg.navDataPresent = (uint8_t) _heartbeatRecvBuf[Heartbeat::NAVDATAPRESENTPOS];
			// std::memcpy ( destino, origen, tamaño)
      		std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::LATITUDEPOS, FLOATSIZE);
      		heartBeatMsg.latitude = aux_float;
      		std::memcpy (&aux_float, _heartbeatRecvBuf.data() + Heartbeat::LONGITUDEPOS, FLOATSIZE);
			heartBeatMsg.longitude = aux_float;

			_heartBeatPublisher.publish(heartBeatMsg);
      	} else {
      		ROS_WARN("Incorrect Heartbeat Message Reception");
      	}
    }
	_heartbeatSocket.async_receive_from(
			boost::asio::buffer(_heartbeatRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_heartbeat, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
}

void SehirusConversion::udp_handle_receive_trackreport(const boost::system::error_code& error,
										   			   std::size_t num_bytes)
{
	// Basic Track Report size: 76 bytes
	// Normal Track Report size: 152 bytes
	// Extended Track Report size: 188 bytes

	if (!error)
	{
		radar_msg_ros_conversion::SehirusBasicTrackReport trackMsg;

		uint32_t msgIdentifier = (uint32_t)_trackReportRecvBuf[3] << 24  |
      				  				 (uint32_t)_trackReportRecvBuf[2] << 16 |
      				  				 (uint32_t)_trackReportRecvBuf[1] << 8  |
      				  				 (uint32_t)_trackReportRecvBuf[0];

      	float aux_float = 0;

      	if ( (msgIdentifier == 0x01AD0101) && (num_bytes == BasicTrackReport::UDPMSGSIZE) )
      	{
      		// Basic track report
      		ROS_INFO_STREAM("Basic Track Report Received with size: " << num_bytes);
      		radar_msg_ros_conversion::SehirusBasicTrackReport basicTrackMsg;
      		trackMsg.trackID = (uint32_t)_trackReportRecvBuf[5] << 8  |
      				  			(uint32_t)_trackReportRecvBuf[4];

      		std::memcpy (&aux_float, _trackReportRecvBuf.data() + 16, 4);
      		basicTrackMsg.estX = aux_float;
      		std::memcpy (&aux_float, _trackReportRecvBuf.data() + 20, 4);
      		basicTrackMsg.estY = aux_float;

      		
      		_basicTrackReportPublisher.publish(basicTrackMsg);

		} else if ( (msgIdentifier == 0x01AD0201) && (num_bytes == NormalTrackReport::UDPMSGSIZE) ) {
			// Normal track report
			ROS_INFO("Normal Track Report Received");

		} else if ( (msgIdentifier == 0x01AD0301) && (num_bytes == ExtendedTrackReport::UDPMSGSIZE) ){
			// Extended track report
			ROS_INFO("Extended Track Report Received");

		} else {
			ROS_WARN("Incorrect Track Report Reception");
		}

		_trackReportSocket.async_receive_from(
				boost::asio::buffer(_trackReportRecvBuf), _server_endpoint,
				boost::bind(&SehirusConversion::udp_handle_receive_trackreport, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
}

void SehirusConversion::main()
{
	_heartBeatPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusHeartbeat>("heartbeat", 10);
	_basicTrackReportPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusBasicTrackReport>("basic_track_report", 10);
	_normalTrackReportPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusNormalTrackReport>("normal_track_report", 10);
	_extendedTrackReportPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusExtendedTrackReport>("extended_track_report", 10);
	// Aquí debo añadir más publicadores para el resto de mensajes
	ros::Duration(1).sleep();
	//*************************

	try
	{
		//boostUdp::resolver resolver(_io_service);
		// A la consulta se le pasan los siguientes argumentos
		// 1º Protocolo a usar, en este caso UDP/IPv4
		// 2º El nombre del servidor (FQDN)
		// 3º El servicio (o puerto) al que se pretende acceder (ver Lista de puertos TCP y UDP en wikipedia)
		//boostUdp::resolver::query query(boostUdp::v4(), "localhost", "1234");
		//_server_endpoint = *resolver.resolve(query);

		// Imprimir la dirección IP del endpoint
		//std::cout << "dirección IP del servidor " << _server_endpoint.address() << std::endl;

		//boostUdp::socket socket(_io_service);
		//_socket.open(boostUdp::v4());

		//_send_buf[0] = 42;
		//_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);

		_heartbeatSocket.async_receive_from(
			boost::asio::buffer(_heartbeatRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_heartbeat, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		_trackReportSocket.async_receive_from(
			boost::asio::buffer(_trackReportRecvBuf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive_trackreport, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

	}
	catch (std::exception& e)
	{
		//std::cerr << e.what() << std::endl;
		ROS_WARN_STREAM("Exception related with UDP socket: " << e.what());
	}
	while (ros::ok())
	{
		_io_service.poll_one();
		//_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);
		ros::Duration(0.05).sleep();
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "sehirus_ros_node");
	SehirusConversion sehirusConversion;
	sehirusConversion.main();
	return 0;
}