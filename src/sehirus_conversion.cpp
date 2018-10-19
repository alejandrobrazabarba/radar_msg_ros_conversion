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
_socket(_io_service)
{
}

/**
* SehirusConversion destructor
*/
SehirusConversion::~SehirusConversion()
{

}

void SehirusConversion::udp_handle_receive(const boost::system::error_code& error,
										   std::size_t num_bytes)
{
	if (!error)
	{
		radar_msg_ros_conversion::SehirusHeartbeat heartBeatMsg;

		// Probemos primero únicamente con los siguientes campos del mensaje heartbeat
		heartBeatMsg.msgIdentifier = (uint32_t)_recv_buf[3] << 24  |
      				  				 (uint32_t)_recv_buf[2] << 16 |
      				  				 (uint32_t)_recv_buf[1] << 8  |
      				  				 (uint32_t)_recv_buf[0];

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

    	char serverName[16];
    	for(int i=0; i<=16; i++) {
      		serverName[i] = _recv_buf[4+i]; 
    	}
		heartBeatMsg.serverName = std::string(serverName);
		heartBeatMsg.commandPort = (uint16_t) _recv_buf[21] << 8 |
								   (uint16_t) _recv_buf[20];
		heartBeatMsg.commandConnected = (uint8_t) _recv_buf[22];
		heartBeatMsg.commandListen = (uint8_t) _recv_buf[23];
		_heartBeatPublisher.publish(heartBeatMsg);

		_socket.async_receive_from(
			boost::asio::buffer(_recv_buf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}
}

void SehirusConversion::main()
{
	_heartBeatPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusHeartbeat>("heartbeat", 10);
	// Aquí debo añadir más publicadores para el resto de mensajes
	ros::Duration(5).sleep();
	//*************************

	try
	{
		boostUdp::resolver resolver(_io_service);
		// A la consulta se le pasan los siguientes argumentos
		// 1º Protocolo a usar, en este caso UDP/IPv4
		// 2º El nombre del servidor (FQDN)
		// 3º El servicio (o puerto) al que se pretende acceder (ver Lista de puertos TCP y UDP en wikipedia)
		boostUdp::resolver::query query(boostUdp::v4(), "localhost", "1234");
		_server_endpoint = *resolver.resolve(query);

		// Imprimir la dirección IP del endpoint
		std::cout << "dirección IP del servidor " << _server_endpoint.address() << std::endl;

		//boostUdp::socket socket(_io_service);
		//_socket = boostUdp::socket(_io_service);
		_socket.open(boostUdp::v4());

		_send_buf[0] = 42;
		_socket.send_to(boost::asio::buffer(_send_buf), _server_endpoint);

		_socket.async_receive_from(
			boost::asio::buffer(_recv_buf), _server_endpoint,
			boost::bind(&SehirusConversion::udp_handle_receive, this,
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
		ros::Duration(1).sleep();
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "sehirus_ros_node");
	SehirusConversion sehirusConversion;
	sehirusConversion.main();
	return 0;
}