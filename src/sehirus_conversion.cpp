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
_nodeHandle()
{
}

/**
* SehirusConversion destructor
*/
SehirusConversion::~SehirusConversion()
{

}

void SehirusConversion::main()
{
	_heartBeatPublisher = _nodeHandle.advertise<radar_msg_ros_conversion::SehirusHeartbeat>("heartbeat", 10);
	// Aquí debo añadir más publicadores para el resto de mensajes
	radar_msg_ros_conversion::SehirusHeartbeat heartBeatMsg;
	ros::Duration(5).sleep();
	//*************************
	while(ros::ok())
	{
		try
		{
			boost::asio::io_service io_service;

			boostUdp::resolver resolver(io_service);
			// A la consulta se le pasan los siguientes argumentos
			// 1º Protocolo a usar, en este caso UDP/IPv4
			// 2º El nombre del servidor (FQDN)
			// 3º El servicio (o puerto) al que se pretende acceder (ver Lista de puertos TCP y UDP en wikipedia)
			boostUdp::resolver::query query(boostUdp::v4(), "localhost", "1234");
			boostUdp::endpoint server_endpoint = *resolver.resolve(query);

			// Imprimir la dirección IP del endpoint
			std::cout << "dirección IP del servidor " << server_endpoint.address() << std::endl;

			boostUdp::socket socket(io_service);
			socket.open(boostUdp::v4());

			boost::array<uint8_t, 1> send_buf  = {{ 42 }};
			socket.send_to(boost::asio::buffer(send_buf), server_endpoint);

			boost::array<uint8_t, 72> recv_buf;
			//boostUdp::endpoint client_endpoint;
			std::cout << "Trying to receive something" << std::endl;
			size_t len = socket.receive_from(
		    	boost::asio::buffer(recv_buf), server_endpoint);

			//std::cout.write(recv_buf.data(), len);
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

			// Probemos primero únicamente con los siguientes campos del mensaje heartbeat
			heartBeatMsg.msgIdentifier = (uint32_t)recv_buf[3] << 24  |
      			  				  	  (uint32_t)recv_buf[2] << 16 |
      			  				  	  (uint32_t)recv_buf[1] << 8  |
      			  				  	  (uint32_t)recv_buf[0];

      		char serverName[16];
      		for(int i=0; i<=16; i++) {
      			serverName[i] = recv_buf[4+i]; 
      		}
			heartBeatMsg.serverName = std::string(serverName);
			heartBeatMsg.commandPort = (uint16_t) recv_buf[21] << 8 |
										(uint16_t) recv_buf[20];
			heartBeatMsg.commandConnected = (uint8_t) recv_buf[22];
			heartBeatMsg.commandListen = (uint8_t) recv_buf[23];
			_heartBeatPublisher.publish(heartBeatMsg);

		}
		catch (std::exception& e)
		{
			//std::cerr << e.what() << std::endl;
			ROS_WARN_STREAM("Exception related with UDP socket: " << e.what());
		}
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "sehirus_ros_node");
	SehirusConversion sehirusConversion;
	sehirusConversion.main();
	return 0;
}