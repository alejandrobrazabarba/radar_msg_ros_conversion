#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <pthread.h>

using boost::asio::ip::udp;

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: client <host>" << std::endl;
      return 1;
    }

    boost::asio::io_service io_service;

    udp::resolver resolver(io_service);
    // A la consulta se le pasan los siguientes argumentos
    // 1º Protocolo a usar, en este caso UDP/IPv4
    // 2º El nombre del servidor (FQDN)
    // 3º El servicio (o puerto) al que se pretende acceder (ver Lista de puertos TCP y UDP en wikipedia)
    udp::resolver::query query(udp::v4(), argv[1], "1234");
    udp::endpoint receiver_endpoint = *resolver.resolve(query);

    // Imprimir la dirección IP del endpoint
    std::cout << "dirección IP del servidor " << receiver_endpoint.address() << std::endl;

    udp::socket socket(io_service);
    socket.open(udp::v4());

    boost::array<char, 1> send_buf  = {{ 42 }};
    socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

    boost::array<char, 128> recv_buf;
    udp::endpoint sender_endpoint;
    std::cout << "Trying to receive something" << std::endl;
    size_t len = socket.receive_from(
        boost::asio::buffer(recv_buf), sender_endpoint);

    std::cout.write(recv_buf.data(), len);
    std::cout << "Something is received" << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}