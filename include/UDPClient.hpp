#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class UDPClient
{
public:
	UDPClient( boost::asio::io_service& io_service, const std::string& host, const std::string& port) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)), send_intervall_(0.f)
	{
		udp::resolver resolver(io_service_);
		udp::resolver::query query(udp::v4(), host, port);
		udp::resolver::iterator iter = resolver.resolve(query);
		endpoint_ = *iter;

		last_send_ = clock();
	}

	~UDPClient()
	{
		socket_.close();
	}

	std::string buildPacket(const int id_kinect, const std::vector<int>& coordinates) 
	{
		std::string packet = boost::lexical_cast<std::string>(id_kinect) + "-" + boost::lexical_cast<std::string>(coordinates.size()*0.5);

		for (size_t i_c = 0; i_c < coordinates.size(); i_c += 2)
			packet  += "-" + boost::lexical_cast<std::string>(coordinates[i_c]) + "-" + boost::lexical_cast<std::string>(coordinates[i_c + 1]);
	
		return packet;
	}
	

	void send(const std::string& packet) 
	{
		time_t current = clock();
		const float time_diff = float(current-last_send_) / CLOCKS_PER_SEC;

		if (time_diff >= send_intervall_) 
		{
			last_send_ = clock();
			
			socket_.send_to(boost::asio::buffer(packet, packet.size() * sizeof(char)), endpoint_);
		}
	}

	/*send packet every seconds*/
	inline void setSendIntervall(float send_intervall) { send_intervall_ = send_intervall;};

private:

	boost::asio::io_service& io_service_;

	udp::socket socket_;

	udp::endpoint endpoint_;

	time_t last_send_;

	float send_intervall_;
};

#endif //UDP_CLIENT_H