module;
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
export module udpreceiver;

namespace comm
{
	using boost::asio::ip::udp;

	export
	template<typename T, int buffer_size>
	class udp_receiver
	{
		using callback = std::function<bool(T&)>;

	private:
		constexpr static int port_def_ = 5557;

	public:
		udp_receiver()
			: socket_(io_service)
		{ }

		void run(callback cb, int port = port_def_)
		{
			callback_ = cb;
			port_ = port;
			//try {
			socket_.open(udp::v4());
			socket_.bind(udp::endpoint(udp::v4(), port_));
			start_receive();
			io_service.run();
			//}
			//catch (std::exception& e) {
			//}
		}

	private:
		void start_receive()
		{
			socket_.async_receive_from(
				boost::asio::buffer(recv_buffer_), 
				remote_endpoint_,
				boost::bind(&udp_receiver::handle_receive, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
		}

		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
		{
			if (!error || error == boost::asio::error::message_size) {
				auto ss = reinterpret_cast<T*>(&recv_buffer_[0]);
				callback_(*reinterpret_cast<T*>(&recv_buffer_[0]));
				start_receive();
			}
		}

		void handle_send(boost::shared_ptr<std::string> /*message*/,
			const boost::system::error_code& /*error*/,
			std::size_t /*bytes_transferred*/)
		{
		}

	private:
		boost::asio::io_service io_service;
		udp::socket socket_;
		udp::endpoint remote_endpoint_;
		std::array<char, buffer_size> recv_buffer_;
		callback callback_;
		int port_;
	};
}