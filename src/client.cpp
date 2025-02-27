#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <iostream>
#include <string>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using wsts = beast::websocket::stream<beast::tcp_stream>;