// 使用Boost.Beast 搭建一个信令服务器
#pragma once
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <iostream>
#include <string>
#include <jsoncpp/json/json.h>

namespace asio = boost::asio;
namespace beast = boost::beast;
using tcp = asio::ip::tcp;
using ws = beast::websocket::stream<beast::tcp_stream>;


class WebSocketSession : public std::enable_shared_from_this<WebSocketSession>{
public:
  WebSocketSession(tcp::socket socket) : ws_(std::move(socket)) {}

  void start(){
    ws_.async_accept([self = shared_from_this()](beast::error_code ec){
      if(ec){
        std::cerr << "Error accepting websocket connection: " << ec.message() << std::endl;
        return;
      }
      self->do_read();
    });
  }

  void do_read() {
    ws_.async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred) {
      if (ec) {
        std::cerr << "Failed to read message: " << ec.message() << std::endl;
        return;
      }
      std::string msg{boost::asio::buffers_begin(self->buffer_.data()), boost::asio::buffers_end(self->buffer_.data())};
      self->buffer_.consume(bytes_transferred);

      // 解析信令消息并转发
      Json::Reader reader;
      Json::Value json_message;
      reader.parse(msg, json_message);
      self->broadcast(json_message);

      // 继续读取下一条消息
      self->do_read();
    });
  }

  void broadcast(const Json::Value& message) {
    for (auto& session : sessions_) {
      session.second->send_message(message);
    }
  }

  void send_message(const Json::Value& message) {
    std::string msg_str = message.toStyledString();
    ws_.write(boost::asio::buffer(msg_str));
  }

  static std::unordered_map<int, std::shared_ptr<WebSocketSession>> sessions_;

private:
  ws ws_;
  beast::flat_buffer buffer_;
};

std::unordered_map<int, std::shared_ptr<WebSocketSession>> WebSocketSession::sessions_;

class SignalingServer{
public:  
  SignalingServer(asio::io_context& io_context, short port)
    : acceptor_(io_context, tcp::endpoint(tcp::v4(), port)){
      std::cout << "Signaling server started on port " << port << std::endl;
    }
  
  void run(){
    accept_connection();
  }

private:

  void accept_connection() {
    acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
          std::cout << "New connection!" << std::endl;
          auto session = std::make_shared<WebSocketSession>(std::move(socket));
          session->start();
        }
        accept_connection();
    });
  }

private:
  tcp::acceptor acceptor_;
};


