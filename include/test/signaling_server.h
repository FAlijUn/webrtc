#pragma once
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <queue>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;

class BrowserSession : public std::enable_shared_from_this<BrowserSession>{
public:
  BrowserSession(std::shared_ptr<websocket::stream<beast::tcp_stream>> ws, std::shared_ptr<websocket::stream<beast::tcp_stream>> gstreamer_session)
    :ws_(ws),
    gstreamer_session_(gstreamer_session){}
  void start(){
    ws_->async_accept([self = shared_from_this()] (beast::error_code ec){
      if(ec){
        std::cerr << "Error in async_accept: " << ec.message() << std::endl;
        return;
      }
      std::cout << "Browser connected" << std::endl;
      self->do_read();
    });
  }

  void do_read(){
    ws_->async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred){
      if(ec){
        std::cerr << "Failed to read message from browser" << ec.message() << std::endl;
        self->ws_->close(websocket::close_code::normal);
        return;
      }
      self->on_read(ec, bytes_transferred);
    });
  }

  void on_read(beast::error_code ec, std::size_t bytes_transferred){
    if(ec){
      std::cerr << "Error in on_read: " << ec.message() << std::endl;
      return;
    }
    std::string message = beast::buffers_to_string(buffer_.data());
    std::cout << "Received message from browser: " << message << std::endl;
    buffer_.consume(buffer_.size());
    if (gstreamer_session_) {
      gstreamer_session_->async_write(asio::buffer(message), [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred) {
        if (ec) {
          std::cerr << "Failed to send message to GStreamer: " << ec.message() << std::endl;
          return;
        }
        self->do_read();
      });
    } else {
      std::cerr << "GStreamer session not available" << std::endl;
    }
  }

  void on_write(beast::error_code ec, std::size_t bytes_transferred){
    if(ec){
      std::cerr << "Error in on_write: " << ec.message() << std::endl;
      return;
    }
    std::cout << "Sent message to gstreamer" << std::endl;
  }

private:
  std::shared_ptr<websocket::stream<beast::tcp_stream>> ws_;
  beast::flat_buffer buffer_;
  std::shared_ptr<websocket::stream<beast::tcp_stream>> gstreamer_session_;
};

class GStreamerSession : public std::enable_shared_from_this<GStreamerSession> {
  public:
    GStreamerSession(std::shared_ptr<websocket::stream<beast::tcp_stream>> ws, std::shared_ptr<websocket::stream<beast::tcp_stream>> browser_session):
      ws_(ws),
      browser_session_(browser_session){}
  
    void start(){
      ws_->async_accept([self = shared_from_this()](beast::error_code ec){
        if(ec){
          std::cerr << "Error in async_accept: " << ec.message() << std::endl;
          return;
        }
        std::cout << "GStreamer connected" << std::endl;
        self->do_read();
      });
    }

    void do_read(){
      ws_->async_read(buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred){
        if(ec){
          std::cerr << "Failed to read message from gstreamer" << ec.message() << std::endl;
          self->ws_->close(websocket::close_code::normal);
          return;
        }
        self->on_read(ec, bytes_transferred);
      });
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred){
      if(ec){
        std::cerr << "Error in on_read: " << ec.message() << std::endl;
        return;
      }
      std::string message = beast::buffers_to_string(buffer_.data());
      std::cout << "Received message from gstreamer: " << message << std::endl;
      buffer_.consume(buffer_.size());
      if (browser_session_ && browser_session_->is_open()) {  // Check validity and connection state
        browser_session_->async_write(asio::buffer(message), [self = shared_from_this()](beast::error_code ec, std::size_t bytes_transferred) {
          if (ec) {
            std::cerr << "Failed to send message to browser: " << ec.message() << std::endl;
            return;
          }
          self->do_read();
        });
      } else {
        std::cerr << "Browser session not available or closed" << std::endl;
      }
    }

    void on_write(beast::error_code ec, std::size_t bytes_transferred){
      if(ec){
        std::cerr << "Error in on_write: " << ec.message() << std::endl;
        return;
      }
      std::cout << "Sent message to browser" << std::endl;
    }
  
  private:
    std::shared_ptr<websocket::stream<beast::tcp_stream>> ws_;
    beast::flat_buffer buffer_;
    std::shared_ptr<websocket::stream<beast::tcp_stream>> browser_session_;
};

class SignalingServer{
public:
  SignalingServer(asio::io_context& ioc):
    ioc_(ioc),
    browser_acceptor_(ioc, tcp::endpoint(tcp::v4(), 8000)),
    gstreamer_acceptor_(ioc, tcp::endpoint(tcp::v4(), 8001)){
      std::cout << "Signaling server started" << std::endl;
    }
  
  void run(){
    start_browser_acceptor();
    start_gstreamer_acceptor();
  }

private:
  void start_browser_acceptor() {
    browser_acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec) {
        std::cout << "Browser connection accepted." << std::endl;
        auto browser_ws = std::make_shared<websocket::stream<beast::tcp_stream>>(std::move(socket));
        
        // 检查GStreamer队列是否有等待的会话
        if (!gstreamer_session_queue_.empty()) {
          auto gstreamer_ws = gstreamer_session_queue_.front();
          gstreamer_session_queue_.pop();
          
          // 创建相互关联的会话
          auto browser_session = std::make_shared<BrowserSession>(browser_ws, gstreamer_ws);
          auto gstreamer_session = std::make_shared<GStreamerSession>(gstreamer_ws, browser_ws);
          browser_session->start();
          gstreamer_session->start();
        } else {
          // 无等待的GStreamer会话，放入浏览器队列
          browser_session_queue_.push(browser_ws);
          std::cout << "Browser session queued, waiting for GStreamer..." << std::endl;
        }
      } else {
        std::cerr << "Error accepting browser connection: " << ec.message() << std::endl;
      }
      start_browser_acceptor();
    });
  }

  void start_gstreamer_acceptor() {
    gstreamer_acceptor_.async_accept([this](beast::error_code ec, tcp::socket socket) {
      if (!ec) {
        std::cout << "GStreamer connection accepted." << std::endl;
        auto gstreamer_ws = std::make_shared<websocket::stream<beast::tcp_stream>>(std::move(socket));
          
        // 检查浏览器队列是否有等待的会话
        if (!browser_session_queue_.empty()) {
          auto browser_ws = browser_session_queue_.front();
          browser_session_queue_.pop();
            
            // 创建相互关联的会话
          auto browser_session = std::make_shared<BrowserSession>(browser_ws, gstreamer_ws);
          auto gstreamer_session = std::make_shared<GStreamerSession>(gstreamer_ws, browser_ws);
          browser_session->start();
          gstreamer_session->start();
        } else {
            // 无等待的浏览器会话，放入GStreamer队列
          gstreamer_session_queue_.push(gstreamer_ws);
          std::cout << "GStreamer session queued, waiting for Browser..." << std::endl;
        }
      } else {
        std::cerr << "Error accepting GStreamer connection: " << ec.message() << std::endl;
      }
      start_gstreamer_acceptor();
    });
}


  // void accept_browser_connection(){
  // }

  // void accept_gstreamer_connection(){
  // }

  boost::asio::io_context& ioc_;
  tcp::acceptor browser_acceptor_;
  tcp::acceptor gstreamer_acceptor_;
  std::shared_ptr<websocket::stream<beast::tcp_stream>> browser_session_ = nullptr;
  std::shared_ptr<websocket::stream<beast::tcp_stream>> gstreamer_session_ = nullptr;
  std::queue<std::shared_ptr<websocket::stream<beast::tcp_stream>>> browser_session_queue_;
  std::queue<std::shared_ptr<websocket::stream<beast::tcp_stream>>> gstreamer_session_queue_;
};




