#pragma once
#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <thread>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <cstring>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class VideoStream{
private:
  SharedMemory *shm_;
public:
  VideoStream(SharedMemory *shm): shm_(shm) {}

  void read_data(){
    while(true){
      std::unique_lock<std::mutex> lock(shm_->mtx_);
      
      // 等待新数据就绪
      shm_->cv_.wait(lock, [this]{ return shm_->data_ready_; });

      // 读取数据
      std::cout << "Read thread: shared_data = " << (int)shm_->shared_data_[0] << std::endl;

      // 更新状态
      shm_->data_ready_ = false;
      shm_->data_processed_ = true;

      lock.unlock();
      shm_->cv_.notify_one();
    }
  }
};