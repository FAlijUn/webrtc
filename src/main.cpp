#include <rclcpp/rclcpp.hpp>
#include "api.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("video_ros_node");

  SharedMemory shm;
  if(!shm.init()){
    std::cerr << "Failed to create shared memory" << std::endl;
    return -1;
  }
  
  
  VideoCapture video_capture(&shm);
  
  std::string host = "localhost";
  std::string port = "8001";
  VideoStream video_stream(&shm, host, port);
  // VideoStreamWeb video_stream_web(&shm);
  VideoROS video_ros(&shm, node);

  if(!video_capture.init()){
    std::cerr << "Failed to initialize video capture" << std::endl;
    return -1;
  }
  if(!video_ros.init()){
    std::cerr << "Failed to initialize video ros" << std::endl;
    return -1;
  }
  

  std::thread capture_thread(&VideoCapture::capture_data, &video_capture);
  // std::thread stream_thread(&VideoStream::main, &video_stream);
  std::thread ros_thread(&VideoROS::read_data, &video_ros);
  video_stream.init();
  

  rclcpp::on_shutdown([&](){
    video_capture.stop();
    video_stream.stop();
    video_ros.stop();
    capture_thread.join();
    ros_thread.join();
    shm.cleanup();
  });


  // rclcpp::spin(node);
  // // video_stream_web.init();
 
  // capture_thread.join();
  // // stream_thread.join();
  // ros_thread.join();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}