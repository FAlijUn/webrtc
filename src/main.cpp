#include <rclcpp/rclcpp.hpp>
#include "api.h"

std::shared_ptr<VideoCapture> video_capture_ptr;
std::shared_ptr<Ros2Publish> ros2_publish_ptr;
std::shared_ptr<VideoStream> video_stream_ptr;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::shared_ptr<ShareData> share_data = std::make_shared<ShareData>(1280, 720, 4);
  video_capture_ptr = std::make_shared<VideoCapture>(share_data);
  ros2_publish_ptr = std::make_shared<Ros2Publish>(share_data);
  video_stream_ptr = std::make_shared<VideoStream>(share_data, "localhost", "8001");

  if(!video_capture_ptr->init()){
    std::cerr << "Failed to initialize video capture" << std::endl;
    return -1;
  }
  if(!ros2_publish_ptr->init()){
    std::cerr << "Failed to initialize ros2 publish" << std::endl;
    return -1;
  }
  if(!video_stream_ptr->init()){
    std::cerr << "Failed to initialize video stream" << std::endl;
    return -1;
  }

  video_capture_ptr->start();
  ros2_publish_ptr->start();
  video_stream_ptr->start();

  auto node = rclcpp::Node::make_shared("node");
  rclcpp::spin(node);

  video_capture_ptr->stop();
  ros2_publish_ptr->stop();
  video_stream_ptr->stop();

  rclcpp::shutdown();
  return 0;
}
