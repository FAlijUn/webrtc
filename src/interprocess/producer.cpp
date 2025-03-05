#include <iostream>
#include <memory>
#include <csignal>
#include "producer.h"

std::shared_ptr<VideoCapture> video_capture;

void signal_handler(int signum) {
  std::cout << "Signal (" << signum << ") received, stopping video capture..." << std::endl;
  if (video_capture) {
    video_capture->stop();
  }
  std::exit(signum);
}

int main(){
  std::signal(SIGINT, signal_handler);
  video_capture = std::make_shared<VideoCapture>();
  if (!video_capture->init()) {
    std::cerr << "Failed to initialize video capture" << std::endl;
    return -1;
  }
  video_capture->start();
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}