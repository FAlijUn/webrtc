#include "api.h"

int main(){
  SharedMemory shm;
  if(!shm.init()){
    std::cerr << "Failed to create shared memory" << std::endl;
    return -1;
  }

  VideoCapture video_capture(&shm);
  VideoStream video_stream(&shm);
  VideoROS video_ros(&shm);

  if(!video_capture.init()){
    std::cerr << "Failed to initialize video capture" << std::endl;
    return -1;
  }
  if(!video_ros.init()){
    std::cerr << "Failed to initialize video ros" << std::endl;
    return -1;
  }

  std::thread capture_thread(&VideoCapture::capture_data, &video_capture);
  std::thread stream_thread(&VideoStream::main, &video_stream);
  std::thread ros_thread(&VideoROS::read_data, &video_ros);

  capture_thread.join();
  stream_thread.join();
  ros_thread.join();

  shm.cleanup();
  return 0;
}