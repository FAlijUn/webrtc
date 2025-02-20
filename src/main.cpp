#include "api.h"

int main(){
  SharedMemory shm;
  if(!shm.init()){
    std::cerr << "Failed to create shared memory" << std::endl;
    return -1;
  }

  VideoCapture video_capture(&shm);
  VideoStream video_stream(&shm);

  video_capture.init();
  std::thread capture_thread(&VideoCapture::capture_data, &video_capture);
  std::thread stream_thread(&VideoStream::main, &video_stream);

  capture_thread.join();
  stream_thread.join();

  shm.cleanup();
  return 0;
}