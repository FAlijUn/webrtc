#include "api.h"

int main(){
  SharedMemory shm;
  if(!shm.init()){
    std::cerr << "Failed to create shared memory" << std::endl;
    return -1;
  }

  VideoCapture video_capture(&shm);
  VideoStream video_stream(&shm);

  std::thread capture_thread(&VideoCapture::write_data, &video_capture);
  std::thread stream_thread(&VideoStream::read_data, &video_stream);

  capture_thread.join();
  stream_thread.join();

  shm.cleanup();
  return 0;
}