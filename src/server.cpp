#include "signaling_server.h"

int main(){
  try{
    asio::io_context ioc;
    SignalingServer server(ioc);
    server.run();
    ioc.run();
  }catch(std::exception& e){
    std::cerr << "Exception: " << e.what() << std::endl;
  }
  return 0;
}