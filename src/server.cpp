#include "signaling_server.h"


int main() {
  try {
    asio::io_context io_context;
    SignalingServer server(io_context, 8000); // WebSocket 监听端口 8000
    server.run();
    io_context.run();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  return 0;
}
