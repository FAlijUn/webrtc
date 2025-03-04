#pragma once
#include <iostream>
#include <sys/ioctl.h>

struct Buffer{
  void *start;
  uint32_t length;
};

int xioctl(int fd, uint64_t request, void* arg) {
  int r = 0;
  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}