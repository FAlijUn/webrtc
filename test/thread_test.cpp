#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

std::mutex mtx;
std::condition_variable cv;
int read_threads_done = 0;  // 计数器，跟踪读线程完成状态

void read_thread1() {
    // 模拟处理
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Read thread 1: Finished processing.\n";
    {
        std::lock_guard<std::mutex> lock(mtx);
        read_threads_done++;
    }
    cv.notify_all();  // 通知写线程
}

void read_thread2() {
    // 模拟处理
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "Read thread 2: Finished processing.\n";
    {
        std::lock_guard<std::mutex> lock(mtx);
        read_threads_done++;
    }
    cv.notify_all();  // 通知写线程
}

void write_thread() {
    // 等待直到所有读线程完成
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, []{ return read_threads_done == 2; });  // 确保两个读线程完成
    std::cout << "Write thread: All read threads have completed. Writing data...\n";
}

int main() {
    std::thread t1(read_thread1);
    std::thread t2(read_thread2);
    std::thread t3(write_thread);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
