#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <set>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <cstring>

std::mutex lock;
std::condition_variable cond;
std::set<uint32_t> items;

void reader(int fd) {
  char buf[128];
  for(;;) {
      int len = read(fd, buf, sizeof(buf));
      //sleep(1);

      if(len <= 0) {
        printf("read = %d\n", len);
      } else {
        buf[len] = 0;
        printf("recv: '%s'\n", buf);

        uint32_t num;
        if(sscanf(buf, "HELLO %d", &num) != 1) {
          printf("error, received %s", buf);
          exit(1);
        }

        {
          std::lock_guard<std::mutex> guard(lock);
          auto it = items.find(num);
          if(it == items.end()) {
            printf("item %d not found in set?\n", num);
            exit(1);
          } else {
            items.erase(it);
          }
        }
        cond.notify_one();
      }
    }
}

int main() {
  int fd = open("/dev/m4char", O_RDWR);
  if(fd < 0) {
    perror("open");
    return -1;
  }

  std::thread reader_thread(reader, fd);
  int i = 0;
  char buf[128];
  for(;;) {
    int len = sprintf(buf, "hello %d", i);
    printf("send: '%s'\n", buf);

    {
      std::unique_lock<std::mutex> lk(lock);
      cond.wait(lk, [&]{return items.size() < 1000;});
      items.insert(i);
      printf("not received: %d\n", items.size());
    }

    int r = write(fd, buf, len);
    if(r != len) {
      printf("write wrote %d but expected %d\n", r, len);
    }

    i++;
  }

  return 0;
}
