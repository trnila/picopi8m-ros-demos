#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

static uint64_t timespec_diff_ns(struct timespec *t1, struct timespec *t2) {
	struct timespec diff;
	if (t2->tv_nsec - t1->tv_nsec < 0) {
		diff.tv_sec  = t2->tv_sec - t1->tv_sec - 1;
		diff.tv_nsec = t2->tv_nsec - t1->tv_nsec + 1000000000UL;
	} else {
		diff.tv_sec  = t2->tv_sec - t1->tv_sec;
		diff.tv_nsec = t2->tv_nsec - t1->tv_nsec;
	}
	return (diff.tv_sec * 1000000000UL + diff.tv_nsec);
}

static struct timespec start;

void benchmark_start() {
  clock_gettime(CLOCK_MONOTONIC, &start);
}

uint64_t benchmark_stop() {
  struct timespec end;
  clock_gettime(CLOCK_MONOTONIC, &end);

  return timespec_diff_ns(&start, &end);
}

void do_benchmark(int fd, int total) {
  uint64_t *times = new uint64_t[total];
  int ping = 0;
  while(ping < total) {
    benchmark_start();
    if(write(fd, &ping, sizeof(ping)) <= 0) {
      perror("write");
      exit(1);
    }

    int got = -1;
    if(read(fd, &got, sizeof(got)) <= 0) {
      perror("read");
      exit(1);
    }

    times[ping] = benchmark_stop();

    ping++;
    if(ping != got) {
      printf("error got %d, expected %d\n", got, ping);
      exit(1);
    }
  }
  for(int i = 0; i < total; i++) { 
    printf("%lu\n", times[i]);
  }
}
