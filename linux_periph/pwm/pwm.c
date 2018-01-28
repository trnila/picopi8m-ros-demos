#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <linux/limits.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


int pwm_init(int n) {
  struct stat s;
  char path[PATH_MAX];

  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d", n);
  if(stat(path, &s) != 0) {
    return -ENODEV;
  }


  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0", n);
  if(stat(path, &s) == 0) {
    return 0;
  }

  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", n);
  int fd = open(path, O_WRONLY);
  if(!fd) {
    return -ENODEV;
  }

  char enable = '0';
  if(write(fd, &enable, 1) != 1) {
    return -1;
  }
  close(fd);
  return 0;
}

int pwm_enable(int n, int enable) {
  char path[PATH_MAX];
  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/enable", n);

  int fd = open(path, O_WRONLY);
  if(!fd) {
    return -1;
  }

  char cmd = enable ? '1' : '0';
  if(write(fd, &cmd, 1) != 1) {
    return -1;
  }

  close(fd);

  return 0;
}

int pwm_set_period(int n, uint64_t period) {
  char path[PATH_MAX];

  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0", n);
  FILE *file = fopen(path, "w");
  if(!file) {
    return -1;
  }

  fprintf(file, "%lu", period);
  fclose(file);
}

int pwm_set_duty(int n, uint64_t duty) {
  char path[PATH_MAX];

  snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", n);

  FILE *file = fopen(path, "w");
  if(!file) {
    return -1;
  }

  fprintf(file, "%lu", duty);
  fclose(file);
}


int main() {
  int pwm = 2;
  int offset_us = 500; 
  int center_us = 1500;
  int step_us = 100;
  uint64_t period = 20000000UL;

  pwm_init(pwm);
  pwm_enable(pwm, 1);
  pwm_set_period(pwm, period);

  int duty = center_us;
  int sign = 1;
  for(;;) {
    pwm_set_duty(pwm, duty * 1000);
    duty += sign * step_us;

    if(duty >= center_us + offset_us || duty <= center_us - offset_us) {
      duty = MIN(center_us + offset_us, MAX(center_us - offset_us, duty));
      sign *= -1;
    }

    printf("duty = %d us\n", duty);
    usleep(20 * 1000);
  }

  return 0;
}

