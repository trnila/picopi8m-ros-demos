#pragma once
#include <linux/time.h>

uint64_t timespec_diff_ns(struct timespec *t1, struct timespec *t2);
