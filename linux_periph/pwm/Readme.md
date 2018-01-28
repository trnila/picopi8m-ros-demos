# pwm

## pwm on linux
Usage:
```sh
$ ./pwm.sh id period_ns duty_ns
```

Examples:
```sh
$ ./pwm.sh 2 1000000000 500000000 # PWM2, 0.5s on, 0.5s off
$ ./pwm.sh 2 20000000 1500000     # PWM2, 20 ms period, 1.5 ms on
$ ./pwm.sh 2 2000 1000            # PWM2, 500 Khz signal

$ for i in {2..4}; do ./pwm.sh $i 1000000000 $((1000000000/$i)); done
```
