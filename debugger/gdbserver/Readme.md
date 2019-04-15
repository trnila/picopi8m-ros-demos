# M4 gdbserver
`gdbserver.py` provides to gdb memory and content of registers on hard fault.
On hard fault, all registers are pushed to the MSP and address of MSP is stored to global variable.
This variable is examined from gdbserver and sent to gdb frontend.

Run `m4run div_zero` and wait until M4 crashes:
```
Started
HARD FAULT
run gdbserver.py
```
Now you can start `./gdbserver.py` and start GDB:
```sh
$ gdb ./build/debug/div_zero -q -ex "target remote :12345"
The target architecture is assumed to be arm
Reading symbols from ./build/debug/div_zero...done.
Remote debugging using :12345
0x1ffe70e6 in division (a=128, b=0) at /root/catkin_ws/src/d/div_zero.c:8
8          return (d + a) / b;
(gdb) p d
$1 = 15
(gdb) p a
$2 = 128
(gdb) p b
$3 = 0
(gdb) bt
#0  0x1ffe70e6 in division (a=128, b=0) at /root/catkin_ws/src/d/div_zero.c:8
#1  main () at /root/catkin_ws/src/d/div_zero.c:22
(gdb) frame 1
#1  main () at /root/catkin_ws/src/d/div_zero.c:22
22          volatile int x = division(val, 128);
(gdb) p val
$4 = 0
```
