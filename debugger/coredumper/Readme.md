# M4 coredumper
M4 coredumper creates elf coredump file from TCM memory, so you can examine some global variables in gdb.
Currently no registers are stored in coredump, so you can't easily examine local variables or get crash callstack.
Core should be paused while creating coredump, otherwise you may get inconsistent memory.
```sh
$ m4run
$ make
$ ./create_coredump
$ gdb build/debug/hello coredump  -q
The target architecture is assumed to be arm
Reading symbols from build/debug/hello...done.
warning: core file may not match specified executable file.
warning: Couldn't find general-purpose registers in core file.
warning: Couldn't find general-purpose registers in core file.
#0  <unavailable> in ?? ()
(gdb) p aa
$1 = 23
(gdb) p text
$2 = "Hello world"
p bb
$3 = 3.14159274
```
