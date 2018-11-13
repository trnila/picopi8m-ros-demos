# RDC

## rdc_gpio1_m4_only.c
Set A53 rdc-domain to read only so Linux cant modify that peripheral.
If so, it will end up with exception:


```sh
$ echo 119 > /sys/class/gpio/export
$ echo out > /sys/class/gpio/gpio119/direction
$ echo 1 > /sys/class/gpio/gpio119/value
$ m4run rdc_gpio1_m4_only
$ echo 1 > /sys/class/gpio/gpio119/value

[  290.203482] SError Interrupt on CPU3, code 0xbf000002 -- SError
[  290.209417] CPU: 3 PID: 2583 Comm: bash Not tainted 4.9.88-2.0.0-gfe2c141ed7a0-dirty #35
[  290.217510] Hardware name: TechNexion PICO-IMX8M and PI baseboard (DT)
[  290.224042] task: ffff800076986900 task.stack: ffff8000746cc000
[  290.229966] PC is at 0xffff8caa9bac
[  290.233458] LR is at 0xffff8ca55398
[  290.236950] pc : [<0000ffff8caa9bac>] lr : [<0000ffff8ca55398>] pstate: 20000000
[  290.244348] sp : 0000ffffc33da690
[  290.247666] x29: 0000ffffc33da690 x28: 0000aaaaf509e740 
[  290.253008] x27: 0000aaaae502e000 x26: 0000aaaae5041000 
[  290.258350] x25: 0000ffff8cb3b648 x24: 0000000000000002 
[  290.263691] x23: 0000aaaaf509e920 x22: 0000000000000002 
[  290.269033] x21: 0000ffff8cb3b560 x20: 0000aaaaf509e920 
[  290.274375] x19: 0000000000000001 x18: 0000ffff8cb3aa70 
[  290.279716] x17: 0000ffff8ca52028 x16: 0000aaaae5058bf0 
[  290.285058] x15: 0000000000000070 x14: 0000000000000000 
[  290.290398] x13: 0000000000000000 x12: 0000000000000020 
[  290.295739] x11: 0000ffffc33da638 x10: 0000000000000000 
[  290.301081] x9 : 0000ffff8ca559c8 x8 : 0000000000000040 
[  290.306422] x7 : 0000000000000001 x6 : 0000000000000031 
[  290.311764] x5 : 0000000155510004 x4 : 0000000000000000 
[  290.317105] x3 : 0000ffff8cb3f1a8 x2 : 0000000000000002 
[  290.322446] x1 : 0000aaaaf509e920 x0 : 0000000000000002 
[  290.327785] 
[  290.329280] Kernel panic - not syncing: Asynchronous SError Interrupt
[  290.335726] CPU: 3 PID: 2583 Comm: bash Not tainted 4.9.88-2.0.0-gfe2c141ed7a0-dirty #35
[  290.343818] Hardware name: TechNexion PICO-IMX8M and PI baseboard (DT)
[  290.350347] Call trace:
[  290.352805] [<ffff0000080898b0>] dump_backtrace+0x0/0x1b0
[  290.358212] [<ffff000008089a74>] show_stack+0x14/0x20
[  290.363271] [<ffff0000084072c8>] dump_stack+0x94/0xb4
[  290.368330] [<ffff00000817b92c>] panic+0x11c/0x278
[  290.373127] [<ffff00000808a1a0>] do_serror+0x88/0x90
[  290.378097] [<ffff0000080835f8>] el0_error_naked+0x10/0x18
[  290.383586] SMP: stopping secondary CPUs
[  290.388042] Kernel Offset: disabled
[  290.391533] Memory Limit: none
[  290.394596] ---[ end Kernel panic - not syncing: Asynchronous SError Interrupt
```
