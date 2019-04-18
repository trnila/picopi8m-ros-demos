# find out where MemManage occured
```sh
MemManage fault                                                                                                                                                                                   
cfsr  =       82 mmfar = 40000000  bfar = 40000000
pc = 1ffe713a    sp =        0    lr = 1ffe7831
r0 = 1ffed071    r1 = 20000bc0    r2 = 20000ab4 r3 = 40000000 r12 = 20001c34

$ addr2line -e build/debug/print 1ffe7136
/root/catkin_ws/src/debugger/memmanage/print.c:28 (discriminator 1)

$ gdb build/debug/print
(gdb) list *0x1ffe713a
0x1ffe713a is in consumer_task (/root/catkin_ws/src/debugger/memmanage/print.c:28).
23
24      void consumer_task(void *arg) {
25          for(;;) {
26              uint32_t* v;
27              xQueueReceive(Q, &v, portMAX_DELAY);
28              printf("read: %d\r\n", *v);
29          }
30      }
31
32      int main(void) {
(gdb) disassemble 0x1ffe713a
Dump of assembler code for function consumer_task:
0x1ffe7124 <+0>:     push    {r0, r1, r2, lr}
0x1ffe7126 <+2>:     ldr     r5, [pc, #28]   ; (0x1ffe7144 <consumer_task+32>)
0x1ffe7128 <+4>:     ldr     r4, [pc, #28]   ; (0x1ffe7148 <consumer_task+36>)
0x1ffe712a <+6>:     ldr     r0, [r5, #0]
0x1ffe712c <+8>:     add     r1, sp, #4
0x1ffe712e <+10>:    mov.w   r2, #4294967295 ; 0xffffffff
0x1ffe7132 <+14>:    bl      0x1ffe7728 <xQueueReceive>
0x1ffe7136 <+18>:    ldr     r3, [sp, #4]
0x1ffe7138 <+20>:    mov     r0, r4
0x1ffe713a <+22>:    ldr     r1, [r3, #0]
0x1ffe713c <+24>:    bl      0x1ffe077c <printf>
0x1ffe7140 <+28>:    b.n     0x1ffe712a <consumer_task+6>
```
