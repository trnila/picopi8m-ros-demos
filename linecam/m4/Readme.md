# M4 linecam
**linecam_simple** uses busy-waiting for SPI transfers
**linecam_debug** prints captured lines to M4 stdout (capturing method can be changed in CMakeLists.txt)
**linecam_irq** uses SDK's FreeRTOS support
**linecam_raw** directly accesses peripheral and handles interrupts for faster responses
