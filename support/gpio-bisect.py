#!/usr/bin/env python3
"""
Simple program that finds pin mapping from das u-boot
Connect led to pin and run this program with serial connected to das u-boot.
If led is lit, then answer 'Y' otherwise type anything.
"""

import serial
import serial.threaded
import time

class TestLines(serial.threaded.LineReader):
    MAX_GPIO = 160
    IGNORED = [9]

    def handle_line(self, data):
        #print(data)
        pass


    def all_gpios(self, maxi=MAX_GPIO):
        return [i for i in range(0, 160) if i not in self.IGNORED and i <= maxi]

    def set(self, pin, state):
        action = "set" if state else "clear"
        self.write_line(f"gpio {action} {pin}")

    def set_all(self, state):
        for pin in self.all_gpios():
            self.set(pin, state)
            time.sleep(0.01)

    def set_until(self, threshold):
        for pin in self.all_gpios():
            self.set(pin, pin <= threshold)
            time.sleep(0.01)


    def bisect(self):
        left = 0
        right = 160

        while left < right:
            center = (right - left) // 2 + left

            self.set_until(center)

            print(center)
            choose = input()
            if choose == "y":
                right = center 
            else:
                left = center + 1 
            print(left, right)

        center = (right - left) // 2 + left
        print("You won")
        print(center)


        for i in range(0, 10):
            self.set(center, 0)
            time.sleep(0.1)
            self.set(center, 1)
            time.sleep(0.1)



s = serial.Serial("/dev/ttyUSB0", 115200)

with serial.threaded.ReaderThread(s, TestLines) as stream:
    stream.set_all(1)
    stream.set_all(0)
    stream.bisect()
