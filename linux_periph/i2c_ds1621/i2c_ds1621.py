#!/usr/bin/env python3
from smbus2 import SMBus
import sys

DS1621_READ_TEMP     = 0xAA
DS1621_READ_COUNTER  = 0xA8
DS1621_READ_SLOPE    = 0xA9
DS1621_START_CONVERT = 0xEE
DS1621_ADDR          = 0x48

bus = SMBus(sys.argv[1] if len(sys.argv) > 1 else 2)

addr = DS1621_ADDR

bus.write_byte(DS1621_ADDR, DS1621_START_CONVERT)

while True:
    data = bus.read_i2c_block_data(addr, DS1621_READ_TEMP, 2)
    count_per_c = bus.read_byte_data(addr, DS1621_READ_SLOPE)
    count_remain = bus.read_byte_data(addr, DS1621_READ_COUNTER)

    temperature = data[0] - 0.25 + (count_per_c - count_remain) / count_per_c
    print(temperature)
