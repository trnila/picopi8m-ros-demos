#!/usr/bin/env python
import time
from pinctrl import PinCtrl

ctrl = PinCtrl()

pin = 122
ctrl.pin_configure(pin)
while True:
    ctrl.pin_set(pin)
    time.sleep(0.05)
    ctrl.pin_clr(pin)
    time.sleep(0.05)



