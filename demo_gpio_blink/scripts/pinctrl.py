import serial
import io
import logging
import time


class PinCtrl:
    def __init__(self):
        self.s = None
        self.sio = None
        self._open()

    def _open(self):
        self.s = serial.Serial('/dev/ttyRPMSG30')
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.s, self.s, 1))

    def _reopen(self):
        while True:
            try:
                self._open()
                return
            except serial.serialutil.SerialException as e:
                logging.exception(e)
                time.sleep(1)


    def send(self, cmd, pin, expect):
        while True:
            try:
                self.sio.write("{} {}\n".format(cmd, pin).decode('utf-8'))
                self.sio.flush()
                rcv = self.sio.readline().encode('utf-8').strip()

                if rcv != "pin {} {}".format(pin, expect):
                    raise Exception("unexpected response {}".format(rcv))
                return
            except serial.serialutil.SerialException as e:
                logging.exception(e)
                self._reopen()



    def pin_configure(self, pin):
        self.send("out", pin, "configured")

    def pin_set(self, pin):
        self.send("set", pin, "set")

    def pin_clr(self, pin):
        self.send("clr", pin, "cleared")

