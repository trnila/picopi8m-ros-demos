import serial
import io


class PinCtrl:
    def __init__(self):
        self.s = serial.Serial('/dev/ttyRPMSG30')
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.s, self.s, 1))


    def send(self, cmd, pin, expect):
        self.sio.write("{} {}\n".format(cmd, pin).decode('utf-8'))
        self.sio.flush()
        rcv = self.sio.readline().encode('utf-8').strip()

        if rcv != "pin {} {}".format(pin, expect):
            raise Exception("unexpected response {}".format(rcv))

    def pin_configure(self, pin):
        self.send("out", pin, "configured")

    def pin_set(self, pin):
        self.send("set", pin, "set")

    def pin_clr(self, pin):
        self.send("clr", pin, "cleared")

