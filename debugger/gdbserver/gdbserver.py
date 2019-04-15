#!/usr/bin/env python3
import sys
import socket
import logging
import struct
import mmap


class M4Target:
    TCM_LOW = 0x1ffe0000
    TCM_HIGH = 0x2001FFFF
    TCM_A53_ADDR = 0x7E0000

    def __init__(self):
        with open("/dev/mem", "rw+b") as fd:
            size = self.TCM_HIGH - self.TCM_LOW - 1
            self.mem = mmap.mmap(fd.fileno(), size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, 0, self.TCM_A53_ADDR)

    def read_mem(self, addr, size):
        if addr > self.TCM_HIGH or (addr < self.TCM_LOW and addr >= 64):
            logging.info("access outside TCM: %x", addr)
            return "0" * size

        if addr > self.TCM_LOW:
            addr -= self.TCM_LOW

        result = bytearray()
        for i in range(size):
            result.append(self.mem[addr + i])
        return bytes(result)

    def get_registers(self):
        sp = struct.unpack('<I', self.read_mem(0x20000b14, 4))[0]
        a = struct.unpack(">9I", self.read_mem(sp, 4*9))
        return [
            a[0], # r0
            a[1], # r1
            a[2], # r2
            a[3], # r3
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            a[4], # r12
            a[8], # sp
            a[5], # lr
            a[6], # pc
            a[7], # psr
        ]

class GDBClientHandler(object):
    def __init__(self, sock, target):
        self.target = target
        self.sock = sock
        self.buf = bytearray(4096)
        self.pos = 0
        self.len = 0

        self.handlers = {
            '?': self.handle_qmark,
            'g': self.handle_read_registers,
            'm': self.handle_read_mem,
        }

    def handle_qmark(self, subcmd):
        self.send('S05')

    def handle_read_registers(self, subcmd):
        if subcmd == '':
            res = "".join(["{:08x}".format(reg) for reg in self.target.get_registers()])
            self.send(res)

    def handle_read_mem(self, subcmd):
        addr, size = subcmd.split(',')
        addr = int(addr, 16)
        size = int(size, 16)
        logging.info('read memory(%#.8x, %d)', addr, size)
        self.send(self.target.read_mem(addr, size).encode('hex'))

    def run(self):
        while True:
            pkt = self.receive()
            if not pkt:
                break

            logging.debug('received %s', pkt)
            self.send_raw('+')

            cmd, subcmd = pkt[0], pkt[1:]
            if cmd == 'k':
                break

            if cmd in self.handlers:
                self.handlers[cmd](subcmd)
            else:
                logging.info('%s command not handled', pkt)
                self.send('')

        self.sock.close()

    def receive(self):
        csum = 0
        their_csum = 0
        state = 0
        request = bytearray()
        while True:
            if self.pos >= self.len:
                self.pos = 0
                self.len = self.sock.recv_into(self.buf)
                if self.len <= 0:
                    return None

            while self.pos < self.len:
                c = chr(self.buf[self.pos])
                self.pos += 1

                if state == 0:
                    if c == '$':
                        state = 1
                elif state == 1:
                    if c == '#':
                        state = 2
                    else:
                        csum += ord(c)
                        request.append(c)
                elif state == 2:
                    their_csum = int(c, 16) << 4
                    state = 3
                elif state == 3:
                    their_csum += int(c, 16)
                    if their_csum != csum & 0xFF:
                        logging.error("invalid checksum")
                        state = 0
                        csum = 0
                    else:
                        state = 0
                        csum = 0
                        return request.decode('utf-8')


    def checksum(self, data):
        checksum = 0
        for c in data:
            checksum += ord(c)
        return checksum & 0xff

    def send(self, msg):
        logging.debug('sending %s', msg)
        self.send_raw('${}#{:02x}'.format(msg, self.checksum(msg)))

    def send_raw(self, r):
        self.sock.sendall(r)


port = 12345

logging.basicConfig(level=logging.INFO)

target = M4Target()

sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', port))
sock.listen(1)

logging.info('listening on %d', port)
while True:
    conn, addr = sock.accept()
    logging.info('connected from %s:%s', addr[0], addr[1])
    GDBClientHandler(conn, target).run()
