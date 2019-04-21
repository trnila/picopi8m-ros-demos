# rpmsg udp
Rpmsg udp module sends all received M4 messages from channel 'udp' over udp to address specified by parameter ip and port.
Its possible to multicast rpmsg messages or just broadcast them if you'r still using ipv4.

## Build & run
```sh
$ make run
```

[![asciicast](https://asciinema.org/a/242020.svg)](https://asciinema.org/a/242020)

## Load module manually
```sh
$ insmod kernel_module/rpmsg_udp.ko ip=::1 port=4444
```

## Change destination at runtime

```sh
$ echo 12345 > /sys/module/rpmsg_udp/parameters/port
$ echo ff02::200 > /sys/module/rpmsg_udp/parameters/ip
```

## Receive examples
### Receive IPv6 multicast
```
$ echo 12345 > /sys/module/rpmsg_udp/parameters/port
$ echo ff04::1 > /sys/module/rpmsg_udp/parameters/ip
$ ./multicast6_recv ff04::1 12345
```

### Receive IPv4 multicast
```
$ echo 12345 > /sys/module/rpmsg_udp/parameters/port
$ echo 239.0.0.1 > /sys/module/rpmsg_udp/parameters/ip
$ ./multicast4_recv 239.0.0.1 12345
```

### Receive IPv4 broadcast
```
$ echo 12345 > /sys/module/rpmsg_udp/parameters/port
$ echo 255.255.255.255 > /sys/module/rpmsg_udp/parameters/ip
$ ./broadcast_recv
```
