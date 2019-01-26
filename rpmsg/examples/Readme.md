# Use of remoteproc's imx_rproc with rpmsg

```sh
$ cd m4
$ echo stop > /sys/kernel/debug/remoteproc/remoteproc0/state
$ m4build
$ cp build/debug/m4char /lib/firmware/rproc-imx-rproc-fw
$ echo start > /sys/kernel/debug/remoteproc/remoteproc0/state
$ ../m4char
```

Same aplies for tty with rpmsg driver imx_rpmsg_tty.
