# notify applications with opened /dev/ttyRPMSG to close it before unloading rpmsg

## prerequisites
```sh
$ modprobe imx_rpmsg
$ modprobe imx_rpmsg_tty
```

## run
```sh
term1 $ while true; do ./m4run strupper; sleep3; done
term2 $ make main && ./main
```
