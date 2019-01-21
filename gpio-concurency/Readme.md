= GPIO concurency problem when used on M4 and linux side =
GPIO data register GPIO_DR holds data for gpio bank. 
Because there is no set/clear registers, you have to synchronize access to GPIO_DR with SEMA4 for example.
Otherwise you will lost data - as you can see in this example.
```sh
$ make
$ m4run m4
$ ./linux
```

SEMA4 requires modification on booth sides, currently linux driver gpio-mxc does not support it, so its easier and faster to not use same GPIO bank on booth sides.
