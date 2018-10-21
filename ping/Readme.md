# continuous ping-pong with rpmsg_char
This example demonstrates continuously running application on Linux that communicates with restarting M4 core.

## Run
Run continuously running application:
```sh
$ make sendping
 $ ./sendpings
```

Restart M4 core anytime you want:
```sh
 $ cd m4
 $ m4run ping
```
