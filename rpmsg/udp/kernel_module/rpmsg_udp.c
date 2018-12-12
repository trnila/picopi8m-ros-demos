#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/net.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/in.h>
#include <asm/uaccess.h>
#include <linux/socket.h>
#include <linux/slab.h>
#include <linux/inet.h>
#include <net/udp.h>
#include <linux/rpmsg.h>

static char ip[40] = "ff04::1";
static uint16_t port = 12345;

static struct socket *sock = NULL;

static struct {
	int family;
	union {
		struct sockaddr_in6 ip6;
		struct sockaddr_in ip4;
	} ip;
} dest;

static int change_ip(const char *val) {
	const char *p;
	int ret;
	int family;
	printk("changing: %s\n", val);

	family = AF_INET;
	p = val;
	while(*p) {
		if(*p == ':') {
			family = AF_INET6;
			break;
		}
		p++;
	}

	if(sock) {
		sock_release(sock);
		sock = NULL;
	}

	ret = sock_create(family, SOCK_DGRAM, IPPROTO_UDP, &sock);
	if(ret < 0) {
		return ret;
	}

	if(family == AF_INET6) {
		if(!in6_pton(val, -1, dest.ip.ip6.sin6_addr.s6_addr, -1, NULL)) {
			return -EADDRNOTAVAIL;
		}
		dest.ip.ip6.sin6_family = AF_INET6;
		dest.ip.ip6.sin6_port = htons(port);
	} else if(family == AF_INET) {
		if(!in4_pton(val, -1, (char*) &dest.ip.ip4.sin_addr.s_addr, -1, NULL)) {
			return -EADDRNOTAVAIL;
		}
		dest.ip.ip4.sin_family = AF_INET;
		dest.ip.ip4.sin_port = htons(port);

		if(strcmp(val, "255.255.255.255")) {
			sock_set_flag(sock->sk, SOCK_BROADCAST);
		}	
	} else {
		return -ENOTSUPP;
	}
	dest.family = family;

	return ret;
}

static int ops_ip_set(const char *val, const struct kernel_param *kp) {
	return change_ip(val);
}

static struct kernel_param_ops ops_ip = {
	.get = &param_get_int,
	.set = &ops_ip_set,
};
module_param_cb(ip, &ops_ip, &ip, 0600);

static int param_ops_port_set(const char *val, const struct kernel_param *kp) {
	int ret;
	ret = param_set_ushort(val, kp);
	if(ret) {
		return ret;
	}

	if(dest.family == AF_INET6) {
		dest.ip.ip6.sin6_port = htons(port);
	} else if(dest.family == AF_INET) {
		dest.ip.ip4.sin_port = htons(port);
	} else {
		return -ENOTSUPP;
	}

	return 0;
}

static struct kernel_param_ops param_ops_port = {
	.get = &param_get_ushort,
	.set = &param_ops_port_set,
};
module_param_cb(port, &param_ops_port, &port, 0600);


static int rpmsg_udp_probe(struct rpmsg_device *rpdev) {
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n", rpdev->src, rpdev->dst);
	return 0;
}

static int rpmsg_udp_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src) {
	struct msghdr msg;
	struct iov_iter iov_iter;
	struct kvec kvec;
	int ret;

	kvec.iov_base = data;
	kvec.iov_len = len;

	iov_iter.type = ITER_KVEC;
	iov_iter.iov_offset = 0;
	iov_iter.count = kvec.iov_len;
	iov_iter.kvec = &kvec;
	iov_iter.nr_segs = 1;

	kernel_param_lock(THIS_MODULE);

	msg.msg_name = &dest.ip;
	msg.msg_namelen = sizeof(dest.ip);
	msg.msg_controllen = 0;
	msg.msg_flags = 0;
	msg.msg_iocb = 0;
	msg.msg_iter = iov_iter;

	ret = sock_sendmsg(sock, &msg);

	kernel_param_unlock(THIS_MODULE);

	return 0;
}

static void rpmsg_udp_remove(struct rpmsg_device *rpdev) {
	dev_info(&rpdev->dev, "rpmsg udp driver is removed\n");
}

static struct rpmsg_device_id rpmsg_driver_udp_id_table[] = {
	{ .name	= "udp" },
	{ /* sentinel */ },
};

static struct rpmsg_driver rpmsg_udp_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_udp_id_table,
	.probe		= rpmsg_udp_probe,
	.callback	= rpmsg_udp_cb,
	.remove		= rpmsg_udp_remove,
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_udp_id_table);


static int __init init(void) {
	if(!sock) {
		change_ip(ip);
	}
	return register_rpmsg_driver(&rpmsg_udp_driver);
}

static void __exit fini(void) {
	unregister_rpmsg_driver(&rpmsg_udp_driver);
}

module_init(init);
module_exit(fini);

MODULE_LICENSE("GPL v2");

