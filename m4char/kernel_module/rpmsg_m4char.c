/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * derived from the omap-rpmsg and rpmsg_char implementation.
 * Remote processor messaging transport - pingpong driver
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/rpmsg.h>
#include <linux/skbuff.h>

#define DEVICE_NAME "m4char"

static int major;
static struct class* char_class  = NULL;
static struct device* char_device = NULL;

static spinlock_t queue_lock;
static struct sk_buff_head queue;
static wait_queue_head_t readq;

static struct rpmsg_device *rpmsg_device = NULL;
static struct mutex mtx;

static int rpmsg_pingpong_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	int err;
	struct sk_buff *skb;

	((char*) data)[len] = 0;
	printk("ping response: '%s'\n", data);

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	memcpy(skb_put(skb, len), data, len);

	spin_lock(&queue_lock);
	skb_queue_tail(&queue, skb);
	spin_unlock(&queue_lock);

	/* wake up any blocking processes, waiting for new data */
	wake_up_interruptible(&readq);
	return 0;
}

static int rpmsg_pingpong_probe(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	rpmsg_device = rpdev;

	return 0;
}

static void rpmsg_pingpong_remove(struct rpmsg_device *rpdev)
{
	printk("releasing...\n");
	mutex_lock(&mtx);
	rpmsg_device = NULL;
	mutex_unlock(&mtx);

	wake_up_interruptible(&readq);

	dev_info(&rpdev->dev, "rpmsg pingpong driver is removed\n");
}

static int dev_open(struct inode *inodep, struct file *filep) {
	return 0;
}

static ssize_t dev_read(struct file *filep, char *buf, size_t len, loff_t *offset){
	unsigned long flags;
	struct sk_buff *skb;
	int use;

	spin_lock_irqsave(&queue_lock, flags);

	/* Wait for data in the queue */
	if (skb_queue_empty(&queue)) {
		spin_unlock_irqrestore(&queue_lock, flags);

		/* Wait until we get data or the endpoint goes away */
		if (wait_event_interruptible(readq, !skb_queue_empty(&queue) || !rpmsg_device))
			return -ERESTARTSYS;

		/* We lost the endpoint while waiting */
		if (!rpmsg_device) {
			return -EPIPE;
		}

		spin_lock_irqsave(&queue_lock, flags);
	}

	skb = skb_dequeue(&queue);
	spin_unlock_irqrestore(&queue_lock, flags);
	if (!skb)
		return -EFAULT;

	use = min_t(size_t, len, skb->len);
	if (copy_to_user(buf, skb->data, use))
		use = -EFAULT;

	kfree_skb(skb);
	return use;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset) {
	void *kbuf;

	if(mutex_lock_interruptible(&mtx)) {
		return -ERESTARTSYS;
	}

	if(!rpmsg_device) {
		mutex_unlock(&mtx);
		return -ENODEV;
	}

	kbuf = memdup_user(buffer, len + 1);
	if (IS_ERR(kbuf)) {
		mutex_unlock(&mtx);
		return PTR_ERR(kbuf);
	}
	((char*) kbuf)[len] = 0;
	printk("'%s'\n", kbuf);


	rpmsg_send(rpmsg_device->ept, kbuf, len);
	kfree(kbuf);
	mutex_unlock(&mtx);
	return len;
}

static int dev_release(struct inode *inodep, struct file *filep){
	return 0;
}

static struct file_operations fops =
{
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_release,
};


static struct rpmsg_device_id rpmsg_driver_pingpong_id_table[] = {
	{ .name	= "m4-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_pingpong_id_table);

static struct rpmsg_driver rpmsg_pingpong_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_pingpong_id_table,
	.probe		= rpmsg_pingpong_probe,
	.callback	= rpmsg_pingpong_cb,
	.remove		= rpmsg_pingpong_remove,
};

static int __init init(void)
{
	major = register_chrdev(0, DEVICE_NAME, &fops);
	if(major < 0) {
		printk("unable to register_chrdev");
		return major;
	}

	char_class = class_create(THIS_MODULE, "m4char");
	if (IS_ERR(char_class)){
		printk("register error");
		return -1;
	}

	char_device = device_create(char_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(char_device)){
		printk("char device error");
		return -1;
	}


	spin_lock_init(&queue_lock);
	skb_queue_head_init(&queue);
	init_waitqueue_head(&readq);

	mutex_init(&mtx);

	return register_rpmsg_driver(&rpmsg_pingpong_driver);
}

static void __exit fini(void)
{
	device_destroy(char_class, MKDEV(major, 0));
	class_unregister(char_class);
	class_destroy(char_class);
	unregister_chrdev(major, DEVICE_NAME);

	unregister_rpmsg_driver(&rpmsg_pingpong_driver);
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("rpmsg char device for m4 communication");
MODULE_LICENSE("GPL v2");
