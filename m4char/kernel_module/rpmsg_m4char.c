#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/rpmsg.h>
#include <linux/skbuff.h>

#define DEVICE_NAME "m4char"

/* maximal size of rpmsg buffer */
#define MAX_SIZE 512

/* 
 * maximal number of messages waiting for user to read
 * otherwise we can run out of memory
 */
#define MAX_AWAITING_MESSAGES 10000 

static int major;
static struct class* char_class  = NULL;
static struct device* char_device = NULL;

static spinlock_t queue_lock;
static struct sk_buff_head queue;
static wait_queue_head_t readq;

static struct rpmsg_device *rpmsg_device = NULL;
static struct mutex mtx;

static char* tx_buffer[MAX_SIZE];

static atomic_t dropped_count;

static int rpmsg_m4char_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	struct sk_buff *skb;
	uint32_t awaiting_messages;
	uint32_t dropped;

	spin_lock(&queue_lock);
	awaiting_messages = skb_queue_len(&queue);
	spin_unlock(&queue_lock);

	if(awaiting_messages >= MAX_AWAITING_MESSAGES) {
		dropped = atomic_inc_return(&dropped_count);
		printk_ratelimited("no available buffer, dropped rpmsg count %u\n", dropped);
		return -ENOBUFS;
	}

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

static int rpmsg_m4char_probe(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	rpmsg_device = rpdev;

	return 0;
}

static void rpmsg_m4char_remove(struct rpmsg_device *rpdev)
{
	mutex_lock(&mtx);
	rpmsg_device = NULL;
	mutex_unlock(&mtx);

	wake_up_interruptible(&readq);

	dev_info(&rpdev->dev, "rpmsg_m4char removed from channel\n");
}

static ssize_t rpmsg_m4char_char_read(struct file *filep, char *buf, size_t len, loff_t *offset){
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

static ssize_t rpmsg_m4char_char_write(struct file *filep, const char __user *buffer, size_t len, loff_t *offset) {
	int err;
	if(len >= MAX_SIZE) {
		return -EINVAL;
	}

	if(mutex_lock_interruptible(&mtx)) {
		return -ERESTARTSYS;
	}

	if(copy_from_user(tx_buffer, buffer, len)) {
		mutex_unlock(&mtx);
		return -EFAULT;
	}

	if(!rpmsg_device) {
		mutex_unlock(&mtx);
		return -EPIPE;
	}

	err = rpmsg_trysend(rpmsg_device->ept, tx_buffer, len);
	mutex_unlock(&mtx);
	return err == 0 ? len : err;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.read = rpmsg_m4char_char_read,
	.write = rpmsg_m4char_char_write,
};

static struct rpmsg_device_id rpmsg_driver_m4char_id_table[] = {
	{ .name	= "m4-channel" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_m4char_id_table);

static struct rpmsg_driver rpmsg_m4char_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_driver_m4char_id_table,
	.probe		= rpmsg_m4char_probe,
	.callback	= rpmsg_m4char_cb,
	.remove		= rpmsg_m4char_remove,
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

	return register_rpmsg_driver(&rpmsg_m4char_driver);
}

static void __exit fini(void)
{
	device_destroy(char_class, MKDEV(major, 0));
	class_unregister(char_class);
	class_destroy(char_class);
	unregister_chrdev(major, DEVICE_NAME);

	unregister_rpmsg_driver(&rpmsg_m4char_driver);
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("rpmsg char device for m4 communication");
MODULE_LICENSE("GPL v2");
