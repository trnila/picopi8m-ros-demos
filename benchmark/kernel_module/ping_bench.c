/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * derived from the omap-rpmsg implementation.
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
#include <linux/time.h>
#include <linux/proc_fs.h>

#define PROCFS_NAME     "ping_benchmark"
#define COUNT           10000

static unsigned int rpmsg_pingpong;
static struct timespec start;
static uint64_t response_time[COUNT];
static int failed;
static int running;

static uint64_t timespec_diff_ns(struct timespec *t1, struct timespec *t2)
{
	struct timespec diff;
	if (t2->tv_nsec - t1->tv_nsec < 0) {
		diff.tv_sec  = t2->tv_sec - t1->tv_sec - 1;
		diff.tv_nsec = t2->tv_nsec - t1->tv_nsec + 1000000000UL;
	} else {
		diff.tv_sec  = t2->tv_sec - t1->tv_sec;
		diff.tv_nsec = t2->tv_nsec - t1->tv_nsec;
	}
	return (diff.tv_sec * 1000000000UL + diff.tv_nsec);
}

static int rpmsg_pingpong_cb(struct rpmsg_device *rpdev, void *data, int len,
						void *priv, u32 src)
{
	int err;
	struct timespec end;
	unsigned int recv;
	int index;

	/* capture receive time */
	getnstimeofday(&end);
	index = rpmsg_pingpong / 2 - 1;

	if(index >= 0 && index < COUNT) {
		response_time[index] = timespec_diff_ns(&start, &end);

		/* check received ping */
		recv = *(unsigned int *)data;
		if(recv != rpmsg_pingpong + 1) {
			printk("received invalid pong: %d, expected: %d\n", recv, rpmsg_pingpong);
			failed = 1;
			return 0;
		}
		rpmsg_pingpong = recv;
	}

	/* pingpongs should not live forever */
	if (rpmsg_pingpong >= 2 * COUNT) {
		dev_info(&rpdev->dev, "Measurement done\n");
		running = 0;
		return 0;
	}
	rpmsg_pingpong++;

	/* capture current time and send ping to m4 */
	getnstimeofday(&start);
	err = rpmsg_sendto(rpdev->ept, (void *)(&rpmsg_pingpong), 4, src);

	if (err)
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);

	return err;
}

static int rpmsg_pingpong_probe(struct rpmsg_device *rpdev)
{
	int err;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	running = 1;

	/* kick first message without measuring */
	rpmsg_pingpong = 0;
	err = rpmsg_sendto(rpdev->ept, (void *)(&rpmsg_pingpong), 4, rpdev->dst);
	if (err) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", err);
		return err;
	}

	return 0;
}

static void rpmsg_pingpong_remove(struct rpmsg_device *rpdev)
{
	running = 0;
	dev_info(&rpdev->dev, "rpmsg pingpong driver is removed\n");
}

static struct rpmsg_device_id rpmsg_driver_pingpong_id_table[] = {
	{ .name	= "rpmsg-openamp-demo-channel" },
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

static int show(struct seq_file* seq, void* priv) {
	int i;

	if(failed) {
		return -EIO;
	}

	if(running) {
		return -EBUSY;
	}

	for(i = 0; i < COUNT; i++) {
		seq_printf(seq, "%llu\n", response_time[i]);
	}
	return 0;
}

static int result_open(struct inode *inode, struct file* file) {
	return single_open(file, show, NULL);	
}

static struct file_operations result_file_ops = {
	.owner   = THIS_MODULE,
	.open    = result_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

static int __init init(void)
{
	int ret;
	ret = register_rpmsg_driver(&rpmsg_pingpong_driver);
	if(ret != 0) {
		return ret;
	}

	if(!proc_create(PROCFS_NAME, 0644, NULL, &result_file_ops)) {
		unregister_rpmsg_driver(&rpmsg_pingpong_driver);
		return -1;
	}

	return ret;
}

static void __exit fini(void)
{
	unregister_rpmsg_driver(&rpmsg_pingpong_driver);
	remove_proc_entry(PROCFS_NAME, NULL);
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("benchmark rpmsg ping response time");
MODULE_LICENSE("GPL v2");
