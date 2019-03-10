#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/mx8_mu.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include "common.h"

#define DEBUG       1
#define COUNT       10000
#define PROCFS_NAME "mu_bench"

struct priv_data {
	void __iomem *base;
	struct mutex mtx;
	wait_queue_head_t done;
	struct timespec start;
	uint64_t response_time[COUNT];
	atomic_t cur;
};

static irqreturn_t irq_handler(int irq, void *dev_id) {
	uint32_t message;
	struct priv_data *priv = dev_id;
	static struct timespec end;
	int cur;

	if(!(MU_ReadStatus(priv->base) & (1 << 27))) {
		printk("invalid irq received!, status = %x\n", MU_ReadStatus(priv->base));
		return IRQ_HANDLED;
	}

	MU_ReceiveMsg(priv->base, 0, &message);
	getnstimeofday(&end);

	cur = atomic_inc_return(&priv->cur);
	if(cur >= 0 && cur < COUNT) {
		priv->response_time[cur] = timespec_diff_ns(&priv->start, &end);
	}

	if(cur < COUNT) {
		getnstimeofday(&priv->start);
		MU_SendMessage(priv->base, 0, message + 1);
	} else if(cur == COUNT) {
		wake_up_interruptible(&priv->done);
	}

	return IRQ_HANDLED;
}

static int show(struct seq_file* seq, void* m) {
	int i;
	struct priv_data *priv = seq->private;

	mutex_lock(&priv->mtx);
	atomic_set(&priv->cur, -2); // skip first measurement

	// kick
	MU_SendMessage(priv->base, 0, 0);

	// wait until last ping
	if(wait_event_interruptible(priv->done, atomic_read(&priv->cur) >= COUNT)) {
		mutex_unlock(&priv->mtx);
		return -EIO;
	}

	for(i = 0; i < COUNT; i++) {
		seq_printf(seq, "%llu\n", priv->response_time[i]);
	}

	mutex_unlock(&priv->mtx);
	return 0;
}

static int result_open(struct inode *inode, struct file* file) {
	return single_open(file, show, PDE_DATA(inode));	
}

static struct file_operations result_file_ops = {
	.owner   = THIS_MODULE,
	.open    = result_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int mu_bench_probe(struct platform_device *pdev) {
	struct priv_data *priv;
	struct resource *res;
	int ret;
	int irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if(!priv) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(priv->base)) {
		return PTR_ERR(priv->base);
	}

	irq = platform_get_irq(pdev, 0);
	if(irq < 0) {
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, irq, irq_handler, 0, "mu", priv);
	if(ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return -ENODEV;
	}

	MU_Init(priv->base);
	MU_EnableRxFullInt(priv->base, 0);
	
	mutex_init(&priv->mtx);
	init_waitqueue_head(&priv->done);

	if(!proc_create_data(PROCFS_NAME, 0600, NULL, &result_file_ops, priv)) {
		dev_err(&pdev->dev, "could not register procfs file\n");
		return -1;
	}

	dev_info(&pdev->dev, "initialized");
	return 0;
}

static int mu_bench_remove(struct platform_device *pdev) {
	remove_proc_entry(PROCFS_NAME, NULL);
	return 0;
}

static const struct of_device_id mu_bench_dt_ids[] = {
	{ .compatible = "fsl,imx6sx-mu" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mu_bench_dt_ids);

static struct platform_driver mu_bench_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mu_bench",
		   .of_match_table = mu_bench_dt_ids,
		   },
	.probe = mu_bench_probe,
	.remove = mu_bench_remove,
};

static int __init init(void)
{
	return platform_driver_register(&mu_bench_driver); 
}

static void __exit fini(void)
{
	platform_driver_unregister(&mu_bench_driver);
}
module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("benchmark mu response time");
MODULE_LICENSE("GPL v2");
