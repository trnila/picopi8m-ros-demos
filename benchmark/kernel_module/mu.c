#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/mx8_mu.h>
#include <linux/proc_fs.h>
#include "common.h"

#define DEBUG 1
#define COUNT 10000
#define PROCFS_NAME     "mu_benchmark"

static struct timespec start;
static uint64_t response_time[COUNT];
static int cur = -1;

struct priv_data {
	void __iomem *base;
	int irq;
};

static irqreturn_t irq_handler(int irq, void *dev_id) {
	uint32_t message;
	struct priv_data *priv = dev_id;
	static struct timespec end;


	if(!(MU_ReadStatus(priv->base) & (1 << 27))) {
		printk("invalid irq received!, status = %x\n", MU_ReadStatus(priv->base));
		return IRQ_HANDLED;
	}

	MU_ReceiveMsg(priv->base, 0, &message);
	getnstimeofday(&end);
	if(cur >= 0 && cur < COUNT) {
		response_time[cur] = timespec_diff_ns(&start, &end);
	}

	cur++;
	if(cur < COUNT) {
		getnstimeofday(&start);
		MU_SendMessage(priv->base, 0, message + 1);
	}

	return IRQ_HANDLED;
}

static int show(struct seq_file* seq, void* priv) {
	int i;

	printk("%d/%d\n", cur, COUNT);
	if(cur < COUNT) {
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

static int mu_bench_probe(struct platform_device *pdev) {
	struct priv_data *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(priv), GFP_KERNEL);
	if(!priv) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(priv->base)) {
		return PTR_ERR(priv->base);
	}

	priv->irq = platform_get_irq(pdev, 0);
	if(priv->irq < 0) {
		return priv->irq;
	}

	ret = devm_request_irq(&pdev->dev, priv->irq, irq_handler, 0, "mu", priv);
	if(ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return -ENODEV;
	}

	if(!proc_create(PROCFS_NAME, 0644, NULL, &result_file_ops)) {
		dev_err(&pdev->dev, "could not register procfs file\n");
		return -1;
	}

	MU_EnableRxFullInt(priv->base, 0);
//	MU_SendMessage(priv->base, 0, 42);
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
