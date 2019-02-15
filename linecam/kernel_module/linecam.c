#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/idr.h>

struct linecam_priv {
	struct spi_device *spi;
	struct gpio_desc *gpio_si;
	struct gpio_desc *gpio_clk;
	int char_minor;
};

#define DEVICE_NAME "linecam"

static int major;
static struct class* char_class  = NULL;
static struct device* char_device = NULL;

static struct mutex mtx;


static DEFINE_IDR(minors);

static int linecam_char_open(struct inode *inode, struct file *filp) {
	mutex_lock(&mtx);
	filp->private_data = idr_find(&minors, MINOR(inode->i_rdev));
	mutex_unlock(&mtx);
	BUG_ON(filp->private_data == NULL);
	return 0;
}

static ssize_t linecam_char_read(struct file *filep, char __user *buf, size_t len, loff_t *offset) {
	struct linecam_priv *priv;
	int err;
	int pixel;
	uint16_t buffer[128];
	uint16_t tx = 0;
	struct spi_transfer xfer;
	struct spi_message msg;

	priv = filep->private_data;
	BUG_ON(priv == NULL);

	if(len != sizeof(buffer)) {
		return -EINVAL;
	}

	err = mutex_lock_interruptible(&mtx);
	if(err) {
		return err;
	}

	gpiod_set_value(priv->gpio_clk, 0);
	gpiod_set_value(priv->gpio_si, 1);
	gpiod_set_value(priv->gpio_clk, 1);
	gpiod_set_value(priv->gpio_si, 0);

	for(pixel = 0; pixel < 128; pixel++) {
		gpiod_set_value(priv->gpio_clk, 0);

		memset(&xfer, 0, sizeof(xfer));
		xfer.tx_buf = &tx;
		xfer.rx_buf = buffer + pixel;
		xfer.len = 2;
		xfer.bits_per_word = 16;

		spi_message_init(&msg);
		spi_message_add_tail(&xfer, &msg);

		err = spi_sync(priv->spi, &msg);
		if(err) {
			printk("spi error");
		}

		gpiod_set_value(priv->gpio_clk, 1);
	}

	mutex_unlock(&mtx);

	if(copy_to_user(buf, buffer, sizeof(buffer))) {
		return -EFAULT;
	}

	return sizeof(buffer);
}


static int linecam_spi_probe(struct spi_device *spi) {
	struct device* dev = &spi->dev;
	struct linecam_priv *priv;

	// set SPI mode CPOL=1
	spi->mode = SPI_CPOL;

	// allocate our private data
	priv = devm_kzalloc(dev, sizeof(priv), GFP_KERNEL);
	if(!priv) {
		dev_err(dev, "failed to allocate memory");
		return -ENOMEM;
	}
	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	// request gpio lines from dts
	priv->gpio_si = devm_gpiod_get(dev, "linecam-si", GPIOD_OUT_LOW);
	if(IS_ERR(priv->gpio_si)) {
		return PTR_ERR(priv->gpio_si);
	}

	priv->gpio_clk = devm_gpiod_get(dev, "linecam-clk", GPIOD_OUT_LOW);
	if(IS_ERR(priv->gpio_clk)) {
		return PTR_ERR(priv->gpio_clk);
	}

	// allocate next minor number and assign private data so we can obtain it in open() 
	mutex_lock(&mtx);
	priv->char_minor = idr_alloc(&minors, priv, 0, MINORMASK + 1, GFP_KERNEL);
	if(priv->char_minor < 0) {
		mutex_unlock(&mtx);
		dev_err(dev, "could not allocate minor: %d", priv->char_minor);
		return priv->char_minor;
	}
	mutex_unlock(&mtx);

	// create device /dev/linecamX
	char_device = device_create(char_class, dev, MKDEV(major, priv->char_minor), priv, "linecam%d", priv->char_minor);
	if (IS_ERR(char_device)){
		idr_remove(&minors, priv->char_minor);
		dev_err(dev, "could not create device: %ld", PTR_ERR(char_device));
		return PTR_ERR(char_device);
	}
	
	return 0;
}

int linecam_spi_remove(struct spi_device *spi) {
	struct linecam_priv *priv;
	priv = spi_get_drvdata(spi);

	mutex_lock(&mtx);
	device_destroy(char_class, MKDEV(major, priv->char_minor));
	idr_remove(&minors, priv->char_minor);
	mutex_unlock(&mtx);

	return 0;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = linecam_char_open,
	.read = linecam_char_read,
};

static struct of_device_id match_table[] = {
     {.compatible = "linecam"},
     {}
};
MODULE_DEVICE_TABLE(of, match_table);

static struct spi_driver linecam_spi_driver = {
        .probe = linecam_spi_probe,
        .remove = linecam_spi_remove,
        .driver = {
                .name = "linecam",
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(match_table),
        },
};

static int __init init(void) {
	int err;
	major = register_chrdev(0, DEVICE_NAME, &fops);
	if(major < 0) {
		printk("unable to register_chrdev");
		return major;
	}

	char_class = class_create(THIS_MODULE, "linecam");
	if (IS_ERR(char_class)){
		printk("register error");
		return -1;
	}

	mutex_init(&mtx);

	err = spi_register_driver(&linecam_spi_driver);
	if(err != 0) {
		printk("could not register driver: %d\n", err);
	}

	return 0;
}

static void __exit fini(void) {
	spi_unregister_driver(&linecam_spi_driver);
	class_unregister(char_class);
	class_destroy(char_class);
	unregister_chrdev(major, DEVICE_NAME);
}

module_init(init);
module_exit(fini);
MODULE_DESCRIPTION("linecam example module");
MODULE_LICENSE("GPL v2");
