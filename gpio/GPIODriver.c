#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>

#include "./GPIODriver.h"

#define DEVICE_NAME "gpiochrdev"


MODULE_LICENSE("Dual BSD/GPL");


struct gpio_device_struct
{
    int Device_Open;
};

struct irq_struct
{
    struct list_head list;
    int irq_number;
    int output_pin;
};

static struct gpio_device_struct gpio_device = { 0 };
struct list_head irq_head;

extern unsigned long volatile jiffies;
unsigned long old_jiffies = 0;


static irqreturn_t gpio_irq_handler(int irq, void* dev_id)
{
    static unsigned long flags = 0;
    unsigned long diff = jiffies - old_jiffies;
    if (diff < 20)
    {
        return IRQ_HANDLED;
    }

    old_jiffies = jiffies;

    local_irq_save(flags);

    struct list_head *ptr;
	struct irq_struct *struct_ptr;

	for(ptr = irq_head.next; ptr != &irq_head; ptr = ptr->next)
	{
        struct_ptr = list_entry(ptr, struct irq_struct, list);
        if (struct_ptr->irq_number == irq)
        {
            pr_info("Irq handled\n");
            break;
        }
	}
    local_irq_restore(flags);

    return IRQ_HANDLED;
}


static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static long int device_ioctl(struct file *, unsigned int, unsigned long);

static struct file_operations fops =
{
    owner: THIS_MODULE,
	read: device_read,
	write:device_write,
	unlocked_ioctl: device_ioctl,
	open: device_open,
	release: device_release

};

struct gpio_file_struct
{
    int id;
    int active_pins;
    int openPins[MAX_GPIO_OFFSET];
};

static struct class *gpio_class;
struct cdev *gpio_cdev;
dev_t deviceNumbers;

LIST_HEAD(irq_head);

void free_irq_gpio(struct request* req_p);
int request_gpio(struct request* req, struct gpio_file_struct *fileStruct);
int free_gpio(struct request* req, struct gpio_file_struct *fileStruct);
void request_irq_gpio(struct request* req);
int read_gpio(struct request* req);
void read_multiple_gpio(struct request* req);
int write_multiple_gpio(struct request* req);
int write_gpio(struct request* req);

static int __init init(void)
{
	int ret = alloc_chrdev_region(&deviceNumbers, 0, 1, DEVICE_NAME);

	if (ret < 0) {
		pr_alert("Error registering: %d\n", ret);
		return -1;
	}


    if ((gpio_class = class_create(THIS_MODULE, "gpio_class")) == NULL)
    {
        pr_notice("Error creating class\n");
        unregister_chrdev_region(deviceNumbers, 1);
    }

    if ((device_create(gpio_class, NULL, deviceNumbers, NULL, "gpio_device")) == NULL)
    {
        pr_notice("Cannot create device\n");
        class_destroy(gpio_class);
        unregister_chrdev_region(deviceNumbers, 1);
        return -1;
    }

    gpio_cdev = cdev_alloc();

	cdev_init(gpio_cdev, &fops);

	ret = cdev_add(gpio_cdev, deviceNumbers, 1);

	if (ret < 0)
	{
        pr_notice("Error %d adding gpio\n", ret);
        device_destroy(gpio_class, deviceNumbers);
        class_destroy(gpio_class);
        unregister_chrdev_region(deviceNumbers, 1);
    }

	pr_info("Controller initialized (major number is %d)\n", MAJOR(deviceNumbers));

	return 0;
}

static void __exit cleanup(void)
{
    cdev_del(gpio_cdev);

    device_destroy(gpio_class, deviceNumbers);

    class_destroy(gpio_class);

	unregister_chrdev_region(deviceNumbers, 1);

	pr_info("GPIO Chip2 module unloaded\n");
}

static int device_open(struct inode *inode, struct file *file)
{
    pr_info("GPIO open\n");

    struct gpio_file_struct *fileStruct = kmalloc(sizeof (struct gpio_file_struct), GFP_KERNEL);

    gpio_device.Device_Open++;

    fileStruct->id = gpio_device.Device_Open;
    fileStruct->active_pins = 0;
    file->private_data = fileStruct;

	return 0;
}


static int device_release(struct inode *inode, struct file *file)
{
	pr_info("GPIO released\n");

	struct gpio_file_struct *fileStruct = (struct gpio_file_struct*)file->private_data;
	int iter = 0;

	while (iter < fileStruct->active_pins) \
	{
        gpio_free(fileStruct->openPins[iter]);
        iter++;
	}

	kfree(file->private_data);

	struct list_head *ptr;
	struct irq_struct *struct_ptr;

	for(ptr = irq_head.next; ptr != &irq_head; ptr = ptr->next)
	{
        struct_ptr = list_entry(ptr, struct irq_struct, list);
        kfree(struct_ptr);
	}

	gpio_device.Device_Open--;

	return 0;
}


static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	pr_info("GPIO write\n");
	return -EPERM;
}

static ssize_t device_read(struct file *filp, char *buff, size_t len, loff_t * off)
{
	pr_info("GPIO read\n");
	return -EPERM;
}

static long int device_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    pr_info("GPIO IOCTL\n");
    int ret = 0;
	struct request* req_p,* req_l;

    struct gpio_file_struct *fileStruct = (struct gpio_file_struct*)file->private_data;


	req_p = (struct request*)ioctl_param;
	req_l = kmalloc(sizeof (struct request), GFP_KERNEL);
	copy_from_user(req_l, req_p, sizeof(struct request));

    switch(ioctl_num)
    {
    case IOCTL_READ_LINE:
        ret = read_gpio(req_l);
        copy_to_user(req_p, req_l, sizeof(struct request));
        break;
    case IOCTL_READ_MULTIPLE_LINES:
        read_multiple_gpio(req_l);
        copy_to_user(req_p, req_l, sizeof(struct request));
        break;
    case IOCTL_WRITE_LINE:
        ret = write_gpio(req_l);
        break;
    case IOCTL_WRITE_MULTIPLE_LINES:
        ret = write_multiple_gpio(req_l);
        break;
    case IOCTL_DIRECTION_OUTPUT:
        ret = gpio_direction_output(req_l->pinNumber[0], req_l->val[0]);
        break;
    case IOCTL_DIRECTION_INPUT:
        ret = gpio_direction_input(req_l->pinNumber[0]);
        break;
    case IOCTL_REQUEST_GPIO:
        ret = request_gpio(req_l, fileStruct);
        break;
    case IOCTL_FREE_GPIO:
        ret = free_gpio(req_l, fileStruct);
        break;
    case IOCTL_REQUEST_IRQ_GPIO:
        request_irq_gpio(req_l);
        break;
    case IOCTL_FREE_IRQ_GPIO:
        free_irq_gpio(req_l);
        break;
    default:
        ret = -EINVAL;
    }
    kfree(req_l);
    return ret;
}

void free_irq_gpio(struct request* req)
{
    int irq_val = gpio_to_irq(req->pinNumber[0]);
    free_irq(irq_val, NULL);

    struct list_head *ptr;
    struct irq_struct *struct_ptr;

    for(ptr = irq_head.next; ptr != &irq_head; ptr = ptr->next)
    {
        struct_ptr = list_entry(ptr, struct irq_struct, list);
        if (irq_val == struct_ptr->irq_number)
        {
            list_del(&struct_ptr->list);
            kfree(struct_ptr);
        }
    }
}

void request_irq_gpio(struct request* req)
{
    if (gpio_is_valid(req->pinNumber[0]))
    {
        int gpio_irq_num = gpio_to_irq(req->pinNumber[0]);
        if (request_irq(gpio_irq_num, (void *)gpio_irq_handler, IRQF_TRIGGER_RISING, DEVICE_NAME, NULL))
        {
            pr_err("gpio: cannot register irq\n");
        }
        else
        {
            struct irq_struct *irqStruct =  kmalloc (sizeof( struct irq_struct), GFP_KERNEL);
            irqStruct->output_pin = req->pinNumber[1];
            irqStruct->irq_number = gpio_irq_num;
            list_add_tail(&irqStruct->list, &irq_head);
        }
    }
}

int free_gpio(struct request* req, struct gpio_file_struct *fileStruct)
{
    int ret = 0;
    int i = 0;

    if (gpio_is_valid(req->pinNumber[0]))
    {
        gpio_free(req->pinNumber[0]);
        while( i<fileStruct->active_pins)
        {
            if (fileStruct->openPins[i] == req->pinNumber[0])
            {
                fileStruct->openPins[i] = 0;
            }
            i++;
        }
        fileStruct->active_pins--;
    }
    else
    {
        ret = -EINVAL;
    }

    return ret;
}

int request_gpio(struct request* req, struct gpio_file_struct *fileStruct)
{
    int ret = 0;

    if (gpio_is_valid(req->pinNumber[0]))
    {
        char *gpio_string = kmalloc (sizeof(char)*10, GFP_KERNEL);
        snprintf(gpio_string, 10, "gpio%d", req->pinNumber[0]);
        pr_info("%s", gpio_string);
        if (gpio_request(req->pinNumber[0], gpio_string) == 0)
        {
            fileStruct->openPins[fileStruct->active_pins] = req->pinNumber[0];
            fileStruct->active_pins++;
        }
        else
        {
            ret = -EBUSY;
        }
        kfree(gpio_string);
    }
    else
    {
        ret = -EINVAL;
    }

    return ret;
}

int read_gpio(struct request* req)
{
    int ret = 0;

    if (gpio_is_valid(req->pinNumber[0]))
    {
        req->val[0] = gpio_get_value(req->pinNumber[0]);
    }
    else
    {
        ret = -EINVAL;
    }

    return ret;
}

void read_multiple_gpio(struct request* req)
{
    int iter = 0;
    while(iter<req->numOfPins)
    {
        if (gpio_is_valid(req->pinNumber[iter]))
        {
            req->val[iter] = gpio_get_value(req->pinNumber[iter]);
        }
        else
        {
            req->val[iter] = -1;
        }
        iter++;
    }
}

int write_gpio(struct request* req)
{
    int ret = 0;
    if (gpio_is_valid(req->pinNumber[0]))
    {
        gpio_set_value(req->pinNumber[0], req->val[0]);
    }
    else
    {
        ret = -EINVAL;
    }

    return ret;
}

int write_multiple_gpio(struct request* req)
{
    int ret = 0;
    int iter = 0;

    while(iter<req->numOfPins)
    {
        if (gpio_is_valid(req->pinNumber[iter]))
        {
            gpio_set_value(req->pinNumber[iter], req->val[iter]);
        }
        else
        {
            ret = -EINVAL;
        }
        iter++;
    }

    return ret;
}

module_init(init);
module_exit(cleanup);
