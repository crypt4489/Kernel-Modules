#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include "./mpu6000.h"


#define DEVICE_NAME "mpu6000"
#define CLASS_NAME "mpu6000_class"
#define I2C_BUS_AVAILABLE 1
#define MPU_SLAVE_ADDRESS 0x68
#define FIFO_BUF_SIZE 1024


MODULE_AUTHOR("Drew Fletcher");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("A driver for a MPU 6000");
MODULE_SUPPORTED_DEVICE("NONE");

static struct i2c_adapter *mpu_6000_i2c_adapter = NULL; //adapter to i2c bus
static struct i2c_client *mpu_6000_i2c_client = NULL; //client struct for mpu 6000

static DECLARE_WAIT_QUEUE_HEAD(fifobuffer);
static DECLARE_WAIT_QUEUE_HEAD(fifoint);
static DECLARE_WAIT_QUEUE_HEAD(fifomutex);
static DECLARE_WAIT_QUEUE_HEAD(fifoready);

static struct task_struct *fifo_thread;

static int fifo_ready = 0;
static int fifo_mutex = 0;
static int ready_mutex = 0;

void fifo_lock(void);
void fifo_unlock(void);
void ready_lock(void);
void ready_unlock(void);
void fill_fifo_buffer(void);

static irqreturn_t mpu_irq_handler(int irq, void* dev_id)
{
    static unsigned long flags = 0;

    local_irq_restore(flags);

    fifo_ready = 1;

    wake_up_interruptible(&fifoint);

    return IRQ_HANDLED;
}


static int fifo_thread_fn(void* param)
{
    allow_signal(SIGKILL);

    while(!kthread_should_stop())
    {
        wait_event_interruptible(fifoint, fifo_ready == 1 || kthread_should_stop());

        if (kthread_should_stop())
        {
            break;
        }

        fifo_lock();

        fill_fifo_buffer();

        fifo_unlock();

        wake_up_interruptible(&fifobuffer); //tell poll that data is ready

        if (signal_pending(fifo_thread))
            break;
    }

    do_exit(0);
    return 0;
}

static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static long int device_ioctl(struct file *, unsigned int, unsigned long);
static unsigned int device_poll(struct file *, poll_table *);
static int device_map(struct file *, struct vm_area_struct *);

static struct file_operations fops =
{
    owner: THIS_MODULE,
	read: device_read,
	write: device_write,
	unlocked_ioctl: device_ioctl,
	poll: device_poll,
	open: device_open,
	release: device_release,
	mmap: device_map

};

static const struct vm_operations_struct mpu_mem_ops = {

};

static const struct i2c_device_id mpu_idtable[] = {
      { DEVICE_NAME, 0 },
      { }
};

MODULE_DEVICE_TABLE(i2c, mpu_idtable);


static struct i2c_driver mpu_driver = {
      .driver = {
              .name   = DEVICE_NAME,
              .owner = THIS_MODULE
      },
};

static struct i2c_board_info mpu_6000_board_info = {
    I2C_BOARD_INFO(DEVICE_NAME, MPU_SLAVE_ADDRESS)
};

struct mpu_file_struct
{
    int fifo_enable;
};

static struct class *mpu_class;
struct cdev *mpu_cdev;
dev_t deviceNumbers;

int read_temperature(void);
int read_xaccel(void);
int read_yaccel(void);
int read_zaccel(void);
int read_xgyro(void);
int read_ygyro(void);
int read_zgyro(void);
void write_mpu(u8 address, u8 val);
u8 read_mpu(u8 address);
void reset_mpu(void);
void request_irq_mpu(void);
void free_irq_mpu(void);
void enable_fifo_mpu(struct file *file);

static u8 *fifo_buffer;

static int __init init(void)
{
	int ret = alloc_chrdev_region(&deviceNumbers, 0, 1, DEVICE_NAME);
    u8 id;
	if (ret < 0) {
		pr_alert("Error registering: %d\n", ret);
		return -1;
	}


    if ((mpu_class = class_create(THIS_MODULE, CLASS_NAME)) == NULL)
    {
        pr_notice("Error creating class\n");
        unregister_chrdev_region(deviceNumbers, 1);
    }

    if ((device_create(mpu_class, NULL, deviceNumbers, NULL, DEVICE_NAME)) == NULL)
    {
        pr_notice("Cannot create device\n");
        class_destroy(mpu_class);
        unregister_chrdev_region(deviceNumbers, 1);
        return -1;
    }

    mpu_cdev = cdev_alloc();

	cdev_init(mpu_cdev, &fops);

	ret = cdev_add(mpu_cdev, deviceNumbers, 1);

	if (ret < 0)
	{
        pr_notice("Error %d adding mpu\n", ret);
        device_destroy(mpu_class, deviceNumbers);
        class_destroy(mpu_class);
        unregister_chrdev_region(deviceNumbers, 1);
    }

    mpu_6000_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

    if (mpu_6000_i2c_adapter != NULL)
    {
        mpu_6000_i2c_client = i2c_new_client_device(mpu_6000_i2c_adapter, &mpu_6000_board_info);
        if (mpu_6000_i2c_client != NULL)
        {
            if (i2c_add_driver(&mpu_driver) != -1)
            {
                ret = 0;
            }
            else
            {
                pr_notice("Can't add driver \n");
            }
        }
        i2c_put_adapter(mpu_6000_i2c_adapter);
    }

	id = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ID);

	pr_info("ID: 0x%x\n", id);

	reset_mpu();

	pr_info("Controller initialized (major number is %d)\n", MAJOR(deviceNumbers));

	return ret;
}

static void __exit cleanup(void)
{
    if (fifo_thread)
    {
        kthread_stop(fifo_thread);
        pr_info("Fifo thread is done\n");
    }

    i2c_unregister_device(mpu_6000_i2c_client);

    i2c_del_driver(&mpu_driver);

    cdev_del(mpu_cdev);

    device_destroy(mpu_class, deviceNumbers);

    class_destroy(mpu_class);

	unregister_chrdev_region(deviceNumbers, 1);

	pr_info("mpu module unloaded\n");
}

static int device_open(struct inode *inode, struct file *file)
{
    pr_info("mpu open\n");

    struct mpu_file_struct *mfs = kmalloc(sizeof (struct mpu_file_struct), GFP_KERNEL);

    mfs->fifo_enable = 0;

    file->private_data = mfs;

	return 0;
}


static int device_release(struct inode *inode, struct file *file)
{
	pr_info("mpu released\n");

    struct mpu_file_struct *mpu_file_data = (struct mpu_file_struct*)file->private_data;

    if (mpu_file_data->fifo_enable)
    {
        int irq = gpio_to_irq(24);
        free_irq(irq, NULL);
    }

    if (fifo_thread)
    {
        kthread_stop(fifo_thread);
        pr_info("Fifo thread is done\n");
    }

	if (fifo_buffer)
	{
        kfree(fifo_buffer);
        pr_info("Fifo buffer cleared\n");
    }

    if (file->private_data) kfree(file->private_data);

	return 0;
}


static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	pr_info("mpu write\n");

	return -EPERM;
}

static ssize_t device_read(struct file *filp, char *buff, size_t len, loff_t * off)
{
	pr_info("mpu read\n");

	return -EPERM;
}

static unsigned int device_poll(struct file *filp, poll_table *wait)
{
    pr_info("mpu poll\n");

    unsigned int mask = 0;

    poll_wait(filp, &fifobuffer, wait);

    pr_info("mpu: fifo ready!\n");

    mask |= POLLIN | POLLRDNORM;

    return mask;
}

static int device_map(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long size = vma->vm_end - vma->vm_start;
    vma->vm_ops = &mpu_mem_ops;

    vma->vm_pgoff = virt_to_phys((void*)fifo_buffer)>>PAGE_SHIFT;

    if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size, vma->vm_page_prot))
    {
        return -EAGAIN;
    }

    printk(KERN_INFO "VMA Open. Virt_addr: %lx, phy_addr: %lx\n", vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT);

    return 0;
}

static long int device_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    pr_info("mpu IOCTL\n");

    struct mpu_request *request;

    int ret = 0;
    int val;

    switch(ioctl_num)
    {
    case IOCTL_MPU_WRITE:
        request = kmalloc(sizeof (struct mpu_request), GFP_KERNEL);
        copy_from_user(request, (struct mpu_request*)ioctl_param, sizeof(struct mpu_request));
        write_mpu(request->address, request->value);
        kfree(request);
        break;
    case IOCTL_MPU_READ:
        request = kmalloc(sizeof (struct mpu_request), GFP_KERNEL);
        copy_from_user(request, (struct mpu_request*)ioctl_param, sizeof(struct mpu_request));
        request->value = read_mpu(request->address);
        copy_to_user((struct mpu_request*)ioctl_param, request, sizeof(struct mpu_request));
        kfree(request);
        break;
    case IOCTL_MPU_TEMP:
        val = read_temperature();
        copy_to_user((int*)ioctl_param, &val, sizeof(int));
        break;
    case IOCTL_MPU_RESET:
        reset_mpu();
        break;
    case IOCTL_MPU_ACCEL_XOUT:
        val = read_xaccel();
        copy_to_user((int*)ioctl_param, &val, sizeof(int));
        break;
    case IOCTL_MPU_ACCEL_YOUT:
        val = read_yaccel();
        copy_to_user((int*)ioctl_param, &val, sizeof(int));
        break;
    case IOCTL_MPU_ACCEL_ZOUT:
        val = read_zaccel();
        copy_to_user((int*)ioctl_param, &val, sizeof(int));
        break;
    case IOCTL_MPU_ENABLE_FIFO:
        request_irq_mpu();
        enable_fifo_mpu(file);
        break;
    case IOCTL_MPU_FIFO_LOCK:
        fifo_lock();
        break;
    case IOCTL_MPU_FIFO_UNLOCK:
        fifo_unlock();
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;

}

void fill_fifo_buffer(void)
{
    u8 fifo_c_h = read_mpu(FIFO_COUNT_H);
    u8 fifo_c_l = read_mpu(FIFO_COUNT_L);

    s16 count = (fifo_c_h << 8) | fifo_c_l;
    s16 i = 0;

    while(i < count)
    {
        fifo_buffer[i] = i2c_smbus_read_byte_data(mpu_6000_i2c_client, FIFO_R_W);
        i++;
    }

    write_mpu(USER_CONTROL, 0x00);
    mdelay(100);
    write_mpu(USER_CONTROL, 0x04);
    mdelay(100);
    write_mpu(USER_CONTROL, 0x40);
    mdelay(100);
    write_mpu(FIFO_EN, 0x80);

}


void fifo_lock(void)
{
    spin_lock(&fifomutex.lock);
    wait_event_interruptible(fifomutex, fifo_mutex == 0);
    fifo_mutex = 1;
    spin_unlock(&fifomutex.lock);
}

void fifo_unlock(void)
{
    spin_lock(&fifomutex.lock);
    fifo_mutex = 0;
    wake_up(&fifomutex);
    spin_unlock(&fifomutex.lock);
}

void ready_lock(void)
{
    spin_lock(&fifoready.lock);
    wait_event_interruptible(fifoready, ready_mutex == 0);
    ready_mutex = 1;
    spin_unlock(&fifoready.lock);
}

void ready_unlock(void)
{
    spin_lock(&fifoready.lock);
    ready_mutex = 0;
    wake_up(&fifoready);
    spin_unlock(&fifoready.lock);
}

void enable_fifo_mpu(struct file *file)
{
    struct mpu_file_struct *mpu_file_data = (struct mpu_file_struct*)file->private_data;

    mpu_file_data->fifo_enable = 1;

    fifo_buffer = kmalloc(PAGE_SIZE * sizeof(u8), GFP_KERNEL);

    write_mpu(USER_CONTROL, 0x40);

    write_mpu(FIFO_EN, 0x80); //enable temp fifo

    fifo_thread = kthread_run(fifo_thread_fn, NULL, "MPU_FIFO_THREAD");

	if (fifo_thread)
	{
        pr_info("Thread created successfully\n");
	}
	else
	{
        pr_info("Failed to create thread\n");
	}
}

int read_temperature(void)
{
    int val = 0;
    u8 temp_high, temp_low;

    s16 temp_short;

    temp_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, TEMP_OUT_H);
    temp_low =  i2c_smbus_read_byte_data(mpu_6000_i2c_client, TEMP_OUT_L);

    pr_info("low temp: 0x%x\n", temp_low);

    pr_info("high temp: 0x%x\n", temp_high);

    temp_short = (temp_high << 8) | temp_low;

    val = temp_short;

    return val;
}

int read_xaccel(void)
{
    int val = 0;
    u8 xaccel_high, xaccel_low;

    s16 xaccel_short;

    xaccel_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_XOUT_H);
    xaccel_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_XOUT_L);

    pr_info("low xacc: 0x%x\n", xaccel_low);

    pr_info("high xacc: 0x%x\n",xaccel_high);

    xaccel_short = (xaccel_high << 8) | xaccel_low;

    val = xaccel_short;

    return val;
}

int read_yaccel(void)
{
    int val = 0;
    u8 yaccel_high, yaccel_low;

    s16 yaccel_short;

    yaccel_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_YOUT_H);
    yaccel_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_YOUT_L);

    pr_info("low yacc: 0x%x\n", yaccel_low);

    pr_info("high yacc: 0x%x\n", yaccel_high);

    yaccel_short = (yaccel_high << 8) | yaccel_low;

    val = yaccel_short;

    return val;
}

int read_zaccel(void)
{
    int val = 0;
    u8 zaccel_high, zaccel_low;

    s16 zaccel_short;

    zaccel_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_ZOUT_H);
    zaccel_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, ACCEL_ZOUT_L);

    pr_info("low zacc: 0x%x\n", zaccel_low);

    pr_info("high zacc: 0x%x\n", zaccel_high);

    zaccel_short = (zaccel_high << 8) | zaccel_low;

    val = zaccel_short;

    return val;
}

int read_xgyro(void)
{
    int val = 0;
    u8 xgyro_high, xgyro_low;

    s16 xgyro_short;

    xgyro_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_XOUT_H);
    xgyro_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_XOUT_L);

    pr_info("low xgyro: 0x%x\n", xgyro_low);

    pr_info("high xgyro: 0x%x\n", xgyro_high);

    xgyro_short = (xgyro_high << 8) | xgyro_low;

    val = xgyro_short;

    return val;
}

int read_ygyro(void)
{
    int val = 0;
    u8 ygyro_high, ygyro_low;

    s16 ygyro_short;

    ygyro_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_YOUT_H);
    ygyro_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_YOUT_L);

    pr_info("low ygyro: 0x%x\n", ygyro_low);

    pr_info("high ygyro: 0x%x\n", ygyro_high);

    ygyro_short = (ygyro_high << 8) | ygyro_low;

    val = ygyro_short;

    return val;
}

int read_zgyro(void)
{
    int val = 0;
    u8 zgyro_high, zgyro_low;

    s16 zgyro_short;

    zgyro_high = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_ZOUT_H);
    zgyro_low = i2c_smbus_read_byte_data(mpu_6000_i2c_client, GYRO_ZOUT_L);

    pr_info("low zgyro: 0x%x\n", zgyro_low);

    pr_info("high zgyro: 0x%x\n", zgyro_high);

    zgyro_short = (zgyro_high << 8) | zgyro_low;

    val = zgyro_short;

    return val;
}

u8 read_mpu(u8 address)
{
    u8 val = i2c_smbus_read_byte_data(mpu_6000_i2c_client, address);

    return val;
}

void write_mpu(u8 address, u8 val)
{
    i2c_smbus_write_byte_data(mpu_6000_i2c_client, address, val);
}

void request_irq_mpu(void)
{
    int irq = gpio_to_irq(24);
    if (request_irq(irq, (void *)mpu_irq_handler, IRQF_TRIGGER_RISING, DEVICE_NAME, NULL))
    {
        pr_err("mpu: cannot register irq\n");
    }
}

void reset_mpu(void)
{
    i2c_smbus_write_byte_data(mpu_6000_i2c_client, PWR_MGMT_1, 0x80);

	mdelay(500);

	i2c_smbus_write_byte_data(mpu_6000_i2c_client, PWR_MGMT_1, 0x00);

	mdelay(500);

	i2c_smbus_write_byte_data(mpu_6000_i2c_client, CONFIG, 0x04);

	mdelay(500);
}


module_init(init);
module_exit(cleanup);


