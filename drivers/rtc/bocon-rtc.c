/*========================================================================
*   Copyright (C) 2003-2021 Guangzhou Bocon Ltd. All rights reserved.
*   
*   Filename：   bocon-rtc.c
*   Author：     luhuadong
*   Create Date：2021年03月08日
*   Description：Read or write file /dev/bocon-rtc to get or set rtc time
*
========================================================================*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/rtc.h>

#define DEFAULT_DEVICENAME       CONFIG_RTC_BOCON_DEV_FAKE_NAME
#define DEFAULT_RTC_DEVICE       CONFIG_RTC_BOCON_DEV_REAL_NAME
#define RTC_DATA_SIZE            7

/* ioctl commands */
#define MEM_CLEAR                0x1

/* If you need alloc major number dynamically, please set BOCON_RTC_MAJOR to 0 */
#define BOCON_RTC_MAJOR          230

static int bocon_rtc_major = BOCON_RTC_MAJOR;
module_param(bocon_rtc_major, int, S_IRUGO);

struct bocon_rtc_dev {
    struct cdev cdev;
    unsigned int data[RTC_DATA_SIZE];
};

static struct bocon_rtc_dev *bocon_rtc_devp;

static struct class  *bocon_rtc_class;
static struct device *bocon_rtc_class_devs;

static int bocon_rtc_open(struct inode *inode, struct file *filp)
{
    filp->private_data = bocon_rtc_devp;
    printk(KERN_INFO "bocon-rtc is opened\n");
    return 0;
}

static int bocon_rtc_release(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "bocon-rtc is release\n");
    return 0;
}

static long bocon_rtc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct bocon_rtc_dev *dev = filp->private_data;

    switch (cmd) {
    case MEM_CLEAR:
        memset(dev->data, 0, sizeof(dev->data));
        printk(KERN_INFO "clear bocon rtc memory %lu bytes\n", sizeof(dev->data));
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static ssize_t bocon_rtc_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
    /* not support ppos */

    int err = -ENODEV;
    struct rtc_time tm;
    unsigned int count = size;

    struct bocon_rtc_dev *dev = filp->private_data;
    struct rtc_device    *rtc = rtc_class_open(DEFAULT_RTC_DEVICE);

    if (!rtc) {
        pr_info("unable to open rtc device (%s)\n", DEFAULT_RTC_DEVICE);
        return err;
    }

    err = rtc_read_time(rtc, &tm);
    if (err) {
        dev_err(rtc->dev.parent, "unable to read the hardware clock\n");
        rtc_class_close(rtc);
        return err;
    }

    dev->data[0] = tm.tm_sec;
    dev->data[1] = tm.tm_min;
    dev->data[2] = tm.tm_hour;
    dev->data[3] = tm.tm_mday;
    dev->data[4] = tm.tm_mon + 1;
    dev->data[5] = tm.tm_year + 1900;
    dev->data[6] = tm.tm_wday;

    if (size > sizeof(dev->data))  /* check length */
        count = sizeof(dev->data);

    if (copy_to_user(buf, dev->data, count))
        return -EFAULT;

    printk(KERN_INFO "read %u bytes: %d-%d-%d, %02d:%02d:%02d, %d\n", count, 
            dev->data[5], dev->data[4], dev->data[3], dev->data[2], 
            dev->data[1], dev->data[0], dev->data[6]);

    return count;
}

static ssize_t bocon_rtc_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
    /* not support ppos */

    int err = -ENODEV;
    struct rtc_time tm;
    unsigned int count = size;

    struct bocon_rtc_dev *dev = filp->private_data;
    struct rtc_device    *rtc = rtc_class_open(DEFAULT_RTC_DEVICE);

    if (!rtc) {
        pr_info("unable to open rtc device (%s)\n", DEFAULT_RTC_DEVICE);
        return err;
    }

    if (!rtc->ops || !rtc->ops->set_time) {
        rtc_class_close(rtc);
        return err;
    }

    if (size > sizeof(dev->data))  /* check length */
        count = sizeof(dev->data);

    if (copy_from_user(dev->data, buf, count)) {
        return -EFAULT;
    }

    tm.tm_sec  = dev->data[0];
    tm.tm_min  = dev->data[1];
    tm.tm_hour = dev->data[2];
    tm.tm_mday = dev->data[3];
    tm.tm_mon  = dev->data[4] - 1;
    tm.tm_year = dev->data[5] - 1900;
    tm.tm_wday = dev->data[6];

    err = rtc_set_time(rtc, &tm);
    rtc_class_close(rtc);

    if (err < 0) {
        printk(KERN_INFO "write %u bytes failed\n", count);
        return err;
    }
    else {
        printk(KERN_INFO "write %u bytes\n", count);
        return count;
    }
}

static const struct file_operations bocon_rtc_fops = {
    .owner   = THIS_MODULE,
    .read    = bocon_rtc_read,
    .write   = bocon_rtc_write,
    .unlocked_ioctl = bocon_rtc_ioctl,
    .open    = bocon_rtc_open,
    .release = bocon_rtc_release,
};

static int __init bocon_rtc_init(void)
{
    int ret;

    bocon_rtc_devp = kzalloc(sizeof(struct bocon_rtc_dev), GFP_KERNEL);
    if (!bocon_rtc_devp) {
        printk(KERN_NOTICE "Error alloc bocon-rtc memory");
        return -ENOMEM;
    }

    /* register char device number */

    if (bocon_rtc_major) {
        ret = register_chrdev(bocon_rtc_major, DEFAULT_DEVICENAME, &bocon_rtc_fops);  /* return zero on success */
    } else {
        ret = register_chrdev(0, DEFAULT_DEVICENAME, &bocon_rtc_fops);  /* return a major a number on success */
    }
    
    if (ret < 0) {
        printk(KERN_NOTICE "Error register chrdev number for bocon-rtc");
        return ret;
    }
    else if (ret > 0) {
        bocon_rtc_major = ret;  /* update major number */
    }

    /* 创建设备信息，执行后会出现 /sys/class/bocon-rtc */
    bocon_rtc_class = class_create(THIS_MODULE, DEFAULT_DEVICENAME);

    /* 创建设备节点 /dev/bocon-rtc，就是根据上面的设备信息来的 */
    bocon_rtc_class_devs = device_create(bocon_rtc_class, NULL, MKDEV(bocon_rtc_major, 0), NULL, DEFAULT_DEVICENAME);

    printk("bocon-rtc init, major = %d\n", bocon_rtc_major);
    return 0;
}

static void __exit bocon_rtc_exit(void)
{
    if (bocon_rtc_class_devs)
        device_destroy(bocon_rtc_class, MKDEV(bocon_rtc_major, 0));

    if (bocon_rtc_class)
        class_destroy(bocon_rtc_class);

    unregister_chrdev(bocon_rtc_major, DEFAULT_DEVICENAME);

    if (bocon_rtc_devp)
        kfree(bocon_rtc_devp);

    printk("bocon-rtc exit\n");
}

module_init(bocon_rtc_init);
module_exit(bocon_rtc_exit);

MODULE_AUTHOR("luhuadong");
MODULE_LICENSE("GPL");
