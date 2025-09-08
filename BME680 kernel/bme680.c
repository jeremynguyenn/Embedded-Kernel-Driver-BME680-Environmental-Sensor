// SPDX-License-Identifier: GPL-2.0
/*
 * Advanced IIO Driver for Bosch BME680 with File Ops, Memory Management,
 * Process Management, Signals, POSIX Threads, Synchronization, and IPC
 * Based on official kernel driver and user-space files
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/events.h>
#include "bme680.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Advanced IIO Driver for BME680 with File Ops, Threads, IPC");
MODULE_VERSION("3.0");

static int heater_temp = 300; // Default module param
module_param(heater_temp, int, 0644);
MODULE_PARM_DESC(heater_temp, "Heater temperature in °C");

static dev_t bme680_dev_num;
static struct cdev bme680_cdev;
static struct class *bme680_class;
static struct device *bme680_device;

/* Regmap config */
const struct regmap_config bme680_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xEF,
    .cache_type = REGCACHE_RBTREE,
};

/* Compensation Functions (Port từ driver_bme680.c) */
static int bme680_compensate_temperature(struct bme680_data *data, s32 adc_temp, s32 *temp) {
    s64 var1, var2;

    var1 = ((s64)adc_temp) - ((s64)data->calib.par_t1 * 128);
    var2 = var1 * ((s64)data->calib.par_t2);
    var2 = var2 / 16384LL + ((s64)data->calib.par_t3 * var1 * var1 / 1024LL / 1024LL / 1024LL / 16LL);
    data->t_fine = (s32)(var2);
    *temp = (s32)(var2 / 5120LL); // micro °C
    return 0;
}

// Tương tự cho pressure, humidity, gas_resistance (port đầy đủ từ source)

/* Read Calib */
static int bme680_read_calib(struct bme680_data *data) {
    u8 buf[32];
    int ret;

    ret = regmap_bulk_read(data->regmap, 0xE1, buf, 32);
    if (ret)
        return ret;

    data->calib.par_t1 = (buf[31] << 8) | buf[30];
    // ... (parse đầy đủ như source)

    return 0;
}

/* Read Raw Data */
static int bme680_read_data(struct bme680_data *data, struct bme680_fifo_data *fdata) {
    u8 buf[15];
    s32 temp_raw, press_raw, hum_raw;
    u32 gas_raw;
    unsigned long flags;
    int ret;

    mutex_lock(&data->lock);
    spin_lock_irqsave(&data->reg_lock, flags);

    /* Trigger forced mode */
    ret = regmap_write(data->regmap, BME680_REG_CTRL_MEAS, 0x24 | (data->oversampling_temp << 5) | (data->oversampling_press << 2));
    if (ret)
        goto unlock;

    msleep(10);

    /* Wait EOC */
    ret = regmap_read_poll_timeout(data->regmap, BME680_REG_STATUS, ret, !(ret & 0x10), 1000, 10000);
    if (ret)
        goto unlock;

    ret = regmap_bulk_read(data->regmap, BME680_REG_TEMP_MSB, buf, 15);
    if (ret)
        goto unlock;

    temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    press_raw = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    hum_raw = (buf[6] << 8) | buf[7];
    gas_raw = (buf[13] << 2) | (buf[14] >> 6);

    ret = bme680_compensate_temperature(data, temp_raw, &fdata->temperature);
    if (ret)
        goto unlock;

    // Tương tự cho pressure, humidity, gas_resistance

    fdata->timestamp = ktime_get_ns();
    atomic_inc(&data->read_count);

unlock:
    if (ret)
        atomic_inc(&data->error_count);
    spin_unlock_irqrestore(&data->reg_lock, flags);
    mutex_unlock(&data->lock);
    wake_up_interruptible(&data->wait_data); // Signal
    return ret;
}

/* Kthread Polling (Process Management) */
static int bme680_poll_thread(void *arg) {
    struct bme680_data *data = arg;
    struct bme680_fifo_data fdata;

    while (!kthread_should_stop()) {
        if (!bme680_read_data(data, &fdata)) {
            down(&data->fifo_sem);
            kfifo_put(&data->data_fifo, fdata);
            up(&data->fifo_sem);
            iio_push_event(data->indio_dev, IIO_UNMOD_EVENT_CODE(IIO_RESISTANCE, 0, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_EITHER), fdata.timestamp);
        }
        msleep_interruptible(1000); // Poll mỗi 1s
    }
    return 0;
}

/* File Operations (Nâng cấp) */
static int bme680_open(struct inode *inode, struct file *file) {
    struct bme680_data *data = container_of(inode->i_cdev, struct bme680_data, cdev);
    file->private_data = data;
    return nonseekable_open(inode, file);
}

static int bme680_release(struct inode *inode, struct file *file) {
    return 0;
}

static ssize_t bme680_read(struct file *file, char __user *buf, size_t len, loff_t *off) {
    struct bme680_data *data = file->private_data;
    struct bme680_fifo_data fdata;
    int ret;

    if (file->f_flags & O_NONBLOCK) {
        ret = down_trylock(&data->fifo_sem);
        if (ret)
            return -EAGAIN;
    } else {
        ret = down_interruptible(&data->fifo_sem);
        if (ret)
            return ret;
        ret = wait_event_interruptible(data->wait_data, kfifo_len(&data->data_fifo) > 0);
        if (ret)
            goto unlock;
    }

    ret = kfifo_to_user(&data->data_fifo, buf, len, &len);
    up(&data->fifo_sem);
    return ret ? ret : len;

unlock:
    up(&data->fifo_sem);
    return ret;
}

static ssize_t bme680_write(struct file *file, const char __user *buf, size_t len, loff_t *off) {
    struct bme680_data *data = file->private_data;
    struct bme680_gas_config gcfg;
    if (len != sizeof(gcfg))
        return -EINVAL;

    if (copy_from_user(&gcfg, buf, len))
        return -EFAULT;

    mutex_lock(&data->lock);
    data->heater_temp = gcfg.heater_temp;
    data->heater_dur = gcfg.heater_dur;
    data->preheat_curr_ma = gcfg.preheat_curr_ma;
    // Update registers (từ bme680_set_gas_heater)
    mutex_unlock(&data->lock);
    return len;
}

static loff_t bme680_llseek(struct file *file, loff_t off, int whence) {
    return no_llseek(file, off, whence); // Không hỗ trợ seek cho FIFO
}

static unsigned int bme680_poll(struct file *file, struct poll_table_struct *pt) {
    struct bme680_data *data = file->private_data;
    poll_wait(file, &data->wait_data, pt);
    return kfifo_len(&data->data_fifo) > 0 ? (POLLIN | POLLRDNORM) : 0;
}

static long bme680_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct bme680_data *data = file->private_data;
    struct bme680_fifo_data fdata;
    int ret;

    switch (cmd) {
    case BME680_IOC_SET_GAS_CONFIG:
        return bme680_write(file, (const char __user *)arg, sizeof(struct bme680_gas_config), NULL);
    case BME680_IOC_READ_FIFO:
        ret = bme680_read_data(data, &fdata);
        if (ret)
            return ret;
        if (copy_to_user((void __user *)arg, &fdata, sizeof(fdata)))
            return -EFAULT;
        return 0;
    case BME680_IOC_GET_SHM:
        if (copy_to_user((void __user *)arg, &data->shm_buffer, sizeof(void *)))
            return -EFAULT;
        return 0;
    default:
        return -ENOTTY;
    }
}

static const struct file_operations bme680_fops = {
    .owner = THIS_MODULE,
    .open = bme680_open,
    .release = bme680_release,
    .read = bme680_read,
    .write = bme680_write,
    .llseek = bme680_llseek,
    .poll = bme680_poll,
    .unlocked_ioctl = bme680_ioctl,
};

/* Debugfs Stats */
static int bme680_debugfs_show(struct seq_file *s, void *unused) {
    struct bme680_data *data = s->private;
    seq_printf(s, "Reads: %d\nErrors: %d\nThread PID: %d\nUptime: %lu jiffies\n",
               atomic_read(&data->read_count), atomic_read(&data->error_count),
               task_pid_nr(data->poll_thread), jiffies - data->start_time);
    return 0;
}

static int bme680_debugfs_open(struct inode *inode, struct file *file) {
    return single_open(file, bme680_debugfs_show, inode->i_private);
}

static const struct file_operations bme680_debugfs_fops = {
    .owner = THIS_MODULE,
    .open = bme680_debugfs_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

/* IIO Read Raw */
static int bme680_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
    struct bme680_data *data = iio_priv(indio_dev);
    struct bme680_fifo_data fdata;
    int ret;

    pm_runtime_get_sync(data->dev);

    ret = bme680_read_data(data, &fdata);
    if (ret)
        goto out;

    switch (chan->type) {
    case IIO_TEMP:
        *val = fdata.temperature;
        return IIO_VAL_INT;
    case IIO_PRESSURE:
        *val = fdata.pressure / 1000;
        *val2 = (fdata.pressure % 1000) * 1000;
        return IIO_VAL_INT_PLUS_MICRO;
    case IIO_HUMIDITYRELATIVE:
    case IIO_RESISTANCE:
        // Tương tự
        break;
    default:
        ret = -EINVAL;
    }

out:
    pm_runtime_mark_last_busy(data->dev);
    pm_runtime_put_autosuspend(data->dev);
    return ret;
}

/* IIO Probe Core */
static int bme680_core_probe(struct device *dev, struct regmap *regmap, const char *name, void *bus_data) {
    struct bme680_data *data;
    struct iio_dev *indio_dev;
    int ret;

    /* Memory Management: kmalloc cho indio_dev */
    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->indio_dev = indio_dev;
    data->regmap = regmap;
    data->dev = dev;
    mutex_init(&data->lock);
    spin_lock_init(&data->reg_lock);
    init_waitqueue_head(&data->wait_data);
    sema_init(&data->fifo_sem, 1);
    INIT_KFIFO(data->data_fifo);
    atomic_set(&data->read_count, 0);
    atomic_set(&data->error_count, 0);
    data->start_time = jiffies;

    /* Shared Memory Allocation */
    data->shm_size = PAGE_SIZE;
    data->shm_buffer = devm_kzalloc(dev, data->shm_size, GFP_KERNEL);
    if (!data->shm_buffer)
        return -ENOMEM;
    memcpy(data->shm_buffer, &data->calib, sizeof(data->calib)); // Copy calib vào shm

    /* Check chip ID */
    u8 chip_id;
    ret = regmap_read(regmap, BME680_REG_CHIP_ID, &chip_id);
    if (ret || chip_id != BME680_CHIP_ID_VAL)
        return ret ? ret : -ENODEV;

    /* Read calib */
    ret = bme680_read_calib(data);
    if (ret)
        return ret;

    /* Reset */
    regmap_write(regmap, BME680_REG_RESET, BME680_CMD_SOFTRESET);
    msleep(10);

    /* Default config */
    data->oversampling_temp = BME680_OVERSAMPLING_X1;
    data->oversampling_press = BME680_OVERSAMPLING_X1;
    data->oversampling_humid = BME680_OVERSAMPLING_X1;
    data->filter_coeff = BME680_FILTER_OFF;
    data->heater_temp = heater_temp;
    data->heater_dur = 150;
    data->gas_enable = true;

    /* Register char device */
    ret = alloc_chrdev_region(&bme680_dev_num, 0, 1, "bme680");
    if (ret)
        return ret;

    cdev_init(&bme680_cdev, &bme680_fops);
    ret = cdev_add(&bme680_cdev, bme680_dev_num, 1);
    if (ret)
        goto err_chrdev;

    bme680_class = class_create(THIS_MODULE, "bme680");
    if (IS_ERR(bme680_class)) {
        ret = PTR_ERR(bme680_class);
        goto err_cdev;
    }

    bme680_device = device_create(bme680_class, NULL, bme680_dev_num, NULL, "bme680");
    if (IS_ERR(bme680_device)) {
        ret = PTR_ERR(bme680_device);
        goto err_class;
    }

    /* IIO setup */
    indio_dev->name = name;
    indio_dev->channels = bme680_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme680_channels);
    indio_dev->info = &bme680_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;

    ret = iio_triggered_buffer_setup(indio_dev, NULL, bme680_trigger_handler, NULL);
    if (ret)
        goto err_device;

    /* Kthread (Process Management) */
    data->poll_thread = kthread_run(bme680_poll_thread, data, "bme680_poll");
    if (IS_ERR(data->poll_thread)) {
        ret = PTR_ERR(data->poll_thread);
        goto err_buffer;
    }

    /* Debugfs */
    data->debugfs_dir = debugfs_create_dir("bme680", NULL);
    debugfs_create_file("stats", 0444, data->debugfs_dir, data, &bme680_debugfs_fops);

    /* PM runtime */
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);
    pm_runtime_enable(dev);

    ret = iio_device_register(indio_dev);
    if (ret)
        goto err_thread;

    dev_info(dev, "BME680 Advanced IIO Driver Probed\n");
    return 0;

err_thread:
    kthread_stop(data->poll_thread);
err_buffer:
    iio_triggered_buffer_cleanup(indio_dev);
err_device:
    device_destroy(bme680_class, bme680_dev_num);
err_class:
    class_destroy(bme680_class);
err_cdev:
    cdev_del(&bme680_cdev);
err_chrdev:
    unregister_chrdev_region(bme680_dev_num, 1);
    return ret;
}

static int bme680_core_remove(struct device *dev) {
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct bme680_data *data = iio_priv(indio_dev);

    pm_runtime_disable(dev);
    kthread_stop(data->poll_thread);
    debugfs_remove_recursive(data->debugfs_dir);
    iio_device_unregister(indio_dev);
    iio_triggered_buffer_cleanup(indio_dev);
    device_destroy(bme680_class, bme680_dev_num);
    class_destroy(bme680_class);
    cdev_del(&bme680_cdev);
    unregister_chrdev_region(bme680_dev_num, 1);
    return 0;
}

/* IIO Info */
static const struct iio_info bme680_iio_info = {
    .driver_module = THIS_MODULE,
    .read_raw = bme680_read_raw,
};

/* I2C Driver */
static int bme680_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct regmap *regmap = devm_regmap_init_i2c(client, &bme680_regmap_config);
    if (IS_ERR(regmap))
        return PTR_ERR(regmap);
    return bme680_core_probe(&client->dev, regmap, id->name, client);
}

static int bme680_i2c_remove(struct i2c_client *client) {
    return bme680_core_remove(&client->dev);
}

static const struct i2c_device_id bme680_id[] = { { "bme680", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, bme680_id);

static const struct of_device_id bme680_of_match[] = {
    { .compatible = "bosch,bme680" },
    {},
};
MODULE_DEVICE_TABLE(of, bme680_of_match);

static struct i2c_driver bme680_i2c_driver = {
    .driver = {
        .name = "bme680_i2c",
        .of_match_table = bme680_of_match,
    },
    .probe = bme680_i2c_probe,
    .remove = bme680_i2c_remove,
    .id_table = bme680_id,
};
module_i2c_driver(bme680_i2c_driver);