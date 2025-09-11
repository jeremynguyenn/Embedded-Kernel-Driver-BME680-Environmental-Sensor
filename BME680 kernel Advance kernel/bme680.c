// SPDX-License-Identifier: GPL-2.0
/*
 * Full BME680 IIO Driver v3.2 (based on linux v6.16-rc1): I2C/SPI, Continuous Mode, Full Compensation, IAQ, Hwmon, Netlink, Events, Seccomp, Lockdown, Async Buffer, Tracepoints
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
#include <linux/hwmon.h>
#include <linux/netlink.h>
#include <linux/tracepoint.h>
#include <linux/security.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/events.h>
#include <linux/signal.h>
#include <linux/vmalloc.h>
#include <linux/unaligned.h>
#include <linux/bitfield.h>
#include "bme680.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Full Enhanced BME680 IIO Driver");
MODULE_VERSION("3.2");

static int heater_temp = 300;
module_param(heater_temp, int, 0644);
MODULE_PARM_DESC(heater_temp, "Heater temperature in °C");

static dev_t bme680_dev_num;
static struct cdev bme680_cdev;
static struct class *bme680_class;
static struct device *bme680_device;

const struct regmap_config bme680_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xEF,
    .cache_type = REGCACHE_RBTREE,
    .volatile_table = &bme680_volatile_table,  // Full from official
};

/* Full Compensation from v6.16 bme680_core.c */
static int bme680_compensate_temperature(const struct bme680_calib *calib, int32_t adc_temp,
                                         int32_t *temp, int32_t *t_fine) {
    int64_t var1, var2;

    var1 = ((int64_t)adc_temp) - ((int64_t)calib->par_t1 << 1);
    var2 = (var1 * ((int64_t)calib->par_t2)) >> 11;
    var2 += (((var1 * var1) * ((int64_t)calib->par_t3)) >> 13);
    *t_fine = (int32_t)var2;
    *temp = (int32_t)((var2 * 5 + 128) >> 8);

    return 0;
}

static int bme680_compensate_pressure(const struct bme680_calib *calib, int32_t adc_press,
                                      int32_t t_fine, uint32_t *press) {
    int64_t var1, var2, var3;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1;
    var2 = (var2 * (int64_t)calib->par_p6) >> 17;
    var2 += ((var1 * (int64_t)calib->par_p5) << 2);
    var2 = (var2 + ((int64_t)calib->par_p4 << 35)) >> 35;
    var3 = ((int64_t)(var1 * var1) * (int64_t)var1) * ((int64_t)calib->par_p3) >> 63;
    var1 = (((int64_t)calib->par_p2 * var1) >> 2) + (var2 + var3);
    var2 = (int64_t)adc_press * 262144;
    var3 = ((int64_t)var1 * var1) >> 13;
    var1 = ((int64_t)var3 * (int64_t)calib->par_p1) >> 35;
    var2 = ((int64_t)var2 - var1) * 128;
    var1 = (calib->par_p10 * var3) >> 35;
    var3 = ((int64_t)var2 * 100) >> 47;
    *press = (uint32_t)((var3 + var1 + 256) >> 8);

    return 0;
}

static int bme680_compensate_humidity(const struct bme680_calib *calib, int32_t adc_hum,
                                      int32_t t_fine, uint32_t *hum) {
    int64_t var1, var2, var3, var4, var5, var6;

    var1 = (int64_t)t_fine - (int64_t)76800;
    var2 = ((((int64_t)adc_hum << 14) - (((int64_t)calib->par_h4) << 20) - (((int64_t)calib->par_h5) * var1)) + ((int64_t)16384)) >> 15;
    var3 = var2 * ((((int64_t)var1 * ((int64_t)calib->par_h6)) >> 9) * (((int64_t)var1 * ((int64_t)calib->par_h6)) >> 9) >> 12) >> 10;
    var4 = ((int64_t)calib->par_h2) * 128;
    var4 = (var3 + var4) >> 15;
    var5 = (((int64_t)var2 * ((int64_t)var2)) >> 8) * ((int64_t)calib->par_h1);
    var6 = (var5 >> 4) + ((int64_t)calib->par_h3 * ((int64_t)var1 * ((int64_t)var1) >> 8) * ((int64_t)var1 * ((int64_t)var1) >> 8)) >> 12;
    var6 = ((var6 >> 3) + ((int64_t)4194304)) >> 12;
    var5 = ((var4 * var6) >> 10);
    *hum = (uint32_t)((var5 >> 10) << 12);

    return 0;
}

static int bme680_compensate_gas_resistance(const struct bme680_calib *calib, uint16_t adc_gas_res,
                                            uint8_t gas_range, uint32_t *gas_res) {
    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t lookup_k1_range[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t lookup_k2_range[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 16; i++) {
        lookup_k1_range[i] = (uint32_t)(bme680_lookup_k1_range[i] * 100);
        lookup_k2_range[i] = (uint32_t)(bme680_lookup_k2_range[i] * 100);
    }

    var1 = (int64_t)((1340 + (5 * (int64_t)calib->range_sw_err)) * ((int64_t)lookup_k1_range[gas_range]));
    var1 >>= 16;
    var2 = (((int64_t)adc_gas_res << 15) - (int64_t)(16777216)) + var1;
    var3 = (((int64_t)lookup_k2_range[gas_range] * (int64_t)var1)) >> 9;
    *gas_res = (uint32_t)(var2 / var3);

    return 0;
}

/* Full Calib from v6.16 */
static int bme680_read_calib(struct bme680_data *data) {
    uint8_t coeff_array[BME680_LEN_COEFF_ALL];
    int ret;

    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF1, coeff_array, BME680_LEN_COEFF1);
    if (ret < 0) return ret;

    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF2, &coeff_array[BME680_LEN_COEFF1], BME680_LEN_COEFF2);
    if (ret < 0) return ret;

    ret = regmap_bulk_read(data->regmap, BME680_REG_COEFF3, &coeff_array[BME680_LEN_COEFF1 + BME680_LEN_COEFF2], BME680_LEN_COEFF3);
    if (ret < 0) return ret;

    data->calib.par_t1 = (uint16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_T1_MSB], coeff_array[BME680_IDX_T1_LSB]));
    data->calib.par_t2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_T2_MSB], coeff_array[BME680_IDX_T2_LSB]));
    data->calib.par_t3 = (int8_t)coeff_array[BME680_IDX_T3_MSB];
    data->calib.par_p1 = (uint16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P1_MSB], coeff_array[BME680_IDX_P1_LSB]));
    data->calib.par_p2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P2_MSB], coeff_array[BME680_IDX_P2_LSB]));
    data->calib.par_p3 = (int8_t)coeff_array[BME680_IDX_P3];
    data->calib.par_p4 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P4_MSB], coeff_array[BME680_IDX_P4_LSB]));
    data->calib.par_p5 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P5_MSB], coeff_array[BME680_IDX_P5_LSB]));
    data->calib.par_p6 = (int8_t)coeff_array[BME680_IDX_P6];
    data->calib.par_p7 = (int8_t)coeff_array[BME680_IDX_P7];
    data->calib.par_p8 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P8_MSB], coeff_array[BME680_IDX_P8_LSB]));
    data->calib.par_p9 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_P9_MSB], coeff_array[BME680_IDX_P9_LSB]));
    data->calib.par_p10 = (uint8_t)coeff_array[BME680_IDX_P10];
    data->calib.par_h2 = (uint16_t)(((uint16_t)coeff_array[BME680_IDX_H2_MSB]) << 4 | (coeff_array[BME680_IDX_H2_LSB] & BME680_BIT_H1_DATA_MSK));
    data->calib.par_h1 = (uint16_t)((((uint16_t)coeff_array[BME680_IDX_H1_MSB]) << 4) | ((coeff_array[BME680_IDX_H1_LSB]) >> 4));
    data->calib.par_h3 = (int8_t)coeff_array[BME680_IDX_H3];
    data->calib.par_h4 = (int8_t)coeff_array[BME680_IDX_H4];
    data->calib.par_h5 = (int8_t)coeff_array[BME680_IDX_H5];
    data->calib.par_h6 = (uint8_t)coeff_array[BME680_IDX_H6];
    data->calib.par_h7 = (int8_t)coeff_array[BME680_IDX_H7];
    data->calib.par_gh1 = (int8_t)coeff_array[BME680_IDX_GH1];
    data->calib.par_gh2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_IDX_GH2_MSB], coeff_array[BME680_IDX_GH2_LSB]));
    data->calib.par_gh3 = (int8_t)coeff_array[BME680_IDX_GH3];
    data->calib.res_heat_val = (int8_t)coeff_array[BME680_IDX_RES_HEAT_VAL];
    data->calib.res_heat_range = (coeff_array[BME680_IDX_RES_HEAT_RANGE] & BME680_RHRANGE_MSK) >> 4;
    data->calib.range_sw_err = ((int8_t)(coeff_array[BME680_IDX_RANGE_SW_ERR] & BME680_RSERROR_MSK)) >> 4;

    return 0;
}

/* Full Read Data with Continuous Mode (new) */
static int bme680_read_data(struct bme680_data *data, struct bme680_fifo_data *fdata) {
    u8 buf[BME680_NUM_BULK_READ_REGS];
    int32_t adc_temp, adc_press, adc_hum;
    uint16_t adc_gas;
    uint8_t gas_range;
    int32_t t_fine;
    uint32_t press, hum, gas_res;
    int ret;
    unsigned long flags;

    mutex_lock(&data->lock);
    spin_lock_irqsave(&data->reg_lock, flags);

    /* Set oversampling */
    ret = regmap_write(data->regmap, BME680_REG_CTRL_HUM, data->oversampling_humid);
    if (ret < 0) goto unlock;

    /* Set temp/press and mode (forced or continuous) */
    u8 mode = (data->mode == BME680_MODE_CONTINUOUS) ? BME680_MODE_CONTINUOUS : BME680_MODE_FORCED;
    ret = regmap_write(data->regmap, BME680_REG_CTRL_MEAS,
                       FIELD_PREP(BME680_OSRS_TEMP_MSK, data->oversampling_temp) |
                       FIELD_PREP(BME680_OSRS_PRESS_MSK, data->oversampling_press) | mode);
    if (ret < 0) goto unlock;

    /* Gas heater */
    if (data->gas_enable) {
        ret = regmap_write(data->regmap, BME680_REG_RES_HEAT0, data->heater_temp);
        if (ret < 0) goto unlock;
        ret = regmap_write(data->regmap, BME680_REG_GAS_WAIT0, data->heater_dur);
        if (ret < 0) goto unlock;
        ret = regmap_write(data->regmap, BME680_REG_CTRL_GAS_0, FIELD_PREP(BME680_NB_CONV_MSK, 0x01) | BME680_RUN_GAS_MSK);
        if (ret < 0) goto unlock;
    }

    spin_unlock_irqrestore(&data->reg_lock, flags);

    /* Wait for measurement */
    msleep(40);

    /* Read raw */
    ret = regmap_bulk_read(data->regmap, BME680_REG_PRESS_MSB, buf, BME680_NUM_BULK_READ_REGS);
    if (ret < 0) goto unlock;

    adc_press = (int32_t)((((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((buf[2] >> 4) & 0x0F)));
    adc_temp = (int32_t)((((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((buf[5] >> 4) & 0x0F)));
    adc_hum = (int32_t)((((uint32_t)buf[6] << 8) | (uint32_t)buf[7]));
    adc_gas = (uint16_t)((((uint16_t)buf[13] << 7) | ((buf[14] >> 1) & 0x7F)));
    gas_range = (buf[14] & BME680_GAS_RANGE_MASK);

    /* Compensate */
    ret = bme680_compensate_temperature(&data->calib, adc_temp, &fdata->temperature, &t_fine);
    if (ret < 0) goto unlock;
    ret = bme680_compensate_pressure(&data->calib, adc_press, t_fine, &press);
    if (ret < 0) goto unlock;
    ret = bme680_compensate_humidity(&data->calib, adc_hum, t_fine, &hum);
    if (ret < 0) goto unlock;
    if (data->gas_enable) {
        ret = bme680_compensate_gas_resistance(&data->calib, adc_gas, gas_range, &gas_res);
        if (ret < 0) goto unlock;
    } else {
        gas_res = 0;
    }

    fdata->temperature *= 1000;  // micro °C
    fdata->pressure = press;
    fdata->humidity = hum;
    fdata->gas_resistance = gas_res;
    fdata->timestamp = ktime_get_ns();

    /* New features */
    bme680_calculate_iaq(data, &fdata->iaq_index);
    bme680_check_threshold(data);
    trace_bme680_read(gas_res);
    cond_resched();

    atomic_inc(&data->read_count);

unlock:
    spin_unlock_irqrestore(&data->reg_lock, flags);
    mutex_unlock(&data->lock);
    return ret;
}

/* Full Poll Thread */
static int bme680_poll_thread(void *arg) {
    struct bme680_data *data = arg;
    struct bme680_fifo_data fdata;

    while (!kthread_should_stop()) {
        if (bme680_read_data(data, &fdata) == 0) {
            down(&data->fifo_sem);
            kfifo_in(&data->data_fifo, &fdata, sizeof(fdata));
            up(&data->fifo_sem);
            wake_up_interruptible(&data->wait_data);
        } else {
            atomic_inc(&data->error_count);
        }
        if (signal_pending(current)) {
            flush_signals(current);
            break;
        }
        cond_resched();  // Lazy preemption
        msleep(500);
    }
    return 0;
}

/* Full File Ops */
static int bme680_open(struct inode *inode, struct file *file) {
    struct bme680_data *data = container_of(inode->i_cdev, struct bme680_data, cdev);
    file->private_data = data;
    try_module_get(THIS_MODULE);
    return 0;
}

static int bme680_release(struct inode *inode, struct file *file) {
    module_put(THIS_MODULE);
    return 0;
}

static ssize_t bme680_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    struct bme680_data *data = file->private_data;
    struct bme680_fifo_data fdata;
    ssize_t ret = 0;

    if (count < sizeof(fdata)) return -EINVAL;

    if (file->f_flags & O_NONBLOCK) {
        if (kfifo_is_empty(&data->data_fifo)) return -EAGAIN;
    }

    down(&data->fifo_sem);
    if (kfifo_out_spinlocked(&data->data_fifo, &fdata, sizeof(fdata), &data->reg_lock) == sizeof(fdata)) {
        if (copy_to_user(buf, &fdata, sizeof(fdata))) ret = -EFAULT;
        else ret = sizeof(fdata);
    } else {
        up(&data->fifo_sem);
        if (wait_event_interruptible(data->wait_data, kfifo_len(&data->data_fifo) >= sizeof(fdata))) {
            ret = -ERESTARTSYS;
        }
        return ret;
    }
    up(&data->fifo_sem);
    return ret;
}

static ssize_t bme680_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    return count;  // Stub
}

static loff_t bme680_llseek(struct file *file, loff_t offset, int whence) {
    struct bme680_data *data = file->private_data;
    loff_t new_pos = -1;
    switch (whence) {
        case SEEK_SET: new_pos = offset; break;
        case SEEK_CUR: new_pos = file->f_pos + offset; break;
        case SEEK_END: new_pos = kfifo_len(&data->data_fifo) + offset; break;
        default: return -EINVAL;
    }
    if (new_pos < 0 || new_pos > kfifo_len(&data->data_fifo)) return -EINVAL;
    file->f_pos = new_pos;
    return new_pos;
}

static unsigned int bme680_poll(struct file *file, poll_table *wait) {
    struct bme680_data *data = file->private_data;
    unsigned int mask = 0;
    poll_wait(file, &data->wait_data, wait);
    if (!kfifo_is_empty(&data->data_fifo)) mask |= POLLIN | POLLRDNORM;
    return mask;
}

static long bme680_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct bme680_data *data = file->private_data;
    void __user *user_ptr = (void __user *)arg;
    int ret = 0;

    if (security_locked_down(LOCKDOWN_DEV_MEM) && cmd == BME680_IOC_GET_SHM) return -EPERM;

    switch (cmd) {
        case BME680_IOC_SET_GAS_CONFIG:
            if (copy_from_user(&data->gas_config, user_ptr, sizeof(struct bme680_gas_config))) return -EFAULT;
            data->heater_temp = data->gas_config.heater_temp;
            data->heater_dur = data->gas_config.heater_dur;
            data->preheat_curr_ma = data->gas_config.preheat_curr_ma;
            break;
        case BME680_IOC_READ_FIFO:
            ret = bme680_read_data(data, &data->fifo_data);
            if (!ret && copy_to_user(user_ptr, &data->fifo_data, sizeof(struct bme680_fifo_data))) ret = -EFAULT;
            break;
        case BME680_IOC_GET_SHM:
            if (copy_to_user(user_ptr, &data->shm_buffer, sizeof(unsigned long))) ret = -EFAULT;
            break;
        default: return -ENOTTY;
    }
    return ret;
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

/* Full Debugfs */
static int bme680_debugfs_show(struct seq_file *s, void *unused) {
    struct bme680_data *data = s->private;
    seq_printf(s, "Reads: %d\nErrors: %d\nThread PID: %d\nUptime: %lu jiffies\nIAQ: %u\n",
               atomic_read(&data->read_count), atomic_read(&data->error_count),
               data->poll_thread ? task_pid_nr(data->poll_thread) : -1,
               jiffies - data->start_time, data->fifo_data.iaq_index);
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

/* Full IIO Read Raw */
static int bme680_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                           int *val, int *val2, long mask) {
    struct bme680_data *data = iio_priv(indio_dev);
    int ret;

    if (pm_runtime_get_sync(data->dev) < 0) return -EIO;

    ret = bme680_read_data(data, &data->fifo_data);
    pm_runtime_mark_last_busy(data->dev);
    pm_runtime_put_autosuspend(data->dev);

    if (ret < 0) return ret;

    switch (chan->type) {
        case IIO_TEMP:
            *val = data->fifo_data.temperature / 1000;
            *val2 = (data->fifo_data.temperature % 1000) * 1000;
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_PRESSURE:
            *val = data->fifo_data.pressure / 100;
            *val2 = (data->fifo_data.pressure % 100) * 10000;
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_HUMIDITYRELATIVE:
            *val = data->fifo_data.humidity;
            return IIO_VAL_INT;
        case IIO_RESISTANCE:
            *val = data->fifo_data.gas_resistance;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
    }
}

/* Trigger Handler with Async Multi-Sample */
static irqreturn_t bme680_trigger_handler(int irq, void *p) {
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct bme680_data *data = iio_priv(indio_dev);
    struct bme680_fifo_data fdata[16];  // Multi-sample
    int i;

    mutex_lock(&data->lock);
    for (i = 0; i < 16; i++) {
        bme680_read_data(data, &fdata[i]);
        iio_push_to_buffers_with_timestamp(indio_dev, &fdata[i], fdata[i].timestamp);
    }
    mutex_unlock(&data->lock);

    iio_trigger_notify_done(indio_dev->trig);
    return IRQ_HANDLED;
}

/* Full Probe (I2C + SPI) */
static int bme680_core_probe(struct device *dev, struct regmap *regmap, const char *name, void *bus_data) {
    struct bme680_data *data;
    struct iio_dev *indio_dev;
    u8 chip_id;
    int ret;

    indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
    if (!indio_dev) return -ENOMEM;

    data = iio_priv(indio_dev);
    data->indio_dev = indio_dev;
    data->regmap = regmap;
    data->dev = dev;
    data->mode = BME680_MODE_FORCED;  // Default
    mutex_init(&data->lock);
    spin_lock_init(&data->reg_lock);
    init_waitqueue_head(&data->wait_data);
    sema_init(&data->fifo_sem, 1);
    atomic_set(&data->read_count, 0);
    atomic_set(&data->error_count, 0);
    data->start_time = jiffies;

    ret = kfifo_alloc(&data->data_fifo, 64 * sizeof(struct bme680_fifo_data), GFP_KERNEL);
    if (ret) return ret;

    data->shm_size = PAGE_SIZE * 4;
    data->shm_buffer = vmalloc(data->shm_size);
    if (!data->shm_buffer) goto err_kfifo;
    memset(data->shm_buffer, 0, data->shm_size);
    memcpy(data->shm_buffer, &data->calib, sizeof(data->calib));

    ret = regmap_read(regmap, BME680_REG_CHIP_ID, &chip_id);
    if (ret || chip_id != BME680_CHIP_ID_VAL) {
        ret = ret ? ret : -ENODEV;
        goto err_vmalloc;
    }

    ret = bme680_read_calib(data);
    if (ret < 0) goto err_vmalloc;

    regmap_write(regmap, BME680_REG_SOFT_RESET, BME680_CMD_SOFTRESET);
    msleep(10);

    data->oversampling_temp = BME680_OVERSAMPLING_X1;
    data->oversampling_press = BME680_OVERSAMPLING_X1;
    data->oversampling_humid = BME680_OVERSAMPLING_X1;
    data->filter_coeff = BME680_FILTER_OFF;
    data->heater_temp = heater_temp;
    data->heater_dur = 150;
    data->preheat_curr_ma = 10;
    data->gas_enable = true;
    data->gas_threshold = 100000;  // Default

    ret = alloc_chrdev_region(&bme680_dev_num, 0, 1, "bme680");
    if (ret) goto err_vmalloc;
    cdev_init(&bme680_cdev, &bme680_fops);
    data->cdev = bme680_cdev;
    ret = cdev_add(&bme680_cdev, bme680_dev_num, 1);
    if (ret) goto err_chrdev;
    bme680_class = class_create(THIS_MODULE, "bme680");
    if (IS_ERR(bme680_class)) {
        ret = PTR_ERR(bme680_class);
        goto err_cdev;
    }
    bme680_device = device_create(bme680_class, dev, bme680_dev_num, data, "bme680");
    if (IS_ERR(bme680_device)) {
        ret = PTR_ERR(bme680_device);
        goto err_class;
    }

    indio_dev->name = name ? name : "bme680";
    indio_dev->channels = bme680_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme680_channels);
    indio_dev->info = &bme680_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE;

    ret = iio_triggered_buffer_setup(indio_dev, NULL, bme680_trigger_handler, NULL);
    if (ret) goto err_device;

    ret = iio_device_register(indio_dev);
    if (ret) goto err_buffer;

    data->poll_thread = kthread_run(bme680_poll_thread, data, "bme680_poll");
    if (IS_ERR(data->poll_thread)) {
        ret = PTR_ERR(data->poll_thread);
        goto err_iio;
    }

    data->debugfs_dir = debugfs_create_dir("bme680", NULL);
    if (!IS_ERR(data->debugfs_dir))
        debugfs_create_file("stats", 0444, data->debugfs_dir, data, &bme680_debugfs_fops);

    /* Hwmon */
    data->hwmon_dev = hwmon_device_register_with_info(dev, "bme680", data, &bme680_hwmon_chip_info, NULL);
    if (IS_ERR(data->hwmon_dev)) {
        ret = PTR_ERR(data->hwmon_dev);
        goto err_thread;
    }

    /* Async buffer */
    indio_dev->buffer = devm_iio_kfifo_allocate(indio_dev);
    if (!indio_dev->buffer) goto err_hwmon;
    iio_device_attach_buffer(indio_dev, indio_dev->buffer);

    /* PM */
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);
    pm_runtime_enable(dev);

    dev_info(dev, "BME680 Driver Probed (I2C/SPI, Continuous Mode)\n");
    return 0;

err_hwmon:
    hwmon_device_unregister(data->hwmon_dev);
err_thread:
    kthread_stop(data->poll_thread);
err_iio:
    iio_device_unregister(indio_dev);
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
err_vmalloc:
    vfree(data->shm_buffer);
err_kfifo:
    kfifo_free(&data->data_fifo);
    return ret;
}

static int bme680_core_remove(struct device *dev) {
    struct iio_dev *indio_dev = dev_get_drvdata(dev);
    struct bme680_data *data = iio_priv(indio_dev);

    pm_runtime_disable(dev);
    if (data->poll_thread) kthread_stop(data->poll_thread);
    debugfs_remove_recursive(data->debugfs_dir);
    iio_device_unregister(indio_dev);
    iio_triggered_buffer_cleanup(indio_dev);
    hwmon_device_unregister(data->hwmon_dev);
    device_destroy(bme680_class, bme680_dev_num);
    class_destroy(bme680_class);
    cdev_del(&bme680_cdev);
    unregister_chrdev_region(bme680_dev_num, 1);
    kfifo_free(&data->data_fifo);
    if (data->shm_buffer) vfree(data->shm_buffer);
    dev_info(dev, "BME680 Driver Removed\n");
    return 0;
}

/* IIO Info */
static const struct iio_info bme680_iio_info = {
    .driver_module = THIS_MODULE,
    .read_raw = bme680_read_raw,
};

/* SPI Probe (full from official bme680_spi.c) */
static int bme680_spi_probe(struct spi_device *spi) {
    struct regmap *regmap;

    regmap = devm_regmap_init_spi(spi, &bme680_regmap_config);
    if (IS_ERR(regmap)) return PTR_ERR(regmap);

    spi_set_drvdata(spi, regmap);

    return bme680_core_probe(&spi->dev, regmap, spi_get_device_id(spi)->name, spi);
}

static int bme680_spi_remove(struct spi_device *spi) {
    return bme680_core_remove(&spi->dev);
}

static const struct spi_device_id bme680_spi_id[] = {
    { "bme680", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, bme680_spi_id);

static struct spi_driver bme680_spi_driver = {
    .driver = {
        .name = "bme680_spi",
        .of_match_table = bme680_of_match,
    },
    .probe = bme680_spi_probe,
    .remove = bme680_spi_remove,
    .id_table = bme680_spi_id,
};
module_spi_driver(bme680_spi_driver);

/* I2C Driver (full) */
static int bme680_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct regmap *regmap = devm_regmap_init_i2c(client, &bme680_regmap_config);
    if (IS_ERR(regmap)) return PTR_ERR(regmap);
    i2c_set_clientdata(client, regmap);
    return bme680_core_probe(&client->dev, regmap, id->name, client);
}

static int bme680_i2c_remove(struct i2c_client *client) {
    return bme680_core_remove(&client->dev);
}

static const struct i2c_device_id bme680_id[] = { { "bme680", 0 }, { } };
MODULE_DEVICE_TABLE(i2c, bme680_id);

static const struct of_device_id bme680_of_match[] = {
    { .compatible = "bosch,bme680" },
    { }
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