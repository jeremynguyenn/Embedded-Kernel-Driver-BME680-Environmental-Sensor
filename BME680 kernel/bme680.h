#ifndef _BME680_H_
#define _BME680_H_

#include <linux/bitfield.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/kfifo.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

typedef enum {
    BME680_BOOL_FALSE = 0,
    BME680_BOOL_TRUE  = 1,
} bme680_bool_t;

typedef enum {
    BME680_MODE_SLEEP  = 0,
    BME680_MODE_FORCED = 1,
} bme680_mode_t;

typedef enum {
    BME680_OVERSAMPLING_SKIP = 0,
    BME680_OVERSAMPLING_X1   = 1,
    BME680_OVERSAMPLING_X2   = 2,
    BME680_OVERSAMPLING_X4   = 3,
    BME680_OVERSAMPLING_X8   = 4,
    BME680_OVERSAMPLING_X16  = 5,
} bme680_oversampling_t;

typedef enum {
    BME680_FILTER_OFF       = 0,
    BME680_FILTER_COEFF_1   = 1,
    BME680_FILTER_COEFF_3   = 2,
    BME680_FILTER_COEFF_7   = 3,
    BME680_FILTER_COEFF_15  = 4,
    BME680_FILTER_COEFF_31  = 5,
    BME680_FILTER_COEFF_63  = 6,
    BME680_FILTER_COEFF_127 = 7,
} bme680_filter_t;

#define BME680_REG_CHIP_ID          0xD0
#define BME680_REG_RESET            0xE0
#define BME680_REG_STATUS           0x73
#define BME680_REG_CTRL_HUM         0x72
#define BME680_REG_CTRL_MEAS        0x74
#define BME680_REG_CONFIG           0x75
#define BME680_REG_CTRL_GAS_1       0x71
#define BME680_REG_CTRL_GAS_0       0x70
#define BME680_REG_GAS_WAIT_0       0x64
#define BME680_REG_RES_HEAT_0       0x5A
#define BME680_REG_IDAC_HEAT_0      0x50
#define BME680_REG_TEMP_MSB         0x22
#define BME680_REG_TEMP_LSB         0x23
#define BME680_REG_TEMP_XLSB        0x24
#define BME680_REG_PRESS_MSB        0x1F
#define BME680_REG_PRESS_LSB        0x20
#define BME680_REG_PRESS_XLSB       0x21
#define BME680_REG_HUM_MSB          0x25
#define BME680_REG_HUM_LSB          0x26
#define BME680_REG_GAS_R_MSB        0x2A
#define BME680_REG_GAS_R_LSB        0x2B
#define BME680_REG_MEAS_STAT_0      0x1D
#define BME680_REG_CALIB_0          0x89
#define BME680_REG_CALIB_1          0xE1
#define BME680_CMD_SOFTRESET        0xB6
#define BME680_CHIP_ID_VAL          0x61
#define BME680_GAS_MEAS_BIT         BIT(5)
#define BME680_GAS_STAB_BIT         BIT(4)
#define BME680_NEW_DATA_MSK         BIT(7)
#define BME680_GAS_MEASURING_MSK    BIT(6)

#define BME680_OSRS_T_MSK           GENMASK(7, 5)
#define BME680_OSRS_P_MSK           GENMASK(4, 2)
#define BME680_OSRS_H_MSK           GENMASK(7, 5)
#define BME680_FILTER_MSK           GENMASK(4, 2)
#define BME680_MODE_MSK             GENMASK(1, 0)
#define BME680_GAS_PROFILE_MSK      GENMASK(3, 0)
#define BME680_RUN_GAS_MSK          BIT(4)
#define BME680_NB_CONV_MSK          GENMASK(3, 0)
#define BME680_OSRS_T_SFT           5
#define BME680_OSRS_P_SFT           2
#define BME680_OSRS_H_SFT           5
#define BME680_FILTER_SFT           2

#define BME680_FIELD_DATA_SIZE      15
#define BME680_MAX_HEATER_TEMP      400
#define BME680_MIN_HEATER_TEMP      200
#define BME680_MAX_OVERSAMPLING     BME680_OVERSAMPLING_X16
#define BME680_MAX_FILTER           BME680_FILTER_COEFF_127
#define BME680_MAX_GAS_PROFILE      10

struct bme680_calib {
    u16 par_t1;
    s16 par_t2;
    s8  par_t3;
    u16 par_p1;
    s16 par_p2;
    s8  par_p3;
    s16 par_p4;
    s16 par_p5;
    s8  par_p6;
    s8  par_p7;
    s16 par_p8;
    s16 par_p9;
    u8  par_p10;
    u16 par_h1;
    u16 par_h2;
    s8  par_h3;
    s8  par_h4;
    s8  par_h5;
    u8  par_h6;
    s8  par_h7;
    s8  par_gh1;
    s16 par_gh2;
    s8  par_gh3;
    u8  res_heat_range;
    s8  res_heat_val;
    s8  range_sw_err;
};

struct bme680_field_data {
    s32 temperature;
    u32 pressure;
    u32 humidity;
    u32 gas_resistance;
    u8  gas_range;
    bool heat_stable;
};

struct bme680_fifo_data {
    s64 timestamp;
    s32 temperature;
    u32 pressure;
    u32 humidity;
    u32 gas_resistance;
};

struct bme680_gas_config {
    u16 heater_temp;
    u16 heater_dur;
    u8  preheat_curr_ma;
};

static const u8 bme680_lookup_k1_range[] = {
    0, 0, 0, 0, 43, 68, 85, 100, 100, 100, 99, 91, 83, 72, 59, 44
};

static const u8 bme680_lookup_k2_range[] = {
    0, 0, 0, 0, 86, 86, 85, 84, 80, 75, 68, 59, 47, 35, 21, 0
};

struct bme680_data {
    struct iio_dev *indio_dev;
    struct regmap *regmap;
    struct bme680_calib calib;
    struct mutex lock;
    spinlock_t reg_lock;
    wait_queue_head_t wait_data;
    DECLARE_KFIFO(data_fifo, struct bme680_fifo_data, 64);
    struct semaphore fifo_sem;
    struct task_struct *poll_thread;
    atomic_t read_count;
    atomic_t error_count;
    unsigned long start_time;
    s32 t_fine;
    u8 oversampling_temp;
    u8 oversampling_press;
    u8 oversampling_humid;
    u8 filter_coeff;
    u16 heater_dur;
    u16 heater_temp;
    u8 preheat_curr_ma;
    bool gas_enable;
    struct device *dev;
    struct pm_runtime_data *pm_data;
    struct dentry *debugfs_dir;
    void *shm_buffer;
    size_t shm_size;
    u8 chip_id;
    u8 heater_profile_len;
    u16 heater_dur_profile[10];
    u16 heater_temp_profile[10];
    u8 heater_res[10];
    u8 heater_idac[10];
};

#define BME680_IOC_MAGIC 'B'
#define BME680_IOC_SET_GAS_CONFIG _IOW(BME680_IOC_MAGIC, 1, struct bme680_gas_config)
#define BME680_IOC_READ_FIFO _IOR(BME680_IOC_MAGIC, 2, struct bme680_fifo_data)
#define BME680_IOC_GET_SHM _IOR(BME680_IOC_MAGIC, 3, unsigned long)

static const struct iio_chan_spec bme680_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_RAW),
        .scan_index = 0,
        .scan_type = {
            .sign = 's',
            .realbits = 20,
            .storagebits = 32,
            .endianness = IIO_BE,
        },
    },
    {
        .type = IIO_PRESSURE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_RAW),
        .scan_index = 1,
        .scan_type = {
            .sign = 'u',
            .realbits = 20,
            .storagebits = 32,
            .endianness = IIO_BE,
        },
    },
    {
        .type = IIO_HUMIDITYRELATIVE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_RAW),
        .scan_index = 2,
        .scan_type = {
            .sign = 'u',
            .realbits = 16,
            .storagebits = 16,
            .endianness = IIO_CPU,
        },
    },
    {
        .type = IIO_RESISTANCE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | BIT(IIO_CHAN_INFO_RAW),
        .scan_index = 3,
        .scan_type = {
            .sign = 'u',
            .realbits = 10,
            .storagebits = 16,
            .endianness = IIO_CPU,
        },
        .event_spec = {
            {
                .type = IIO_EV_TYPE_CHANGE,
                .dir = IIO_EV_DIR_EITHER,
                .mask_separate = BIT(IIO_EV_INFO_VALUE),
            },
        },
        .num_event_specs = 1,
    },
    IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const struct regmap_range bme680_volatile_ranges[] = {
    {
        .range_min = BME680_REG_MEAS_STAT_0,
        .range_max = BME680_REG_GAS_R_LSB,
    },
};

static const struct regmap_access_table bme680_volatile_table = {
    .yes_ranges = bme680_volatile_ranges,
    .n_yes_ranges = ARRAY_SIZE(bme680_volatile_ranges),
};

int bme680_core_probe(struct device *dev, struct regmap *regmap, const char *name, void *bus_data);
int bme680_core_remove(struct device *dev);
int bme680_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask);
int bme680_read_data(struct bme680_data *data, struct bme680_fifo_data *fdata);
int bme680_read_calib(struct bme680_data *data);

<<<<<<< HEAD
#endif /* _BME680_H_ */
=======
#endif /* _BME680_H_ */
>>>>>>> 118756714b84fee6678b770077b3fa8a0ea4dab6
