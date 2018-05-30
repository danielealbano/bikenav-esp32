#ifndef __BMI160_WRAPPER__
#define __BMI160_WRAPPER__

#define BMM150_USE_FLOATING_POINT 1

#include "bmi160.h"
#include "bmi160_defs.h"

#define TAG "BMI160_WRAPPER"

#define I2C_ACK_CHECK_EN                0x1
#define I2C_ACK_CHECK_DIS               0x0
#define I2C_ACK_VAL                     0x0
#define I2C_NACK_VAL                    0x1
#define BMI160_I2C_MASTER_SCL_IO        22
#define BMI160_I2C_MASTER_SDA_IO        21
#define BMI160_I2C_PORT_NUM             I2C_NUM_0
#define BMI160_I2C_FREQ_HZ              4 * 100000


typedef struct bmi160_wrapper bmi160_wrapper_t;

struct bmi160_wrapper
{
    struct i2c 
    {
        struct pins
        {
            gpio_num_t sda;
            gpio_num_t scl;
        };
        
        uint32_t frequency_hz;
        uint8_t port_number;
        uint8_t address;
    };
    struct bmi160_dev sensor;
};

#endif
