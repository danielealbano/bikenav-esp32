#ifndef __IMU__
#define __IMU__

#define BMM150_USE_FLOATING_POINT 1

#include "bmi160.h"
#include "bmi160_defs.h"
#include "bmm150.h"
#include "bmm150_defs.h"

#define TAG "IMU"

#define I2C_ACK_CHECK_EN    0x1
#define I2C_ACK_CHECK_DIS   0x0
#define I2C_ACK_VAL         0x0
#define I2C_NACK_VAL        0x1

#define IMU_I2C_MAX_WAIT    1000/portTICK_PERIOD_MS

typedef struct imu imu_t;

struct imu
{
    struct i2c 
    {
        struct pins
        {
            gpio_num_t sda;
            gpio_num_t scl;
        } pins;
        
        uint32_t frequency_hz;
        uint8_t port_number;
        uint8_t bmi160_address;
        uint8_t bmm150_address;
    } i2c;

    struct bmi160_dev sensor_bmi160;
    struct bmm150_dev sensor_bmm150;
};

imu_t* imu_alloc(uint8_t i2c_bmi160_address, uint8_t i2c_bmm150_address, gpio_num_t i2c_pins_sda, 
                 gpio_num_t i2c_pins_scl, uint8_t i2c_port_number, uint32_t frequency_hz);
void imu_init(imu_t *imu);
void imu_free(imu_t *imu);

#endif
