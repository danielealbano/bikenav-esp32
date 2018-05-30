
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "bmi160_wrapper.h"

#define I2C_ACK_CHECK_EN                0x1
#define I2C_ACK_CHECK_DIS               0x0
#define I2C_ACK_VAL                     0x0
#define I2C_NACK_VAL                    0x1

#define BMI160_WRAPPER_I2C_MAX_WAIT     1000/portTICK_PERIOD_MS

// TODO
// need to add support for user data into the bmi160 official driver to make it work properly without
// any magic constant containing the port_number.
// The port number can be encoded within the user data because the code doesn't need access to the
// entire structure

int8_t bmi160_wrapper_user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    if (length == 0) {
        return (int8_t)ESP_ERR_INVALID_SIZE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_DIS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, reg_data, length - 1, I2C_ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data+length-1, I2C_NACK_VAL);
    i2c_master_stop(cmd);

    // TODO: to fix
    esp_err_t ret = i2c_master_cmd_begin(bmi160_wrapper->i2c.port_number, cmd, BMI160_WRAPPER_I2C_MAX_WAIT);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

int8_t bmi160_wrapper_user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{	
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_EN);
    i2c_master_write(cmd, reg_data, length, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // TODO: to fix
    esp_err_t ret = i2c_master_cmd_begin(bmi160_wrapper->i2c.port_number, cmd, BMI160_WRAPPER_I2C_MAX_WAIT);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

void bmi160_wrapper_user_delay_ms(uint32_t period)
{
    uint32_t start = millis();
    do {
        // do nothing
    } while(millis() - start <= period);
}

bmi160_wrapper_t* bmi160_wrapper_alloc(uint8_t i2c_address, gpio_num_t i2c_pins_sda, gpio_num_t i2c_pins_scl, uint8_t i2c_port_number, uint32_t frequency_hz)
{
    bmi160_wrapper_t *bmi160_wrapper = (bmi160_wrapper_t*)malloc(sizeof(bmi160_wrapper_t));
    memset(bmi160_wrapper, 0, sizeof(bmi160_wrapper_t));

    bmi160_wrapper->i2c.address = i2c_address;
    bmi160_wrapper->i2c.port_number = i2c_port_number;
    bmi160_wrapper->i2c.frequency_hz = frequency_hz;
    bmi160_wrapper->i2c.pins.sda = i2c_pins_sda;
    bmi160_wrapper->i2c.pins.scl = i2c_pins_scl;

    return bmi160_wrapper;
}

void bmi160_wrapper_init_i2c(bmi160_wrapper_t *bmi160_wrapper)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = bmi160_wrapper->i2c.pins.sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = bmi160_wrapper->i2c.pins.scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = bmi160_wrapper->i2c.frequency_hz;

    i2c_param_config(bmi160_wrapper->i2c.port_number, &conf);
    i2c_driver_install(bmi160_wrapper->i2c.port_number, conf.mode, 0, 0, 0);
}

void bmi160_wrapper_init_sensor(bmi160_wrapper_t *bmi160_wrapper)
{
    int8_t result;
    struct bmi160_dev *sensor = bmi160_wrapper->sensor;

    sensor->id = bmi160_wrapper->i2c.address;
    sensor->interface = BMI160_I2C_INTF;
    sensor->read = bmi160_wrapper_support_user_i2c_read;
    sensor->write = bmi160_wrapper_support_user_i2c_write;
    sensor->delay_ms = bmi160_wrapper_support_user_delay_ms;
    
    if ((result = bmi160_init(sensor)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %d", result);
        return;
    }
    ESP_LOGI(TAG, "BMI160 CHIP ID : %d", sensor->chip_id);

    /* Select the Output data rate, range of accelerometer sensor */
    sensor->accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    sensor->accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    sensor->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor->gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    sensor->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    if ((result = bmi160_set_sens_conf(sensor)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_set_sens_conf failed: %d", result);
        return;
    }
}

void bmi160_wrapper_init_perform_self_testing(bmi160_wrapper_t *bmi160_wrapper)
{
    // TODO: add support for self testing
}

void bmi160_wrapper_init(bmi160_wrapper_t *bmi160_wrapper)
{
    bmi160_wrapper_init_i2c(bmi160_wrapper);
    bmi160_wrapper_init_sensor(bmi160_wrapper);
    bmi160_wrapper_init_perform_self_testing(bmi160_wrapper);
}

void bmi160_wrapper_free(bmi160_wrapper_t *bmi160_wrapper)
{
    free(bmi160_wrapper);
}