#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xtensa/core-macros.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"

#include "imu.h"

#define TAG "MAIN"

imu_t *imu;

#define BMI160_SENSOR_RANGE 65535.0f
#define BMI160_SENSOR_LOW 32768.0f

float convert_raw(int raw, float range_abs)
{
    float slope;
    float val;

    /* Input range will be -32768 to 32767
     * Output range must be -range_abs to range_abs */
    val = (float)raw;
    slope = (range_abs * 2.0f) / BMI160_SENSOR_RANGE;
    return -(range_abs) + slope * (val + BMI160_SENSOR_LOW);
}

float convert_raw_gyro(int raw)
{
    return convert_raw(raw, 2000);
}

float convert_raw_acceleration(int raw)
{
    return convert_raw(raw, 2);
}

void imu_fetch_data()
{
    // struct bmi160_aux_data mag;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &imu->sensor_bmi160);
    //bmi160_read_aux_data_auto_mode(mag.data, &sensor_bmi160);
    //bmm150_aux_mag_data(mag.data, &sensor_bmm150);

    float ax = convert_raw_acceleration(accel.x);
    float ay = convert_raw_acceleration(accel.y);
    float az = convert_raw_acceleration(accel.z);
    float gx = convert_raw_gyro(gyro.x);
    float gy = convert_raw_gyro(gyro.y);
    float gz = convert_raw_gyro(gyro.z);

    ESP_LOGI(TAG, "[ACCEL] X: %f, Y: %f, Z: %f", ax, ay, az);
    ESP_LOGI(TAG, "[GYRO]  X: %f, Y: %f, Z: %f", gx, gy, gz);
    //ESP_LOGI(TAG, "[MAG] X: %f, Y: %f, Z: %f", sensor_bmm150.data.x, sensor_bmm150.data.y, sensor_bmm150.data.z);
}

void app_loop()
{
    while (1) {
        imu_fetch_data();
    }
}

/*
void setup_display()
{
}
*/

/*
void setup_graphics()
{
    ug = ug_alloc(160, 128, UG_COLOR_RGB565);
    ug_set_background_color(ug, UG_COLOR_BLACK);
}
*/
void setup_imu()
{
    imu = imu_alloc(
        BMI160_I2C_ADDR,
        BMI160_AUX_BMM150_I2C_ADDR,
        21,
        22,
        I2C_NUM_0,
        4 * 100000
    );
}

void app_main()
{
    setup_imu();
    //setup_graphics();
    //setup_display();

    ESP_LOGI(TAG, "Free memory after init: %d", (int)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    app_loop();
}
