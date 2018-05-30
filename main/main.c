#define BMM150_USE_FLOATING_POINT 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "xtensa/core-macros.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_deep_sleep.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "graphics.h"
#include "font.h"
#include "lcd.h"
#include "font6x8.h"
#include "font6x10.h"
#include "font7x12.h"

#include "bmi160.h"
#include "bmi160_defs.h"
#include "bmm150.h"
#include "bmm150_defs.h"

#define TAG "MAIN"


#define I2C_ACK_CHECK_EN                0x1
#define I2C_ACK_CHECK_DIS               0x0
#define I2C_ACK_VAL                     0x0
#define I2C_NACK_VAL                    0x1
#define BMI160_I2C_MASTER_SCL_IO        22
#define BMI160_I2C_MASTER_SDA_IO        21
#define BMI160_I2C_PORT_NUM             I2C_NUM_0
#define BMI160_I2C_FREQ_HZ              4 * 100000


struct bmi160_dev sensor_bmi160;
struct bmm150_dev sensor_bmm150;
float accel_scale, gyro_range;

uint16_t **framebuffer;
graphics_t *graphics;
font_t *font6x8;
font_t *font6x10;
font_t *font7x12;

int fps = 0;
int frame = 0;
int refreshes = 0;
spi_device_handle_t *spi_global;

RTC_DATA_ATTR uint8_t from_deep_sleep;

inline static int millis()
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

inline static float radians(int angle)
{
    if (angle > 360)
    {
        angle = angle % 360;
    }

    return (float)angle * 0.01745;
}

void draw_compass(graphics_t *g, int angle)
{
    int circle_x = 64;
    int circle_y = 64;
    int circle_radius = 60;
    int needle_margin = 10;

    int triangle_radius_1 = circle_radius - needle_margin;
    int triangle_radius_2 = 45;

    int needle_x0 = triangle_radius_1 * cos(radians(angle));
    int needle_y0 = triangle_radius_1 * sin(radians(angle));
    int needle_x1 = triangle_radius_2 * cos(radians(angle + 90 + 60));
    int needle_y1 = triangle_radius_2 * sin(radians(angle + 90 + 60));
    int needle_x2 = triangle_radius_2 * cos(radians(angle + 90 + 60 + 60));
    int needle_y2 = triangle_radius_2 * sin(radians(angle + 90 + 60 + 60));

    graphics_draw_circle(g,
        circle_x, circle_y,
        circle_radius,
        GRAPHICS_COLOR_BLACK);

    graphics_draw_triangle(g,
        circle_x + needle_x0, circle_y + needle_y0,
        circle_x + needle_x1, circle_y + needle_y1,
        circle_x + needle_x2, circle_y + needle_y2,
        GRAPHICS_COLOR_RED);

    graphics_draw_circle(g,
        circle_x + needle_x0, circle_y + needle_y0,
        4,
        GRAPHICS_COLOR_GREEN);
}

void framebuffer_draw_info(graphics_t *g)
{
}


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

void framebuffer_draw(void *pvParameters)
{
    struct bmi160_aux_data mag;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    static int fps_counter = 0;
    static int last_second_millis = 0;
    int curx;

    graphics_t *g = graphics;

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor_bmi160);
    bmi160_read_aux_data_auto_mode(mag.data, &sensor_bmi160);
    bmm150_aux_mag_data(mag.data, &sensor_bmm150);

    float ax = convert_raw_acceleration(accel.x);
    float ay = convert_raw_acceleration(accel.y);
    float az = convert_raw_acceleration(accel.z);
    float gx = convert_raw_gyro(gyro.x);
    float gy = convert_raw_gyro(gyro.y);
    float gz = convert_raw_gyro(gyro.z);

    char ax_str[10];
    snprintf(ax_str, 10, "%f", ax);
    char ay_str[10];
    snprintf(ay_str, 10, "%f", ay);
    char az_str[10];
    snprintf(az_str, 10, "%f", az);
    char gx_str[10];
    snprintf(gx_str, 10, "%f", gx);
    char gy_str[10];
    snprintf(gy_str, 10, "%f", gy);
    char gz_str[10];
    snprintf(gz_str, 10, "%f", gz);

    graphics_begin(graphics);
    
    graphics_set_text_color(g, GRAPHICS_COLOR_WHITE, GRAPHICS_COLOR_BLACK);

    graphics_set_text_font(graphics, font7x12);
    graphics_set_cursor(g, 1, 1);
    graphics_print_string(g, "FPS: ");
    graphics_print_number(g, fps);
    graphics_print_string(g, " Frame: ");
    graphics_print_number(g, frame);

    graphics_set_text_font(graphics, font6x8);
    curx = 14; graphics_set_cursor(g, 1, curx);
    graphics_print_string(g, "ACCEL ");
    
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "X: "); graphics_print_string(g, ax_str);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Y: "); graphics_print_string(g, ay_str);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Z: "); graphics_print_string(g, az_str);

    curx += 9; graphics_set_cursor(g, 1, curx);
    graphics_print_string(g, "GYRO ");
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "X: "); graphics_print_string(g, gx_str);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Y: "); graphics_print_string(g, gy_str);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Z: "); graphics_print_string(g, gz_str);
    
    curx += 9; graphics_set_cursor(g, 1, curx);
    graphics_print_string(g, "MAG ");
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "X: "); graphics_print_number(g, sensor_bmm150.data.x);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Y: "); graphics_print_number(g, sensor_bmm150.data.y);
    curx += 9; graphics_set_cursor(g, 6, curx);
    graphics_print_string(g, "Z: "); graphics_print_number(g, sensor_bmm150.data.z);
    
    graphics_end(graphics);

    frame++;
    fps_counter++;

    int now_millis = millis();
    if (now_millis - last_second_millis > 1000)
    {
        fps = fps_counter;
        fps_counter = 0;
        last_second_millis = now_millis;
        
        ESP_LOGI(TAG, "FPS: %d", fps);
    }
}

void app_loop(void *pvParameters)
{
    spi_device_handle_t *spi = spi_global;

    int refreshes_counter = 0;
    int last_second_millis = 0;

    while (1) {
        int now_millis = millis();

        framebuffer_draw((void*)NULL);
        lcd_update_display(spi, graphics);
        
        if (now_millis - last_second_millis > 1000) {
            //ESP_LOGI(TAG, "Refreshes: %d", refreshes_counter);
            refreshes = refreshes_counter;
            refreshes_counter = 0;
            last_second_millis = now_millis;
        }
    }
}

void setup_spi(spi_device_handle_t * spi)
{
    esp_err_t ret;

    spi_bus_config_t *buscfg = (spi_bus_config_t *)malloc(sizeof(spi_bus_config_t));
    memset(buscfg, 0, sizeof(spi_bus_config_t));

    spi_device_interface_config_t *devcfg = (spi_device_interface_config_t *)malloc(sizeof(spi_device_interface_config_t));
    memset(devcfg, 0, sizeof(spi_device_interface_config_t));

    buscfg->miso_io_num = PIN_NUM_MISO;
    buscfg->mosi_io_num = PIN_NUM_MOSI;
    buscfg->sclk_io_num = PIN_NUM_CLK;
    buscfg->quadwp_io_num = -1;
    buscfg->quadhd_io_num = -1;
    buscfg->max_transfer_sz = 160 * 128 * 2 + 20;

    devcfg->clock_speed_hz = 40 * 1000 * 1000;
    devcfg->mode = 0;
    devcfg->spics_io_num = PIN_NUM_CS;
    devcfg->queue_size = 10;
    devcfg->pre_cb = lcd_spi_pre_transfer_callback;

    ret = spi_bus_initialize(HSPI_HOST, buscfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, devcfg, spi);
    ESP_ERROR_CHECK(ret);
}

void setup_display(spi_device_handle_t * spi)
{
    lcd_init(spi);
}

void setup_graphics()
{
    font6x8 = font_init(font6x8_xres, font6x8_yres, font6x8_pixels);
    font6x10 = font_init(font6x10_xres, font6x10_yres, font6x10_pixels);
    font7x12 = font_init(font7x12_xres, font7x12_yres, font7x12_pixels);

    graphics = graphics_init(160, 128);
    graphics_set_background_color(graphics, GRAPHICS_COLOR_BLACK);
}

int8_t bmi160_user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    if (length == 0) {
        return (int8_t)ESP_ERR_INVALID_SIZE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMI160_I2C_ADDR << 1 ) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_DIS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMI160_I2C_ADDR << 1 ) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, reg_data, length - 1, I2C_ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data+length-1, I2C_NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BMI160_I2C_PORT_NUM, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

int8_t bmi160_user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{	
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMI160_I2C_ADDR << 1 ) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, I2C_ACK_CHECK_EN);
    i2c_master_write(cmd, reg_data, length, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(BMI160_I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (int8_t)ret;
}

void bmi160_user_delay_ms(uint32_t period)
{
    uint32_t start = millis();
    do {
        // do nothing
    } while(millis() - start <= period);
}

void setup_bmi160()
{
    int8_t result;
    i2c_config_t conf;

    memset(&conf, 0, sizeof(conf));
    memset(&sensor_bmi160, 0, sizeof(sensor_bmi160));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = BMI160_I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = BMI160_I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = BMI160_I2C_FREQ_HZ;
    i2c_param_config(BMI160_I2C_PORT_NUM, &conf);
    i2c_driver_install(BMI160_I2C_PORT_NUM, conf.mode, 0, 0, 0);

    sensor_bmi160.id = BMI160_I2C_ADDR;
    sensor_bmi160.interface = BMI160_I2C_INTF;
    sensor_bmi160.read = bmi160_user_i2c_read;
    sensor_bmi160.write = bmi160_user_i2c_write;
    sensor_bmi160.delay_ms = bmi160_user_delay_ms;

    
    if ((result = bmi160_init(&sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %d", result);
        return;
    }
    ESP_LOGI(TAG, "BMI160 CHIP ID : %d", sensor_bmi160.chip_id);

    /* Select the Output data rate, range of accelerometer sensor */
    sensor_bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    sensor_bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    sensor_bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor_bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor_bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    sensor_bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor_bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor_bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    if ((result = bmi160_set_sens_conf(&sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_set_sens_conf failed: %d", result);
        return;
    }

    // bmi160_perform_self_test
}

int8_t bmm150_user_aux_read(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
    return bmi160_aux_read(reg_addr, aux_data, len, &sensor_bmi160);
}

int8_t bmm150_user_aux_write(uint8_t id, uint8_t reg_addr, uint8_t *aux_data, uint16_t len)
{
	return bmi160_aux_write(reg_addr, aux_data, len, &sensor_bmi160);
}

void bmm150_user_delay_ms(uint32_t period)
{
    uint32_t start = millis();
    do {
        // do nothing
    } while(millis() - start <= period);
}

void setup_bmm150()
{
    int8_t result;
    memset(&sensor_bmm150, 0, sizeof(sensor_bmm150));

    sensor_bmi160.aux_cfg.aux_sensor_enable = 1; // auxiliary sensor enable
    sensor_bmi160.aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR; // auxiliary sensor address
    sensor_bmi160.aux_cfg.manual_enable = 0; // setup mode enable
    sensor_bmi160.aux_cfg.aux_rd_burst_len = 2;// burst read of 2 byte

    sensor_bmm150.read = bmm150_user_aux_read;
    sensor_bmm150.write = bmm150_user_aux_write;
    sensor_bmm150.delay_ms = bmm150_user_delay_ms;
    sensor_bmm150.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
    sensor_bmm150.intf = BMM150_I2C_INTF;

    if ((result = bmi160_aux_init(&sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_init failed: %d", result);
        return;
    }

    if ((result = bmm150_init(&sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_init failed: %d", result);
        return;
    }
    ESP_LOGI(TAG, "BMM150 CHIP ID : %d", sensor_bmm150.chip_id);

    sensor_bmm150.settings.pwr_mode = BMM150_NORMAL_MODE;
    if ((result = bmm150_set_op_mode(&sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_set_op_mode failed: %d", result);
        return;
    }

    sensor_bmm150.settings.preset_mode = BMM150_PRESETMODE_LOWPOWER;
    if ((result = bmm150_set_presetmode(&sensor_bmm150)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmm150_set_presetmode failed: %d", result);
        return;
    }

    sensor_bmi160.aux_cfg.aux_odr = 8;
    if ((result = bmi160_config_aux_mode(&sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_config_aux_mode failed: %d", result);
        return;
    }

    if ((result = bmi160_set_aux_auto_mode(&sensor_bmi160.aux_cfg.aux_i2c_addr, &sensor_bmi160)) != BMI160_OK) {
        ESP_LOGE(TAG, "bmi160_set_aux_auto_mode failed: %d", result);
        return;
    }
}


void app_main()
{
    spi_device_handle_t *spi = (spi_device_handle_t *)malloc(sizeof(spi_device_handle_t));
    memset(spi, 0, sizeof(spi_device_handle_t));

    esp_wifi_set_mode(WIFI_MODE_NULL);
    //esp_bt_controller_disable();

    setup_bmi160();
    setup_bmm150();
    setup_spi(spi);
    setup_display(spi);
    setup_graphics();

    ESP_LOGI(TAG, "Free memory after init: %d", (int)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    spi_global = spi;

    app_loop((void*)NULL);

    from_deep_sleep = 1;

    /*
    xTaskCreatePinnedToCore(update_display, "update display", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(framebuffer_draw, "fb draw", 4096, NULL, 3, NULL, 1);
    */
}
