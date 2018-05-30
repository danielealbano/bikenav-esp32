#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"
#include "esp_system.h"

#include "lcd.h"
#include "graphics.h"

#define TAG "LCD"

void lcd_cmd(lcd_t* lcd, const uint8_t cmd)
{
    spi_transaction_t t;

    lcd_spi_setup_transaction(lcd, &t, &cmd, 1, true);
    lcd_spi_send_transaction(lcd, &t);
}

void lcd_data(lcd_t* lcd, const uint8_t *data, int data_len)
{
    spi_transaction_t t;

    if (data_len == 0) {
        return;
    }

    lcd_spi_setup_transaction(lcd, &t, data, data_len, true);
    lcd_spi_send_transaction(lcd, &t);
}

void lcd_spi_setup_transaction(lcd_t *lcd, spi_transaction_t* t, void* payload, int payload_length, bool is_data)
{
    lcd_spi_setup_transaction_ex(lcd, t, payload, payload_length, is_data, 0);
}

void lcd_spi_setup_transaction_ex(lcd_t *lcd, spi_transaction_t* t, void* payload, int payload_length, bool is_data, int flags)
{
    memset(&t, 0, sizeof(t));

    t->user = (void*)((lcd->pins->dc << 8) | (is_data ? 1 : 0));
    t->length = payload_length * 8;
    t->tx_buffer = payload;
    t->flags = flags;
}

void lcd_spi_send_transaction(lcd_t *lcd, spi_transaction_t* t)
{
    ret = spi_device_transmit(lcd->spi, t);
    assert(ret==ESP_OK);
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    gpio_num_t dc_pin = (gpio_num_t)(t->user >> 8);
    int dc_level = (int)(t->user & 0xFF);

    gpio_set_level(dc_pin, dc_level);
}

void lcd_send_init_sequence(lcd_t *lcd)
{
    int cmd = 0;

    gpio_set_direction(lcd->pins->dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(lcd->pins->rst, GPIO_MODE_OUTPUT);

    gpio_set_level(lcd->pins->rst, 1);
    vTaskDelay(10 / portTICK_RATE_MS);
    gpio_set_level(lcd->pins->rst, 0);
    vTaskDelay(10 / portTICK_RATE_MS);
    gpio_set_level(lcd->pins->rst, 1);
    vTaskDelay(10 / portTICK_RATE_MS);

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        cmd++;
    }
    
    vTaskDelay(10 / portTICK_RATE_MS);
}

void lcd_spi_transactions_queue(lcd_t *lcd, spi_transaction_t *transactions, uint16_t transactions_count)
{
    esp_err_t ret;
    for (int i = 0; i < transactions_count; i++) {
        ret = spi_device_queue_trans(lcd->spi, transactions + i, portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    lcd->spi->transaction_sent_count += transactions_count;
}

void lcd_spi_transactions_wait(lcd_t *lcd)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;

    for (int i = 0; i < transaction_sent_count; i++) {
        ret = spi_device_get_trans_result_ex(lcd->spi, &rtrans, portMAX_DELAY, false);
        assert(ret == ESP_OK);
    }

    lcd->spi->transaction_sent_count = 0;
}

void lcd_send_lines_seps525(lcd_t* lcd)
{
    uint16_t t_i = 0;

    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, false, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0x17;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, true, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, false, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0x18;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, true, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = lcd->width - 1;

    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, false, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0x19;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, true, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, false, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0x1a;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, true, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = lcd->height - 1;

    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], NULL, 1, false, SPI_TRANS_USE_TXDATA);
    lcd->spi->transactions[t_i++].tx_data[0] = 0x22;
    lcd_spi_setup_transaction_ex(&lcd->spi->transactions[t_i], fb_data, lcd->width * lcd->height * lcd->bps * 8, true, SPI_TRANS_USE_TXDATA); t_i++;

    lcd_spi_transactions_queue(lcd, t, t_i);
}

void lcd_update_display(lcd_t *lcd)
{
    lcd_spi_transactions_wait(lcd);

    if (lcd->current_fb && lcd->release_fb_cb) {
        lcd->release_fb_cb(lcd);
    }
    lcd->current_fb = lcd->acquire_fb_cb(lcd);

    lcd_send_lines(lcd);
}

lcd_t* lcd_alloc(uint16_t width, uint16_t height, uint8_t bps)
{
    lcd_t *lcd = (lcd_t*)malloc(sizeof(lcd_t));
    memset(lcd, 0, sizeof(lcd_t));

    lcd->width = width;
    lcd->height = height;
    lcd->bps = bps;
    lcd->current_fb = NULL;

    lcd->spi->device = NULL;
    lcd->spi->buscfg = (spi_bus_config_t *)malloc(sizeof(spi_bus_config_t));
    lcd->spi->devcfg = (spi_device_interface_config_t *)malloc(sizeof(spi_device_interface_config_t));

    memset(buscfg, 0, sizeof(spi_bus_config_t));
    memset(devcfg, 0, sizeof(spi_device_interface_config_t));

    return lcd;
}

void lcd_init_cb(lcd_t *lcd, lcd_acquire_fb_cb_t* acquire_fb_cb, lcd_release_fb_cb_t* release_fb_cb)
{
    lcd->acquire_fb_cb = acquire_fb_cb;
    lcd->release_fb_cb = release_fb_cb; 
}

void lcd_init_pins(lcd_t *lcd, gpio_num_t miso, gpio_num_t clk, gpio_num_t cs, gpio_num_t dc, gpio_num_t rst)
{
    lcd->pins.miso = miso;
    lcd->pins.clk = clk;
    lcd->pins.cs = cs;
    lcd->pins.dc = dc;
    lcd->pins.rst = rst;
}

void lcd_free(lcd_t* lcd)
{
    free(lcd);
}

void lcd_setup_spi(lcd_t *lcd)
{
    esp_err_t ret;

    buscfg->miso_io_num = PIN_NUM_MISO;
    buscfg->mosi_io_num = PIN_NUM_MOSI;
    buscfg->sclk_io_num = PIN_NUM_CLK;
    buscfg->quadwp_io_num = -1;
    buscfg->quadhd_io_num = -1;
    buscfg->max_transfer_sz = (lcd->width * lcd->height * lcd->bps) + 50;

    devcfg->clock_speed_hz = 40 * 1000 * 1000;
    devcfg->mode = 0;
    devcfg->spics_io_num = PIN_NUM_CS;
    devcfg->queue_size = 10;
    devcfg->pre_cb = lcd_spi_pre_transfer_callback;

    ret = spi_bus_initialize(HSPI_HOST, lcd->spi->buscfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, lcd->spi->devcfg, &lcd->spi->device);
    ESP_ERROR_CHECK(ret);
}

void lcd_setup(lcd_t *lcd)
{
    lcd_send_init_sequence(lcd);
}

void lcd_update_display(lcd_t *lcd)
{
#if defined(DISPLAY_SEPS525)
    lcd_send_lines_seps525(lcd);
#endif
}
