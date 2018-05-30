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

void lcd_cmd(spi_device_handle_t* spi, const uint8_t cmd)
{
    spi_transaction_t t;

    lcd_spi_setup_transaction(&t, &cmd, 1, true);
    lcd_spi_send_transaction(lcd, &t);
}

void lcd_data(spi_device_handle_t* spi, const uint8_t *data, int data_len)
{
    spi_transaction_t t;

    if (data_len == 0) {
        return;
    }

    lcd_spi_setup_transaction(&t, data, data_len, true);
    lcd_spi_send_transaction(lcd, &t);
}

void lcd_spi_setup_transaction(spi_transaction_t* t, void* payload, int payload_length, bool is_data)
{
    lcd_spi_setup_transaction_ex(t, payload, payload_length, is_data, 0);
}

void lcd_spi_setup_transaction_ex(spi_transaction_t* t, void* payload, int payload_length, bool is_data, int flags)
{
    memset(&t, 0, sizeof(t));

    t->user = (void*)(is_data ? 1 : 0);
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
    lcd_t* lcd = (int)t->user >> 8;
    int dc = (int)t->user & 0x01;
    gpio_set_level(lcd->pins->dc, dc);
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

    lcd->width = width;
    lcd->height = height;
    lcd->bps = bps;

    lcd->current_fb = NULL;

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
