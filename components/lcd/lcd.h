#ifndef __LCD_H__
#define __LCD_H__

#include <esp_attr.h>
#include "graphics.h"

#define LCD_SPI_TRANS_IS_COMMAND 0
#define LCD_SPI_TRANS_IS_DATA 1

// #define DISPLAY_ILI
// #define DISPLAY_SSD1351
#define DISPLAY_SEPS525

#define PIN_NUM_MISO    12
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     14
#define PIN_NUM_CS      15
#define PIN_NUM_DC      26
#define PIN_NUM_RST     33

typedef struct lcd lcd_t;
typedef struct lcd_init_cmd lcd_init_cmd_t;
typedef void (*lcd_acquire_fb_cb_t)(lcd_t *lcd);
typedef void (*lcd_release_fb_cb_t)(lcd_t *lcd, void* fb);

struct lcd
{
    uint16_t width;
    uint16_t height;
    uint8_t bps;

    struct {
        gpio_num_t miso;
        gpio_num_t clk;
        gpio_num_t cs;
        gpio_num_t dc;
        gpio_num_t rst;
    } pins;

    struct {
        spi_device_handle_t device;
        spi_device_interface_config_t *devcfg;
        spi_bus_config_t *buscfg;
        uint8_t transactions_count;
        spi_transaction_t transactions[];
    } spi;
};

struct lcd_init_cmd
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes;
};

#if defined(DISPLAY_ILI)
DRAM_ATTR static const lcd_init_cmd_t lcd_init_cmds[] = {
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

#elif defined(DISPLAY_SEPS525)
DRAM_ATTR static const lcd_init_cmd_t lcd_init_cmds[] = {
    // display mode, analog reset
    {0x04, {0x01}, 1},

    // display mode, normal mode
    {0x04, {0x00}, 1},

    // display power - off
    {0x06, {0x00}, 1},

    // turn on internal oscillator using external resistor
    {0x02, {0x01}, 1},

    // 90 hz frame rate, divider 0 - b0011
    //{0x03, {0x30}, 1},
    // 120 hz frame rate, divider 0 - b1001
    {0x03, {0x90}, 1},

    // duty cycle 127
    {0x28, {0x7f}, 1},

    // start on line 0 - mapping ram display start line
    // missing in doc
    {0x29, {0x00}, 1},

    // mode set - BGR b10000000
    {0x13, {0x00}, 1},

    // rgb_if - 16 bit / MPU - it was 0x31
    {0x14, {0x11}, 1},

    // memory write mode - 8 bit, 65k
    {0x16, {0x66}, 1},

    // driving current r g b (uA)
    {0x10, {0x45}, 1},
    {0x11, {0x34}, 1},
    {0x12, {0x33}, 1},

    // precharge time r g b
    {0x08, {0x04}, 1},
    {0x09, {0x05}, 1},
    {0x0a, {0x05}, 1},

    // precharge current r g b (uA)
    {0x0b, {0x9d}, 1},
    {0x0c, {0x8c}, 1},
    {0x0d, {0x57}, 1},

    // reference voltage controlled by external resistor (from newhavendisplay doc)
    {0x80, {0x00}, 1},

    // display power - on
    {0x06, {0x01}, 1},

    // end
    {0, {0}, 0xff},
};

#elif defined(DISPLAY_SSD1351)

#define SSD1351_CMD_SETCOLUMN 		0x15
#define SSD1351_CMD_SETROW    		0x75
#define SSD1351_CMD_WRITERAM   		0x5C
#define SSD1351_CMD_READRAM   		0x5D
#define SSD1351_CMD_SETREMAP 		0xA0
#define SSD1351_CMD_STARTLINE 		0xA1
#define SSD1351_CMD_DISPLAYOFFSET 	0xA2
#define SSD1351_CMD_DISPLAYALLOFF 	0xA4
#define SSD1351_CMD_DISPLAYALLON  	0xA5
#define SSD1351_CMD_NORMALDISPLAY 	0xA6
#define SSD1351_CMD_INVERTDISPLAY 	0xA7
#define SSD1351_CMD_FUNCTIONSELECT 	0xAB
#define SSD1351_CMD_DISPLAYOFF 		0xAE
#define SSD1351_CMD_DISPLAYON     	0xAF
#define SSD1351_CMD_PRECHARGE 		0xB1
#define SSD1351_CMD_DISPLAYENHANCE	0xB2
#define SSD1351_CMD_CLOCKDIV 		0xB3
#define SSD1351_CMD_SETVSL 		    0xB4
#define SSD1351_CMD_SETGPIO 		0xB5
#define SSD1351_CMD_PRECHARGE2 		0xB6
#define SSD1351_CMD_SETGRAY 		0xB8
#define SSD1351_CMD_USELUT 		    0xB9
#define SSD1351_CMD_PRECHARGELEVEL 	0xBB
#define SSD1351_CMD_VCOMH 		    0xBE
#define SSD1351_CMD_CONTRASTABC		0xC1
#define SSD1351_CMD_CONTRASTMASTER	0xC7
#define SSD1351_CMD_MUXRATIO        0xCA
#define SSD1351_CMD_COMMANDLOCK     0xFD
#define SSD1351_CMD_HORIZSCROLL     0x96
#define SSD1351_CMD_STOPSCROLL      0x9E
#define SSD1351_CMD_STARTSCROLL     0x9F

DRAM_ATTR static const lcd_init_cmd_t lcd_init_cmds[] = {
    {SSD1351_CMD_COMMANDLOCK, {0x12}, 1},
    {SSD1351_CMD_COMMANDLOCK, {0xB1}, 1},
    {SSD1351_CMD_DISPLAYOFF, {0}, 0},
    {SSD1351_CMD_CLOCKDIV, {0xF1}, 1},
    {SSD1351_CMD_MUXRATIO, {0x7F}, 1},
    {SSD1351_CMD_SETREMAP, {0x74}, 1},
    {SSD1351_CMD_SETCOLUMN, {0x00, 0x7F}, 2},
    {SSD1351_CMD_SETROW, {0x00, 0x7F}, 2},
    {SSD1351_CMD_STARTLINE, {0x00}, 1},
    {SSD1351_CMD_DISPLAYOFFSET, {0x00}, 1},
    {SSD1351_CMD_SETGPIO, {0x00}, 1},
    {SSD1351_CMD_FUNCTIONSELECT, {0x01}, 1},
    {SSD1351_CMD_SETVSL, {0xA0, 0xB5, 0x55}, 3},
    {SSD1351_CMD_CONTRASTABC, {0xC8, 0x80, 0xC8}, 3},
    {SSD1351_CMD_CONTRASTMASTER, {0x0F}, 1},
    //{SSD1351_CMD_DISPLAYENHANCE, {0xA4, 0x00, 0x00}, 1},
    {SSD1351_CMD_USELUT, {0}, 0},
    {SSD1351_CMD_PRECHARGE, {0x34}, 1},
    {SSD1351_CMD_PRECHARGE2, {0x01}, 1},
    {SSD1351_CMD_VCOMH, {0x03}, 1},
    {SSD1351_CMD_NORMALDISPLAY, {0}, 0},
    {SSD1351_CMD_DISPLAYON, {0}, 0},
    {0, {0}, 0xff},
};
#endif

void _lcd_spi_transactions_send(spi_device_handle_t *spi, spi_transaction_t *transactions, uint16_t transactions_count);
void _lcd_spi_transactions_wait(spi_device_handle_t *spi);
void _lcd_send_lines(graphics_t *g, spi_device_handle_t *spi, uint16_t rect_x, uint16_t rect_y, uint16_t rect_width, uint16_t rect_height, uint16_t *fb_data);
void _lcd_update_display(spi_device_handle_t *spi, graphics_t *g);

void lcd_cmd(spi_device_handle_t* spi, const uint8_t cmd);
void lcd_data(spi_device_handle_t* spi, const uint8_t *data, int len);
void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
void lcd_init(spi_device_handle_t* spi);
void lcd_update_display(spi_device_handle_t *spi, graphics_t *g);

#endif
