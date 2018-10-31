/*
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "graphics.h"
#include "font.h"

#define TAG "GRAPHICS"

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

graphics_t *graphics_init(int w, int h)
{
    graphics_t *this = (graphics_t *)malloc(sizeof(graphics_t));

    this->xres = w;
    this->yres = h;
    this->font = 0;
    this->cursor_x = this->cursor_y = this->cursor_base_x = 0;
    this->font_front_color = 50;
    this->font_back_color = -1;
    this->background_color = GRAPHICS_COLOR_BLACK;

    int backbuffer_size_malloc = this->yres * this->xres * sizeof(uint16_t);
    ESP_LOGI(TAG, "backbuffer_size_malloc = %d", backbuffer_size_malloc);
    this->backbuffer = (uint16_t *)heap_caps_malloc(backbuffer_size_malloc, MALLOC_CAP_DMA);
    memset(this->backbuffer, 0, backbuffer_size_malloc);

    int frontbuffer_size_malloc = this->yres * this->xres * sizeof(uint16_t);
    ESP_LOGI(TAG, "frontbuffer_size_malloc = %d", frontbuffer_size_malloc);
    this->frontbuffer = (uint16_t *)heap_caps_malloc(frontbuffer_size_malloc, MALLOC_CAP_DMA);
    memset(this->frontbuffer, 0, frontbuffer_size_malloc);
    
    graphics_update_changed_rect_full(this);

    return this;
}

void graphics_free(graphics_t *this)
{
    free(this);
}

void graphics_set_background_color(graphics_t *this, uint16_t color)
{
    this->background_color = color;
}

void graphics_set_text_color(graphics_t *this, uint16_t front, uint16_t back)
{
    this->font_front_color = front;
    this->font_back_color = back;
}

void graphics_set_text_font(graphics_t *this, font_t *font)
{
    this->font = font;
}

void graphics_set_cursor(graphics_t *this, int x, int y)
{
    this->cursor_x = this->cursor_base_x = x;
    this->cursor_y = y;
}

uint16_t graphics_color(char r, char g, char b)
{
    uint16_t v = 0;

    v |= ((r >> 3) << 11);
    v |= ((g >> 2) << 5);
    v |= ((b >> 3) << 0);

    v = (v >> 8) | (v << 8);

    return v;
}

void graphics_begin(graphics_t *this)
{
    graphics_clear(this, this->background_color);
}

void graphics_end(graphics_t *this)
{
    void *temp1 = this->frontbuffer;
    this->frontbuffer = this->backbuffer;
    this->backbuffer = temp1;
}

void graphics_print_string(graphics_t *this, char *str)
{
    if (!this->font) {
        return;
    }

    while (*str) {
        if (*str >= 32 && *str < 128) {
            font_draw_char_2(
                this->font, 
                this, 
                this->cursor_x, 
                this->cursor_y, 
                *str, 
                this->font_front_color, 
                this->font_back_color);
        }

        this->cursor_x += this->font->char_width;
        if (this->cursor_x + this->font->char_width > this->xres || *str == '\n') {
            this->cursor_x = this->cursor_base_x;
            this->cursor_y += this->font->char_height;
        }

        str++;
    }
}

void graphics_print_number(graphics_t *this, int number)
{
    graphics_print_number_ex(this, number, 10, 0);
}

void graphics_print_number_ex(graphics_t *this, int number, int base, int min_characters)
{
    bool sign = number < 0;
    if (sign) {
        number = -number;
    }

    const char baseChars[] = "0123456789ABCDEF";
    char temp[33];
    temp[32] = 0;
    int i = 31;
    do {
        temp[i--] = baseChars[number % base];
        number /= base;
    } while (number > 0);
    if (sign) {
        temp[i--] = '-';
    }
    for (; i > 31 - min_characters; i--) {
        temp[i] = ' ';
    }

    graphics_print_string(this, &temp[i + 1]);
}

void graphics_clear(graphics_t *this, uint16_t color)
{
    graphics_clear_rect(this, color, 0, 0, this->xres - 1, this->yres - 1);
}

void graphics_clear_rect(graphics_t *this, uint16_t color, int left, int top, int right, int bottom)
{
    for (int y = top; y <= bottom; y++) {
        for (int x = left; x <= right; x++) {
            this->backbuffer[GRAPHICS_COORDS(this, y, x)] = color;
        }
    }

    graphics_update_changed_rect(this, left, top);
    graphics_update_changed_rect(this, right, bottom);
}

void graphics_update_changed_rect_full(graphics_t *this)
{
    this->changed_rect_top = 0;
    this->changed_rect_bottom = this->yres - 1;
    this->changed_rect_left = 0;
    this->changed_rect_right = this->xres - 1;
}

void graphics_update_changed_rect_reset(graphics_t *this)
{
    this->changed_rect_top = -1;
    this->changed_rect_bottom = -1;
    this->changed_rect_left = -1;
    this->changed_rect_right = -1;
}

void graphics_update_changed_rect(graphics_t *this, int x, int y)
{
    if (y < this->changed_rect_top || this->changed_rect_top == -1) {
        this->changed_rect_top = y;
    }

    if (y > this->changed_rect_bottom || this->changed_rect_bottom == -1) {
        this->changed_rect_bottom = y;
    }

    if (x < this->changed_rect_left || this->changed_rect_left == -1) {
        this->changed_rect_left = x;
    }

    if (x > this->changed_rect_right || this->changed_rect_right == -1) {
        this->changed_rect_right = x;
    }
}

void graphics_dot_fast(graphics_t *this, int x, int y, uint16_t color)
{
    graphics_update_changed_rect(this, x, y);
    this->backbuffer[GRAPHICS_COORDS(this, y, x)] = color;
}

void graphics_dot(graphics_t *this, int x, int y, uint16_t color)
{
    if ((unsigned int)x < this->xres && (unsigned int)y < this->yres) {
        graphics_update_changed_rect(this, x, y);
        this->backbuffer[GRAPHICS_COORDS(this, y, x)] = color;
    }
}

void graphics_dot_add(graphics_t *this, int x, int y, uint16_t color)
{
    if ((unsigned int)x < this->xres && (unsigned int)y < this->yres) {
        graphics_update_changed_rect(this, x, y);
        this->backbuffer[GRAPHICS_COORDS(this, y, x)] = color + this->backbuffer[GRAPHICS_COORDS(this, y, x)];
    }
}

uint16_t graphics_get(graphics_t *this, int x, int y)
{
    if ((unsigned int)x < this->xres && (unsigned int)y < this->yres) {
        return this->backbuffer[GRAPHICS_COORDS(this, y, x)];
    }
    return 0;
}

void graphics_x_line(graphics_t *this, int x0, int x1, int y, uint16_t color)
{
    if (x0 > this->xres) {
        x0 = this->xres;
    } else if (x0 < 0) {
        x0 = 0;
    }

    if (x1 > this->xres) {
        x1 = this->xres;
    } else if (x1 < 0) {
        x1 = 0;
    }

    if (y > this->yres) {
        y = this->yres;
    } else if (y < 0) {
        y = 0;
    }

    if (x0 > x1) {
        int xb = x0;
        x0 = x1;
        x1 = xb;
    }

    graphics_update_changed_rect(this, x0, y);
    graphics_update_changed_rect(this, x1, y);

    uint32_t pos = GRAPHICS_COORDS(this, y, x0);
    for (int x = x0; x < x1; x++) {
        this->backbuffer[pos++] = color;
    }
}

void graphics_line(graphics_t *this, int x1, int y1, int x2, int y2, uint16_t color)
{
    if (x1 > this->xres) {
        x1 = this->xres;
    } else if (x1 < 0) {
        x1 = 0;
    }

    if (y1 > this->yres) {
        y1 = this->yres;
    } else if (y1 < 0) {
        y1 = 0;
    }

    if (x2 > this->xres) {
        x2 = this->xres;
    } else if (x2 < 0) {
        x2 = 0;
    }

    if (y2 > this->yres) {
        y2 = this->yres;
    } else if (y2 < 0) {
        y2 = 0;
    }

    int x, y, xe, ye;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int dx1 = labs(dx);
    int dy1 = labs(dy);
    int px = 2 * dy1 - dx1;
    int py = 2 * dx1 - dy1;

    if (dy1 <= dx1) {
        if (dx >= 0) {
            x = x1;
            y = y1;
            xe = x2;
        } else {
            x = x2;
            y = y2;
            xe = x1;
        }

        graphics_dot_fast(this, x, y, color);
        for (; x < xe; x++) {
            if (px < 0) {
                px = px + 2 * dy1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    y = y + 1;
                } else {
                    y = y - 1;
                }

                px = px + 2 * (dy1 - dx1);
            }

            graphics_dot_fast(this, x, y, color);
        }
    } else {
        if (dy >= 0) {
            x = x1;
            y = y1;
            ye = y2;
        } else {
            x = x2;
            y = y2;
            ye = y1;
        }

        graphics_dot_fast(this, x, y, color);

        for (; y < ye; y++) {
            if (py <= 0) {
                py = py + 2 * dx1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    x = x + 1;
                } else {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }

            graphics_dot_fast(this, x, y, color);
        }
    }
}

void graphics_fill_rect(graphics_t *this, int x, int y, int w, int h, uint16_t color)
{
    if (x < 0)
    {
        w += x;
        x = 0;
    }

    if (y < 0)
    {
        h += y;
        y = 0;
    }

    if (x + w > this->xres)
    {
        w = this->xres - x;
    }

    if (y + h > this->yres)
    {
        h = this->yres - y;
    }

    for (int j = y; j < y + h; j++)
    {
        for (int i = x; i < x + w; i++)
        {
            graphics_dot_fast(this, i, j, color);
        }
    }
}

void graphics_rect(graphics_t *this, int x, int y, int w, int h, uint16_t color)
{
    graphics_fill_rect(this, x, y, w, 1, color);
    graphics_fill_rect(this, x, y, 1, h, color);
    graphics_fill_rect(this, x, y + h - 1, w, 1, color);
    graphics_fill_rect(this, x + w - 1, y, 1, h, color);
}

void graphics_draw_circle(graphics_t *this, int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    graphics_dot_fast(this, x0, y0 + r, color);
    graphics_dot_fast(this, x0, y0 - r, color);
    graphics_dot_fast(this, x0 + r, y0, color);
    graphics_dot_fast(this, x0 - r, y0, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        graphics_dot_fast(this, x0 + x, y0 + y, color);
        graphics_dot_fast(this, x0 - x, y0 + y, color);
        graphics_dot_fast(this, x0 + x, y0 - y, color);
        graphics_dot_fast(this, x0 - x, y0 - y, color);
        graphics_dot_fast(this, x0 + y, y0 + x, color);
        graphics_dot_fast(this, x0 - y, y0 + x, color);
        graphics_dot_fast(this, x0 + y, y0 - x, color);
        graphics_dot_fast(this, x0 - y, y0 - x, color);
    }
}

void graphics_draw_triangle(graphics_t *this, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    graphics_line(this, x0, y0, x1, y1, color);
    graphics_line(this, x0, y0, x2, y2, color);
    graphics_line(this, x2, y2, x1, y1, color);
}
*/