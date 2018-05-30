#include <stdlib.h>
#include "font.h"
#include "graphics.h"

font_t *font_init(int char_width, int char_height, const unsigned char *pixels)
{
    font_t *f = (font_t *)malloc(sizeof(font_t));

    f->char_width = char_width;
    f->char_height = char_height;
    f->pixels = pixels;

    return f;
}

void font_free(font_t *f)
{
    free(f);
}

void font_draw_char(font_t *f, graphics_t *g, int x, int y, char ch, int front_color, int back_color)
{
    int i,j,k,xo,yo,c,bn;
    uint8_t b, bt;
    unsigned char* p;
    ug_point p;

    bt = (uint8_t)ch;
    
    switch (bt)
    {
        case 0xF6: bt = 0x94; break; // ö
        case 0xD6: bt = 0x99; break; // Ö
        case 0xFC: bt = 0x81; break; // ü
        case 0xDC: bt = 0x9A; break; // Ü
        case 0xE4: bt = 0x84; break; // ä
        case 0xC4: bt = 0x8E; break; // Ä
        case 0xB5: bt = 0xE6; break; // µ
        case 0xB0: bt = 0xF8; break; // °
    }
    
    yo = y;
    bn = f->char_width;
    if (!bn) {
        return;
    }

    bn >>= 3;
    if (f->char_width % 8) {
        bn++;
    }

    p = f->pixels;
    p += bt * f->char_height * bn;

    for (j = 0; j < f->char_height; j++) {
        xo = x;
        c = f->char_width;
        for (i = 0; i < bn; i++) {
            b = *p++;
            for (k=0; (k<8) && c; k++) {
                ug_point.x = xo;
                ug_point.y = yo;
                
                if (b & 0x01) {
                    ug_point(g, xo, yo, front_color);
                } else if (back_color >= 0) {
                    ug_point(g, xo, yo, back_color);
                }

                b >>= 1;
                xo++;
                c--;
            }
        }

        yo++;
    }
}

void font_print_string(font_t *f, graphics_t *g, char *str)
{
    while (*str) {
        font_draw_char(
            g->font, 
            g, 
            g->cursor_x, 
            g->cursor_y, 
            *str, 
            g->font_front_color, 
            g->font_back_color);

        g->cursor_x += g->font->char_width;
        if (g->cursor_x + g->font->char_width > g->xres || *str == '\n') {
            g->cursor_x = g->cursor_base_x;
            g->cursor_y += g->font->char_height;
        }

        str++;
    }
}

void font_print_number(font_t *f, graphics_t *g, int number)
{
    font_print_number_ex(f, g, number, 10, 0);
}

void font_print_number_ex(font_t *f, graphics_t *g, int number, int base, int min_characters)
{
    const char baseChars[] = "0123456789ABCDEF";
    bool sign = number < 0;
    if (sign) {
        number = -number;
    }

    char temp[33];
    temp[32] = NULL;
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

    font_print_string(g, &temp[i + 1]);
}
