#include <stdlib.h>
#include "ug.h"
#include "ug_font.h"

void ug_font_draw_char(font_t *f, graphics_t *g, int x, int y, char ch, int front_color, int back_color)
{
    int i,j,k,c,bn;
    uint8_t b, bt;
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
    
    p.y = y;

    for (j = 0; j < f->char_height; j++) {
        p.x = x;
        c = f->char_width;
        for (i = 0; i < bn; i++) {
            b = *p++;
            for (k=0; (k < 8) && c; k++) {
                if (b & 0x01) {
                    ug_point(g, p, front_color);
                } else if (back_color >= 0) {
                    ug_point(g, p, back_color);
                }

                b >>= 1;
                c--;
            }
        }

        p.y++;
    }
}

void ug_font_print_string(font_t *f, graphics_t *g, char *str)
{
    while (*str) {
        ug_font_draw_char(
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

void ug_font_print_number(font_t *f, graphics_t *g, int number)
{
    ug_font_print_number_ex(f, g, number, 10, 0);
}

void ug_font_print_number_ex(font_t *f, graphics_t *g, int number, int base, int min_characters)
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

    ug_font_print_string(g, &temp[i + 1]);
}
