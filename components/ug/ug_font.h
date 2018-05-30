#ifndef __UG_FONT_H__
#define __UG_FONT_H__

#include "ug_font_6x8.h"
#include "ug_font_6x10.h"
#include "ug_font_7x12.h"

typedef struct graphics graphics_t;
typedef struct font font_t;

struct font 
{
  int char_width;
  int char_height;
  const unsigned char **pixels;
};

void ug_font_draw_char(font_t *f, graphics_t *g, int x, int y, char ch, int front_color, int back_color);
void ug_font_print_string(font_t *f, graphics_t *g, char *str);
void ug_font_print_number(font_t *f, graphics_t *g, int number);
void ug_font_print_number_ex(font_t *f, graphics_t *g, int number, int base, int min_characters);

#endif