/*
#ifndef __GRAPHICS_H__
#define __GRAPHICS_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct graphics graphics_t;
typedef struct font font_t;
typedef struct triangle_tree triangle_tree_t;

struct graphics
{
  int xres;
  int yres;
  uint16_t *backbuffer;
  uint16_t *frontbuffer;
  int cursor_x, cursor_y, cursor_base_x;
  uint16_t background_color, font_front_color, font_back_color;
  font_t *font;
  int changed_rect_top, changed_rect_bottom, changed_rect_left, changed_rect_right;
};

#define GRAPHICS_COORDS(g, y, x) ((g->xres * y) + x)

#define GRAPHICS_COLOR_BLACK (graphics_color(0, 0, 0))
#define GRAPHICS_COLOR_WHITE (graphics_color(255, 255, 255))
#define GRAPHICS_COLOR_RED (graphics_color(255, 0, 0))
#define GRAPHICS_COLOR_GREEN (graphics_color(0, 255, 0))
#define GRAPHICS_COLOR_BLUE (graphics_color(0, 0, 255))

graphics_t* graphics_init(int w, int h);
void graphics_set_background_color(graphics_t *this, uint16_t color);
void graphics_set_text_color(graphics_t* this, uint16_t front, uint16_t back);
void graphics_set_text_font(graphics_t *this, font_t* font);
void graphics_set_cursor(graphics_t *this, int x, int y);
void graphics_begin(graphics_t *this);
void graphics_end(graphics_t *this);
void graphics_print_string(graphics_t *this, char *str);
void graphics_print_number(graphics_t *this, int number);
void graphics_print_number_ex(graphics_t *this, int number, int base, int min_characters);
void graphics_clear(graphics_t *this, uint16_t color);
void graphics_clear_rect(graphics_t *this, uint16_t color, int left, int top, int right, int bottom);
void graphics_update_changed_rect_full(graphics_t *this);
void graphics_update_changed_rect_reset(graphics_t *this);
void graphics_update_changed_rect(graphics_t *this, int x, int y);
void graphics_dot_fast(graphics_t *this, int x, int y, uint16_t color);
void graphics_dot(graphics_t* this, int x, int y, uint16_t color);
void graphics_dot_add(graphics_t* this, int x, int y, uint16_t color);
uint16_t graphics_get(graphics_t* this, int x, int y);
void graphics_x_line(graphics_t* this, int x0, int x1, int y, uint16_t color);
void graphics_line(graphics_t* this, int x1, int y1, int x2, int y2, uint16_t color);
void graphics_fill_rect(graphics_t *this, int x, int y, int w, int h, uint16_t color);
void graphics_rect(graphics_t *this, int x, int y, int w, int h, uint16_t color);
uint16_t graphics_color(char r, char g, char b);
void graphics_draw_circle(graphics_t *this, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void graphics_draw_triangle(graphics_t *this, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
#endif

*/