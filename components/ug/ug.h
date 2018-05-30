#ifndef __UG_H__
#define __UG_H__

#include <stdtypes.h>
#include <stdbool.h>

#define UG_STYLE_BORDER_TYPE_NORMAL				1

#define UG_STYLE_DRAW_COLOR_TYPE_NONE			0
#define UG_STYLE_DRAW_COLOR_TYPE_SOLID_COLOR	1
#define UG_STYLE_DRAW_COLOR_TYPE_CUSTOM			2

// https://github.com/xaelsouth/rtl-wmbus/tree/master/include/fixedptc
// https://github.com/andrewray/mirage-kfreebsd/tree/master/fixpt

#define UG_BUFFER_POS(g, x, y)  ((g->pitch * y) + x)
#define UG_COLOR_TO_RGB565(c)   ((uint16_t)(((c.b & 0xF8) << (11-3)) | ((c.g & 0xFC) << (5-2)) | ((c.r & 0xF8) >> 3)))

// General
typedef struct ug ug_t;
typedef struct ug_color ug_color_t;
typedef struct ug_point ug_point_t;
typedef struct ug_size ug_size_t;
typedef struct ug_rect ug_rect_t;
typedef struct ug_style ug_style_t;

// Style - Draw
typedef uint8_t ug_style_draw_color_type_t;
typedef struct ug_style_draw_color ug_style_draw_color_t;
typedef struct ug_style_draw_color_custom ug_style_draw_color_custom_t;
typedef struct ug_style_draw_color_solid_color ug_style_draw_color_solid_color_t;
typedef void (*ug_style_draw_color_custom_cb_t)(graphics_t*, ug_point_t*, ug_style_draw_color_t*, void*, ug_color_t*);

// Style - Borders
typedef uint8_t ug_style_border_type_t;
typedef struct ug_style_border ug_style_border_t;

// Style - Fill
typedef struct ug_style_fill ug_style_fill_t;

struct ug {
    uint16_t *framebuffers[];
    uint8_t framebuffers_count;
    int pitch;
    ug_size_t size
    ug_rect_t clip
};

struct ug_color {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
};

struct ug_point {
	int16_t x;
	int16_t y;
};

struct ug_size {
	uint16_t width;
	uint16_t height;
};
struct ug_rect {
	ug_point_t point;
	ug_size_t size;
};

struct ug_style_draw_color_custom {
	ug_style_draw_color_custom_cb_t cb;
	(void*) user_data;
};

struct ug_style_draw_color_solid_color {
	ug_color_t color;
};

struct ug_style_draw_color {
	ug_style_draw_color_type_t type;
	union type_data {
		ug_style_draw_color_solid_color_t solid;
		ug_style_draw_color_custom_t custom;
	}
};

struct ug_style_border {
	ug_style_border_type_t type;
	bool antialias;
	uint8_t thickness;
	ug_style_draw_color_t draw;
};

struct ug_style_fill {
	ug_style_draw_color_t draw;
};

struct ug_style {
	ug_style_border_t borders;
	ug_style_fill_t fill;
};

ug_t* ug_

uint16_t ug_color_apply_alphablend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha);
bool ug_is_point_in_rect(ug_point_t p, ug_rect_t r);
void ug_point_internal(ug_t *g, point_t p, ug_style_draw_color_t *style_draw_color);
void ug_line_x(ug_t *g, point_t p0, point_t p1, ug_style_draw_color_t *style_draw_color);
void ug_point(ug_t *g, ug_point_t p, color_t c);
void ug_line(ug_t *g, point_t p0, point_t p1, ug_style *style);
void ug_spinline(ug_t *g, point_t *points, uint8_t points_size, ug_style *style);
void ug_curve_bezier(ug_t *g, point_t *p0, point_t *p1, point_t *p2, ug_style *style);
void ug_curve_quadratic(ug_t *g, point_t *p0, point_t *p1, point_t *p2, point_t *p2, ug_style *style);
void ug_rect(ug_t *g, rect_t *r, ug_style *style);
void ug_triangle(ug_t *g, point_t *p0, point_t *p1, point_t *p2, ug_style *style);
void ug_circle(ug_t *g, point_t *p, uint8_t r, ug_style *style);
void ug_ellipse(ug_t *g, point_t *p, uint8_t r_a, uint8_t r_b, ug_style *style);
void ug_shape(ug_t *g, point_t *ps, uint8_t points, ug_style *style);
#endif
