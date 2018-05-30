#include <stdtypes.h>
#include <stdbool.h>

#include "ug.h"

uint16_t ug_color_apply_alphablend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t)((result >> 16) | result);
}

bool ug_is_point_in_rect(ug_point_t p, ug_rect_t r)
{
	return
		p.x < r.point.x || p.x > r.point.x + r.size.w
		||
		p.y < r.point.y || p.y > r.point.y + r.size.h
		? false
		: true;
}

void ug_point_internal(ug_t *g, point_t p, ug_style_draw_color_t *style_draw_color)
{
    ug_color c;

    if (style_draw_color->type == UG_STYLE_DRAW_COLOR_TYPE_NONE) {
        return;
    } else if (style_draw_color->type == UG_STYLE_DRAW_COLOR_TYPE_SOLID_COLOR) {
        c = style_draw_color.solid.color;
    } else if (style_draw_color->type == UG_STYLE_DRAW_COLOR_TYPE_CUSTOM) {
        style_draw_color.custom.cb(g, &p, style_draw_color, style_draw_color.custom.user_data, &c);
    }

    uint32_t pixel_pos = UG_BUFFER_POS(g, p.x, p.y);
    uint16_t v = UG_COLOR_TO_RGB565(c);
    if (c.a < 255) {
        v = ug_color_apply_alphablend_rgb565(v, g->fb_pixels[pixel_pos], c.a);
    }

    g->fb_pixels[pixel_pos] = v;
}

void ug_line_x(ug_t *g, point_t p0, point_t p1, ug_style_draw_color_t *style_draw_color)
{
    uint16_t x0, uint16_t x1;

    if (x0 < x1) {
        x0 = p0.x;
        x1 = p1.x;
    } else {
        x1 = p0.x;
        x0 = p1.x;
    }

    x0 = max(0, x1);
    x1 = min(g->width, x2);

    for (p0.x = x0; p0.x < x1; p0.x++) {
        ug_point_internal(g, p0, style_draw_color)
    }
}

void ug_point(ug_t *g, ug_point_t p, color_t c)
{
	if (!ug_is_point_in_rect(p, g->clip)) {
		return;
	}

	ug_point_internal(g, p, c);
}

void ug_line(ug_t *g, point_t p0, point_t p1, ug_style *style)
{
    if (
        (p0.x < g->clip.point.x && p1.x < g->clip.point.x)
        ||
        (p0.x >= w && p1.x >= w)
        ||
        (p0.y < g->clip.point.y && p1.y < g->clip.point.y)
        ||
        (p0.y >= h && p1.y >= h)
    ) {
        return;
    }
    
    int dx =  abs(p1.x - p0.x), sx = p0.x < p1.x ? 1 : -1;
    int dy = -abs(p1.y - p0.y), sy = p0.y < p1.y ? 1 : -1; 
    int err = dx + dy, e2;

    for (;;) {
        ug_point_internal(g, color, p0.x, p0.y);

        if (p0.x == p1.x && p0.y == p1.y) {
            break;
        }

        e2 = 2 * err;
        
        if (e2 >= dy) {
            err += dy;
            p0.x += sx;
        }

        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ug_spinline(ug_t *g, point_t *points, uint8_t points_size, ug_style *style)
{
    if (points_size < 2) {
        return;
    }

    for (int i = 1; i < points_size; i++) {
        ug_line(g, points[i - 1, points[i], style);
    }
}

void ug_curve_bezier(ug_t *g, point_t *p0, point_t *p1, point_t *p2, ug_style *style)
{
    // TODO
}

void ug_curve_quadratic(ug_t *g, point_t *p0, point_t *p1, point_t *p2, point_t *p2, ug_style *style)
{
    // TODO
}

void ug_rect(ug_t *g, rect_t *r, ug_style *style)
{
    // TODO
}

void ug_triangle(ug_t *g, point_t *p0, point_t *p1, point_t *p2, ug_style *style)
{
    // REFACTOR

    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if (y0 == y2) {
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        graphics_draw_x_line(g, color, a, b, y0);
        return;
    }

    int16_t
        dx01 = x1 - x0,
        dy01 = y1 - y0,
        dx02 = x2 - x0,
        dy02 = y2 - y0,
        dx12 = x2 - x1,
        dy12 = y2 - y1;

    int32_t
        sa   = 0,
        sb   = 0;

    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if(a > b) _swap_int16_t(a, b);
        graphics_draw_x_line(g, color, a, b, y);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if(a > b) _swap_int16_t(a, b);
        graphics_draw_x_line(g, color, a, b, y);
    }
}

void ug_circle(ug_t *g, point_t *p, uint8_t r, ug_style *style)
{
    /*
    // REFACTOR


    int x = radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (radius << 1);

    while (x >= y) {
        graphics_draw_point(g, color, x0 + x, y0 + y);
        graphics_draw_point(g, color, x0 + y, y0 + x);
        graphics_draw_point(g, color, x0 - y, y0 + x);
        graphics_draw_point(g, color, x0 - x, y0 + y);
        graphics_draw_point(g, color, x0 - x, y0 - y);
        graphics_draw_point(g, color, x0 - y, y0 - x);
        graphics_draw_point(g, color, x0 + y, y0 - x);
        graphics_draw_point(g, color, x0 + x, y0 - y);

        if (err <= 0) {
            y++;
            err += dy;
            dy += 2;
        }
        
        if (err > 0) {
            x--;
            dx += 2;
            err += dx - (radius << 1);
        }
    }

    // REFACTOR

    int xo = radius_outer;
    int xi = radius_inner;
    int y = 0;
    int erro = 1 - xo;
    int erri = 1 - xi;

    while(xo >= y) {
        graphics_draw_x_line(g, color, x0 + xi, x0 + xo, y0 + y);
        graphics_draw_y_line(g, color, x0 + y,  y0 + xi, y0 + xo);
        graphics_draw_x_line(g, color, x0 - xo, x0 - xi, y0 + y);
        graphics_draw_y_line(g, color, x0 - y,  y0 + xi, y0 + xo);
        graphics_draw_x_line(g, color, x0 - xo, x0 - xi, y0 - y);
        graphics_draw_y_line(g, color, x0 - y,  y0 - xo, y0 - xi);
        graphics_draw_x_line(g, color, x0 + xi, x0 + xo, y0 - y);
        graphics_draw_y_line(g, color, x0 + y,  y0 - xo, y0 - xi);

        y++;

        if (erro < 0) {
            erro += 2 * y + 1;
        } else {
            xo--;
            erro += 2 * (y - xo + 1);
        }

        if (y > radius_inner) {
            xi = y;
        } else {
            if (erri < 0) {
                erri += 2 * y + 1;
            } else {
                xi--;
                erri += 2 * (y - xi + 1);
            }
        }
    }*/
}

void ug_ellipse(ug_t *g, point_t *p, uint8_t r_a, uint8_t r_b, ug_style *style)
{
    /*
    // REFACTOR

    thickness--;

    uint8_t x0_inner = x - width_radius;
    uint8_t y0_inner = y - height_radius;
    uint8_t x1_inner = x + width_radius;
    uint8_t y1_inner = y + height_radius;

    uint8_t x0_outer = x0_inner - thickness;
    uint8_t y0_outer = y0_inner - thickness;
    uint8_t x1_outer = x1_inner + thickness;
    uint8_t y1_outer = y1_inner + thickness;

    int width_outer = abs (x1_outer - x0_outer), height_outer = abs (y1_outer - y0_outer), width_bit1_outer = height_outer & 1;
    long dx_outer = 4 * (1 - width_outer) * height_outer * height_outer, dy_outer = 4 * (width_bit1_outer + 1) * width_outer * width_outer;
    long err_outer = dx_outer + dy_outer + width_bit1_outer * width_outer * width_outer, e2_outer;

    if (x0_outer > x1_outer) {
        x0_outer = x1_outer; x1_outer += width_outer;
    }

    if (y0_outer > y1_outer) {
        y0_outer = y1_outer;
    }

    y0_outer += (height_outer + 1) / 2;
    y1_outer = y0_outer - width_bit1_outer;
    width_outer *= 8 * width_outer;
    width_bit1_outer = 8 * height_outer * height_outer;

    int width_inner = abs (x1_inner - x0_inner), height_inner = abs (y1_inner - y0_inner), width_bit1_inner = height_inner & 1;
    long dx_inner = 4 * (1 - width_inner) * height_inner * height_inner, dy_inner = 4 * (width_bit1_inner + 1) * width_inner * width_inner;
    long err_inner = dx_inner + dy_inner + width_bit1_inner * width_inner * width_inner, e2_inner;

    if (x0_inner > x1_inner) {
        x0_inner = x1_inner; x1_inner += width_inner;
    }

    if (y0_inner > y1_inner) {
        y0_inner = y1_inner;
    }

    y0_inner += (height_inner + 1) / 2;
    y1_inner = y0_inner - width_bit1_inner;
    width_inner *= 8 * width_inner;
    width_bit1_inner = 8 * height_inner * height_inner;

    do {
        for (int y = y0_inner; y <= y0_outer; y++) {
            graphics_draw_x_line(g, color, x1_inner, x1_outer, y);
        }

        for (int y = y0_inner; y <= y0_outer; y++) {
            graphics_draw_x_line(g, color, x0_outer, x0_inner, y);
        }

        for (int y = y1_outer; y <= y1_inner; y++) {
            graphics_draw_x_line(g, color, x1_inner, x1_outer, y);
        }

        for (int y = y1_outer; y <= y1_inner; y++) {
            graphics_draw_x_line(g, color, x0_outer, x0_inner, y);
        }

        e2_outer = 2 * err_outer;

        if (e2_outer >= dx_outer) {
            x0_outer++;
            x1_outer--;
            err_outer += dx_outer += width_bit1_outer;
        }

        if (e2_outer <= dy_outer) {
            y0_outer++;
            y1_outer--;
            err_outer += dy_outer += width_outer;
        }
        
        e2_inner = 2 * err_inner;

        if (e2_inner >= dx_inner) {
            x0_inner++;
            x1_inner--;
            err_inner += dx_inner += width_bit1_inner;
        }

        if (e2_inner <= dy_inner) {
            y0_inner++;
            y1_inner--;
            err_inner += dy_inner += width_inner;
        }
    } while (x0_outer <= x1_outer);
    */
}

void ug_shape(ug_t *g, point_t *ps, uint8_t points, ug_style *style)
{
    // TODO
}

#endif