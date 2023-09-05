/*
 * RGB LED (WS2812) Strip control
 * WHowe <github.com/whowechina>
 */

#ifndef RGB_H
#define RGB_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"

void rgb_init();
void rgb_update();

uint32_t rgb32(uint32_t r, uint32_t g, uint32_t b, bool gamma_fix);

void rgb_set_colors(const uint32_t *colors, unsigned index, size_t num);
void rgb_set_color(unsigned index, uint32_t color);
void rgb_key_color(unsigned index, uint32_t color);
void rgb_gap_color(unsigned index, uint32_t color);

#endif