/*
 * Chu Pico Silder Keys
 * WHowe <github.com/whowechina>
 * 
 * MPR121 CapSense based Keys
 */

#include "slider.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "board_defs.h"

#include "config.h"
#include "mpr121.h"

#define MPR121_ADDR 0x5A

static uint16_t readout[36];
// static uint16_t touch[3];
static uint32_t touch_status;
static unsigned touch_count[36];


static uint8_t key_order[] = {
    // PCB key order
    // 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
    // 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31

    // SensorA: 16, 17, 18,  0,  1,  2,  3,  4,  5, 19, 20, 21
    // SensorB: 22, 23, 24,  6,  7,  8,  9, 10, 11, 12, 26, 27
    // SensorC: 28, 29, 12, 13, 14, 15, 30, 31

    // We want to transform to this order:
    // 0  2  4  6  8  10  12  14  16  18  20  22  24  26  28  30
    // 1  3  5  7  9  11  13  15  17  19  21  23  25  27  29  31

    // Lookup table for bit -> key index
    1,  3,  5,  0,  2,  4,  6,  8,  10,  7,  9,  11,
    13, 15, 17, 12, 14, 16, 18, 20, 22, 19, 21,  23,
    25, 27, 24, 26, 28, 30, 29, 31
};

void slider_update()
{
    static uint32_t last_touched;
    uint32_t tmp = 0;
    tmp |= mpr121_touched(MPR121_ADDR);
    tmp |= mpr121_touched(MPR121_ADDR + 1) << 12;
    tmp |= mpr121_touched(MPR121_ADDR + 2) << 24;

    touch_status = 0;
    for (int i = 0; i < 32; i++) {
        uint32_t val = tmp & (1 << i);
        val = (val == 0 ? 0 : 1);

        uint8_t key_index = key_order[i];
        touch_status |= (val << key_index);
    }
    uint32_t just_touched = touch_status & ~last_touched;
    for (int i = 0; i < 32; i++) {
        if (just_touched & (1 << i)) {
            touch_count[i]++;
        }
    }
}


void slider_init()
{
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    for (int m = 0; m < 3; m++) {
        mpr121_init(MPR121_ADDR + m);
    }
    slider_update_config();
}

const uint16_t *slider_raw()
{
    uint16_t tmp[36];
    mpr121_raw(MPR121_ADDR, tmp, 12);
    mpr121_raw(MPR121_ADDR + 1, tmp + 12, 12);
    mpr121_raw(MPR121_ADDR + 2, tmp + 24, 12);

    for (int i = 0; i < 32; i++) {
        readout[key_order[i]] = tmp[i];
    }
    return readout;
}

bool slider_touched(unsigned key)
{
    if (key >= 32) {
        return 0;
    }

    return touch_status & (1 << key);
}

unsigned slider_count(unsigned key)
{
    if (key >= 32) {
        return 0;
    }
    return touch_count[key];
}

void slider_reset_stat()
{
    memset(touch_count, 0, sizeof(touch_count));
}

void slider_update_config()
{
    for (int m = 0; m < 3; m++) {
        mpr121_debounce(MPR121_ADDR + m, chu_cfg->sense.debounce_touch,
                                         chu_cfg->sense.debounce_release);
        mpr121_sense(MPR121_ADDR + m, chu_cfg->sense.global,
                                      chu_cfg->sense.keys + m * 12,
                                      m != 2 ? 12 : 8);
        mpr121_filter(MPR121_ADDR + m, chu_cfg->sense.filter >> 6,
                                       (chu_cfg->sense.filter >> 4) & 0x03,
                                       chu_cfg->sense.filter & 0x07);
    }
}
