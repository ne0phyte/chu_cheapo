/*
 * I2C Hub Using PCA9548A
 * WHowe <github.com/whowechina>
 */


#ifndef I2C_HUB_H
#define I2C_HUB_H

#include "hardware/i2c.h"
#include "board_defs.h"

#define I2C_HUB_ADDR 0x70
static inline void i2c_hub_init()
{
    // pull up gpio I2C_HUB_EN
    gpio_init(I2C_HUB_EN);
    gpio_set_dir(I2C_HUB_EN, GPIO_OUT);
    gpio_put(I2C_HUB_EN, 1);

    // reset at start to avoid failed power-on
    sleep_ms(10);
    gpio_put(I2C_HUB_EN, 0);
    sleep_us(50);
    gpio_put(I2C_HUB_EN, 1);
}

static inline void i2c_select(i2c_inst_t *i2c_port, uint8_t chn)
{
    i2c_write_blocking_until(i2c_port, I2C_HUB_ADDR, &chn, 1, false, from_us_since_boot(time_us_64() + 1000));
}

#endif
