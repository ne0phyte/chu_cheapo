/*
 * Chu Pico Air Sensor
 * WHowe <github.com/whowechina>
 * 
 * Use ToF (Sharp GP2Y0E03) to detect air keys
 */

#include "air.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "board_defs.h"
#include "config.h"

#define MEASURING_INTERVAL 20000
#define MAX_DISTANCE 500
#define SMOOTHING_BUFFER_SIZE 3

typedef struct {
    uint trig_pin;
    uint echo_pin;
    volatile uint64_t start_time;
    volatile uint64_t end_time;
    volatile bool measuring;
    volatile bool ready;
    float distance;
    float distance_smooth;
    float smoothing_buffer[SMOOTHING_BUFFER_SIZE];
    int buffer_index;
    uint64_t last_update;
} hc_sr04_sensor_t;

hc_sr04_sensor_t sensor;

void echo_irq_handler(uint gpio, uint32_t events) {
    if (sensor.echo_pin == gpio && sensor.measuring) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            sensor.start_time = time_us_64();
        } else if (events & GPIO_IRQ_EDGE_FALL) {
            sensor.end_time = time_us_64();
            sensor.measuring = false;
            sensor.ready = true;
        }
    }
}

void init_sensor(hc_sr04_sensor_t *sensor, uint trig_pin, uint echo_pin) {
    sensor->trig_pin = trig_pin;
    sensor->echo_pin = echo_pin;
    sensor->measuring = false;
    sensor->ready = false;
    sensor->distance_smooth = -1;  // Initialize to an impossible value to ensure the first measurement is printed
    sensor->buffer_index = 0;

    for (int i = 0; i < SMOOTHING_BUFFER_SIZE; i++) {
        sensor->smoothing_buffer[i] = 0;
    }

    gpio_init(sensor->trig_pin);
    gpio_set_dir(sensor->trig_pin, GPIO_OUT);
    gpio_put(sensor->trig_pin, 0);

    gpio_init(sensor->echo_pin);
    gpio_set_dir(sensor->echo_pin, GPIO_IN);
    gpio_set_irq_enabled_with_callback(sensor->echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &echo_irq_handler);
}

void trigger_sensor(hc_sr04_sensor_t *sensor) {
    gpio_put(sensor->trig_pin, 1);
    sleep_us(10);
    gpio_put(sensor->trig_pin, 0);
    sensor->measuring = true;
    sensor->ready = false;
}

void update_sensor(hc_sr04_sensor_t *sensor) {
    if ((time_us_64() - sensor->last_update < MEASURING_INTERVAL)) {
        return;
    }
    sensor->last_update = time_us_64();

    if (sensor->ready) {
        sensor->distance = 10.0f * (((sensor->end_time - sensor->start_time) * 0.0343) / 2.0);
        if (sensor->distance > MAX_DISTANCE) {
            sensor->distance = 0;
        }
        sensor->smoothing_buffer[sensor->buffer_index] = sensor->distance;
        sensor->buffer_index = (sensor->buffer_index + 1) % SMOOTHING_BUFFER_SIZE;

        // Calculate the average distance
        float sum = 0;
        for (int i = 0; i < SMOOTHING_BUFFER_SIZE; i++) {
            sum += sensor->smoothing_buffer[i];
        }
        float average_distance = sum / SMOOTHING_BUFFER_SIZE;

        if (average_distance < chu_cfg->tof.offset || average_distance > (chu_cfg->tof.offset + 6 * chu_cfg->tof.pitch)) {
            //printf("[%d] Out of range ignored: %d cm\n", sensor->echo_pin, distance_cm);
            average_distance = 0;
        } else {
            //printf("[%d] Distance: %d cm\n", sensor->echo_pin, distance_cm);
        }
        sensor->distance_smooth = average_distance;
    }
    trigger_sensor(sensor);
}


void air_init()
{
    init_sensor(&sensor, 0, 1);
}

size_t air_num()
{
    return 5;
}

static inline uint8_t air_bits(int dist, int offset)
{
    if (dist < offset) {
        return 0;
    }

    int pitch = chu_cfg->tof.pitch;
    int index = (dist - offset) / pitch;
    if (index >= 6) {
        return 0;
    }

    return 1 << index;
}

uint8_t air_bitmap()
{
    int offset = chu_cfg->tof.offset;
    int pitch = chu_cfg->tof.pitch;
    int distance = (int)sensor.distance_smooth;
    uint8_t bitmap = 0;
    bitmap |= air_bits(distance, offset) | air_bits(distance, offset + pitch);
    return bitmap;
}

unsigned air_value(uint8_t index)
{
    int offset = chu_cfg->tof.offset;
    int pitch = chu_cfg->tof.pitch;
    int distance = (int)sensor.distance_smooth;

    int sensor_index = (distance - offset) / pitch;
    if (sensor_index >= 6) {
        return 0;
    } else {
        return sensor_index + 1;
    }
}

void air_update()
{
    update_sensor(&sensor);
}

uint16_t air_raw(uint8_t index)
{
    return (int)sensor.distance_smooth;
}
