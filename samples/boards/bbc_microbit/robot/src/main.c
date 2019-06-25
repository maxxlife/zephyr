/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <gpio.h>
#include <device.h>
#include <i2c.h>
#include <errno.h>
#include <pwm.h>
#include <display/mb_display.h>

//#define LEFT_LINE   EXT_P0_GPIO_PIN

#define I2C_SLV_ADDR 0x10
#define I2C_DEV "I2C_0"
#define EXT_P13_GPIO_PIN 23 /* P13, SPI1 SCK */
#define EXT_P14_GPIO_PIN 22 /* P14, SPI1 MISO */
//#define CONFIG_I2C_DW_CLOCK_SPEED I2C_SPEED_FAST

static struct device * gpio;
struct device * i2c_dev;
unsigned int left_line[1];
unsigned int right_line[1];
unsigned char buf[4];
unsigned char speed_hex[1];
static void line_detection(struct device * dev, struct gpio_callback * cb,
    u32_t pins) {
    gpio_pin_read(gpio, EXT_P13_GPIO_PIN, left_line);
    gpio_pin_read(gpio, EXT_P14_GPIO_PIN, right_line);
    /* printk("%d  %d\n", left_line[0], right_line[0]); */
}
int decimal_to_hex(int speed_decimal) {
    speed_hex[0] = (speed_decimal & 0x000000FF);
    return speed_hex[0];
}
/* control motors */
/* send digit > 0 motor turns forward */
/* send 0 motor stops */
/* send digit < 0 motor turns backward */
void motor_left_control(int left_speed) {
    if (left_speed < 0) {
        left_speed = left_speed * (-1);
        buf[0] = 0x00;
        buf[1] = 0x01;
        buf[2] = decimal_to_hex(left_speed);
        /* buf[3] = 0xA0; */
    } else {
        buf[0] = 0x00;
        buf[1] = 0x00;
        buf[2] = decimal_to_hex(left_speed);
      /* buf[3] = 0xA0; */
    }
    /* left motor */
    i2c_write(i2c_dev, buf, 3, 0x10);
}

void motor_right_control(int right_speed) {
if (right_speed < 0) {
    right_speed = right_speed * (-1);
    buf[0] = 0x02;
    buf[1] = 0x01;
    buf[2] = decimal_to_hex(right_speed);
/* no need to use for Maqueen DFRobot */
 /* buf[3] = 0xA0; */
} else {
    buf[0] = 0x02;
    buf[1] = 0x00;
    buf[2] = decimal_to_hex(right_speed);
/* no need to use for Maqueen DFRobot */
 /* buf[3] = 0xA0; */
}

/* right motor */
i2c_write(i2c_dev, buf, 3, 0x10);
}

void main(void) {
    static struct gpio_callback line_sensors;
    gpio = device_get_binding(SW0_GPIO_CONTROLLER);
    i2c_dev = device_get_binding(I2C_DEV);

    gpio_pin_configure(gpio, EXT_P13_GPIO_PIN, (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE));

    gpio_pin_configure(gpio, EXT_P14_GPIO_PIN, (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE));
    gpio_init_callback( & line_sensors, line_detection,
        BIT(EXT_P13_GPIO_PIN) | BIT(EXT_P14_GPIO_PIN));
    gpio_add_callback(gpio, & line_sensors);

    gpio_pin_enable_callback(gpio, EXT_P13_GPIO_PIN);
    gpio_pin_enable_callback(gpio, EXT_P14_GPIO_PIN);
    while (1) {
        if ((left_line[0] == 0) && (right_line[0] == 0)) {
            motor_left_control(200);
            motor_right_control(200);
        } else {
            if ((left_line[0] == 0) && (right_line[0] == 1)) {
                motor_left_control(0);
                motor_right_control(200);
                if ((left_line[0] == 1) && (right_line[0] == 1)) {
                    motor_left_control(0);
                    motor_right_control(200);
                }
            } else {
                if ((left_line[0] == 1) && (right_line[0] == 0)) {
                  motor_left_control(200);
                  motor_right_control(0);
                    if ((left_line[0] == 1) && (right_line[0] == 1)) {
                        motor_left_control(200);
                        motor_right_control(0);
                    }
                    if ((left_line[0] == 1) && (right_line[0] == 0)) {
                        motor_left_control(200);
                    } else {
                        motor_right_control(0);
                    }
                }
            }
        }
        }
    }
