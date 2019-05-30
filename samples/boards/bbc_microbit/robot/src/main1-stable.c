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

#include <display/mb_display.h>
#define I2C_SLV_ADDR	0x10

#define I2C_DEV "I2C_0"
//#define I2C_SPEED_FAST
//CONFIG_BOARD_BBC_MICROBIT

void main(void)
{
  unsigned char buf[4];

  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x14;//0xfa
  buf[3] = 0xA0;
int speed=255;
    struct device *i2c_dev;

    printk("Starting i2c scanner...\n");

    i2c_dev = device_get_binding(I2C_DEV);
    i2c_write(i2c_dev, buf , 4, 0x10);//left motor
      buf[0] = 0x02;
      buf[1] = 0x00;
    i2c_write(i2c_dev, buf , 4, 0x10);//right motor

  /*
	Wire.beginTransmission(0x10);
  Wire.write((byte)0x00);
  Wire.write((byte)0x00);
  Wire.write(_m1spd); //speed
  Wire.endTransmission();
  Wire.beginTransmission(0x10);
  Wire.write((byte)0x02);
  Wire.write((byte)0x00);
  Wire.write(_m2spd); //speed
  Wire.endTransmission();
  */
     }
