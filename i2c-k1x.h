/* SPDX-License-Identifier: GPL-2.0 */
/*
 * i2c-stm32.h
 *
 * Author: Troy Mitchell <troymitchell988@gmail.com>
 *
 */

struct spacemit_k1_i2c {
	struct device		*dev;
	struct i2c_adapter	adap;
};
