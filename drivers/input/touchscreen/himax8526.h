
/* Himax Android Driver Sample Code Ver 1.4 Himax verions of driver
 *
 * Copyright (C) 2013 Himax Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define HIMAX_X_MAX 2240
#define HIMAX_Y_MAX 1408
#define HIMAX_X_MAX_370T  2112
#define HIMAX_Y_MAX_370T  1280




#ifndef _LINUX_HIMAX_TS_H
#define _LINUX_HIMAX_TS_H

#define HIMAX_TS_NAME "hx8527_ts"

struct himax_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
	int (*_pfCreateVirKey)(void);
};

#endif /* _LINUX_HIMAX_TS_H */
