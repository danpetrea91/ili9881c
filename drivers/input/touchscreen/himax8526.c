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

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>
#include <linux/of_gpio.h>
#include "himax8526.h"
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif


/******************** Himax: Function Define ********************/

#define HX_TP_SYS_FS
#define HX_85XX_A_SERIES_PWON


/******************** Himax: Function Define ********************/

/******************** Himax: define and initial setting********************/
/********** 1. About fundamental define and setting **********/
#define HX_TP_MAX_FINGER             5
#define HX_TOUCH_INFO_VKEY           22
#define HX_TOUCH_INFO_ID_1_INFO      53
#define HX_TOUCH_INFO_ID_2_INFO      54
#define HX_TOUCH_INFO_POINT_CNT      28
#define DEFAUULT_X_RES               720
#define DEFAUULT_Y_RES               1280
#define HX_TP_X_CH_NUM               11
#define HX_TP_Y_CH_NUM               21
#define HX_KEY_COUNT                 4

/*
* #define HX_VKEY_0   101
* #define HX_VKEY_1   102
* #define HX_VKEY_2   103
* #define HX_VKEY_3   104
* #define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}
*/

#define HX_CMD_NOP                   0x00   /* no operation */
#define HX_CMD_SETMICROOFF           0x35   /* set micro on */
#define HX_CMD_SETROMRDY             0x36   /* set flash ready */
#define HX_CMD_TSSLPIN               0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT              0x81   /* set sleep out */
#define HX_CMD_TSSOFF                0x82   /* sense off */
#define HX_CMD_TSSON                 0x83   /* sense on */
#define HX_CMD_ROE                   0x85   /* read one event */
#define HX_CMD_RAE                   0x86   /* read all events */
#define HX_CMD_RLE                   0x87   /* read latest event */
#define HX_CMD_CLRES                 0x88   /* clear event stack */
#define HX_CMD_TSSWRESET             0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD   /* set cache function */
#define HX_CMD_SETIDLE               0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3   /* set idle delay */
#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F
#define HX_CMD_B9                    0xB9
#define HX_CMD_76                    0x76

#define DEFAULT_RETRY_CNT            3
#define HIMAX_INFO_MAX_LEN	512
#define DEBUG_DIR_NAME		"ts_debug"

#define STORE_TS_INFO(buf, name, max_touches, fw_vkey_support) \
	snprintf(buf, HIMAX_INFO_MAX_LEN, \
			"controller     = himax.\n" \
			"chip name      = %s\n" \
			"max_touches    = %d\n" \
			"driver_ver     = N/A\n" \
			"fw_ver         = V03\n" \
			"fw_vkey_support= %s\n", \
			name, max_touches, fw_vkey_support)

#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1
#define USB_SHIFT 0
#define AC_SHIFT 1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)


static int tpd_keys_local[3];
char button_value[3] = {KEY_MENU, KEY_HOMEPAGE, KEY_BACK};

/***********Himax Test Define*********/

int is_tp_driver_loaded;
static int himax_ts_resume(struct i2c_client *client);
static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg);

#ifdef HX_TP_FW_UPDATE
static struct task_struct		*update_firmware_tsk;
#endif

struct himax_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int intr_gpio;

	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;

	struct miscdevice firmware;
	struct attribute_group attrs;
	int status;
	struct switch_dev touch_sdev;
	int abs_x_max;
	int abs_y_max;
	int rst_gpio;
	struct wake_lock wakelock;
	int i2c_pull_up;
	int regulator_en;
	int virtual_key;
	const char *name;
	u32 num_max_touches;
	bool enable;
	char *ts_info;
	bool suspended;
	struct dentry *dir;
	uint16_t addr;
#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif
};

static struct himax_ts_data *private_ts;
static struct semaphore pSem;

struct i2c_client *touch_i2c;
static int himax_debug_flag = 1;
/********** 1. About fundamental define and setting **********/

/********** 2. About Firmware update define and setting **********/

#define FW_VER_MAJ_FLASH_ADDR	33
#define FW_VER_MAJ_FLASH_LENG	1

#define FW_VER_MIN_FLASH_LENG	1

#define CFG_VER_MAJ_FLASH_ADDR	39
#define CFG_VER_MAJ_FLASH_LENG	4
#define CFG_VER_MIN_FLASH_ADDR	43
#define CFG_VER_MIN_FLASH_LENG	3

#ifdef HX_TP_SYS_FS
static unsigned char FW_VER_MAJ_FLASH_buff1[FW_VER_MAJ_FLASH_LENG * 4];
static unsigned char FW_VER_MIN_FLASH_buff1[FW_VER_MIN_FLASH_LENG * 4];
#endif

static int hx_point_num;
static int p_point_num = 0xFFFF;
static int tpd_key;
static int tpd_key_old = 0xFF;

#ifdef Gesture_Detemination
static int Dist_Cal_EX = 0xFFFF;
static int Dist_Cal_Now = 0xFFFF;
static int ZoomInCnt;
static int ZoomOutCnt;
static int ZoomInFlag;
static int ZoomOutFlag;

#endif

#ifdef HX_TP_FW_UPDATE
static unsigned char upgrade_fw[32*1024];
static unsigned char isTP_Updated;

static unsigned char CTPM_FW[] = {
#include "qrc8x302_CheckSumAdd.i"
};
#endif


unsigned char SFR_3u_1[16][2] = { {0x18, 0x06}, {0x18, 0x16}, {0x18, 0x26},
				{0x18, 0x36}, {0x18, 0x46}, {0x18, 0x56},
				{0x18, 0x66}, {0x18, 0x76}, {0x18, 0x86},
				{0x18, 0x96}, {0x18, 0xA6}, {0x18, 0xB6},
				{0x18, 0xC6}, {0x18, 0xD6}, {0x18, 0xE6},
				{0x18, 0xF6} };

unsigned char SFR_6u_1[16][2] = { {0x98, 0x04}, {0x98, 0x14}, {0x98, 0x24},
				{0x98, 0x34}, {0x98, 0x44}, {0x98, 0x54},
				{0x98, 0x64}, {0x98, 0x74}, {0x98, 0x84},
				{0x98, 0x94}, {0x98, 0xA4}, {0x98, 0xB4},
				{0x98, 0xC4}, {0x98, 0xD4}, {0x98, 0xE4},
				{0x98, 0xF4} };
/********** 2. About Firmware update define and setting **********/



#define ABS_MT_POSITION         0x2a    /* Group a set of X and Y */
#define ABS_MT_AMPLITUDE        0x2b    /* Group a set of Z and W */

/********** 3. About Debug tool define and setting **********/

#define DEFAULT_X_CHANNEL            11   /* face the TS, x-axis */
#define DEFAULT_Y_CHANNEL            21   /* face the TS, y-axis */
#define DEFAULT_SELF_CHANNEL         32

#ifdef HX_TP_SYS_FS
static uint8_t *getMutualBuffer(void);
static void setMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDebugLevel(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);
static int himax_touch_sysfs_init(void);
static void himax_touch_sysfs_deinit(void);

static struct kobject *android_touch_kobj;
static uint8_t himax_command;
static uint8_t debug_log_level;
/* x asix, when you face the top of TS */
static uint8_t x_channel = DEFAULT_X_CHANNEL;
/* y asix, when you face the top of TS  */
static uint8_t y_channel = DEFAULT_Y_CHANNEL;
static uint8_t *diag_mutual;
static uint8_t diag_command;
static uint8_t diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL] = {0};
#endif
/********** 3. About Debug tool define and setting **********/

/******************** Himax: I2C R/W function ********************/
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data,
			uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		usleep_range(10000, 10100);
	}
	if (retry == toRetry) {
		printk(KERN_INFO "[TP] %s: i2c_read_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data,
			uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t)*(length+1), GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		usleep_range(10000, 10100);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[TP] %s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command,
				uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data,
				uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t)*length, GFP_KERNEL);

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		usleep_range(10000, 10100);
	}

	if (retry == toRetry) {
		printk(KERN_ERR "[TP] %s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length,
			uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		usleep_range(10000, 10100);
	}
	if (retry == toRetry) {
		printk(KERN_INFO "[TP] %s: i2c_read_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;
}
/******************** Himax: I2C R/W function ********************/

/******************** Himax: Firmware update function ********************/
#ifdef HX_TP_FW_UPDATE
int himax_ManualMode(int enter)
{
	unsigned char cmd[2];
	cmd[0] = enter;

	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x42, 1, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

int himax_FlashMode(int enter)
{
	unsigned char cmd[2];
	cmd[0] = enter;

	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 1, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

void himax_HW_reset(int RST)
{

	gpio_set_value_cansleep(RST, 0);
	msleep(20);


	gpio_set_value_cansleep(RST, 1);
	msleep(50);
}


int himax_lock_flash(void)
{
	unsigned char cmd[5];

	/* lock sequence start */
	cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x06;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03; cmd[1] = 0x00; cmd[2] = 0x00;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00; cmd[1] = 0x00; cmd[2] = 0x7D; cmd[3] = 0x03;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);
	return 0;
	/* lock sequence stop */
}

int himax_unlock_flash(void)
{
	unsigned char cmd[5];

	/* unlock sequence start */
	cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x06;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03; cmd[1] = 0x00; cmd[2] = 0x00;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00; cmd[1] = 0x00; cmd[2] = 0x3D; cmd[3] = 0x03;
	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0]) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}
	msleep(50);

	return 0;
	/* unlock sequence stop */
}

int himax_modifyIref(void)
{

	unsigned char i;
	unsigned char cmd[5];
	unsigned char Iref[2] = {0x00, 0x00};

	cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x08;
	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00; cmd[1] = 0x00; cmd[2] = 0x00;
	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if ((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	usleep_range(5000, 5100);
	for (i = 0; i < 16; i++) {
		if (cmd[1] == SFR_3u_1[i][0] && cmd[2] == SFR_3u_1[i][1]) {
			Iref[0] = SFR_6u_1[i][0];
			Iref[1] = SFR_6u_1[i][1];
		}
	}

	cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x06;
	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00; cmd[1] = 0x00; cmd[2] = 0x00;
	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = Iref[0]; cmd[1] = Iref[1]; cmd[2] = 0x27; cmd[3] = 0x27;
	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0])) < 0) {
		printk(KERN_ERR "[TP] %s: i2c access fail!\n", __func__);
		return 0;
	}

	return 1;
}

/*
 **  in longcheer case , before FW upgrade ,
 **  it read the minor cfg version(0XAB, in D32 case)
 **  if new fw has  different from  current fw, return 1
 */
#define LC_FW_VER  (0xAB)
/*
u8 himax_read_LC_FW_ver(void);
*/

u8 himax_read_LC_FW_ver(void)
{
	const u16 FLASH_VER_START_ADDR = CFG_VER_MIN_FLASH_ADDR;
	u16 i;
	unsigned char cmd[16];

	if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0])) < 0)
		return 0;
	msleep(120);

	himax_FlashMode(1);

	for (i = FLASH_VER_START_ADDR; i < FLASH_VER_START_ADDR + 1; i++) {
		cmd[0] = i & 0x1F;
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3,
			&cmd[0])) < 0)
			return 0;
		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0,
			&cmd[0])) < 0)
			return 0;
		if ((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4,
			&cmd[0])) < 0)
			return 0;

		if (cmd[3] != CTPM_FW[LC_FW_VER])
			return 1;
		else
			return 0;
	}

	himax_FlashMode(0);

	return 0;
}


static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)
{
	u16 checksum = 0;
	unsigned char cmd[5], last_byte;
	int FileLength, i, readLen, k, lastLength;

	FileLength = fullLength - 2;
	memset(cmd, 0x00, sizeof(cmd));


	if (himax_modifyIref() == 0)
		return 0;

	himax_FlashMode(1);

	FileLength = (FileLength + 3) / 4;
	for (i = 0; i < FileLength; i++) {
		last_byte = 0;
		readLen = 0;

		cmd[0] = i & 0x1F;
		if (cmd[0] == 0x1F || i == FileLength - 1)
			last_byte = 1;
		cmd[1] = (i >> 5) & 0x1F; cmd[2] = (i >> 10) & 0x1F;
		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3,
			&cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0,
			&cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		if ((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4,
			&cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		if (i < (FileLength - 1)) {
			checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
			if (i == 0)
				printk(KERN_ERR
	"[TP] %s: himax_marked cmd 0 to 3 : 0x%x, 0x%x, 0x%x, 0x%x\n",
		__func__, cmd[0], cmd[1], cmd[2], cmd[3]);
		} else {
			printk(KERN_ERR
	"[TP] %s: himax_marked cmd 0 to 3 : 0x%x, 0x%x, 0x%x, 0x%x\n",
		__func__, cmd[0], cmd[1], cmd[2], cmd[3]);
			printk(KERN_ERR
	"[TP] %s: himax_marked, checksum (not last): %d\n",
		__func__, checksum);

			lastLength = (((fullLength - 2) % 4) > 0) ?
				((fullLength - 2) % 4) : 4;

			for (k = 0; k < lastLength; k++)
				checksum += cmd[k];

			printk(KERN_ERR
		"[TP] %s: himax_marked, checksum (final): %d\n",
		__func__, checksum);


			if (ImageBuffer[fullLength - 1] ==
				(u8)(0xFF & (checksum >> 8)) &&
				ImageBuffer[fullLength - 2] ==
				(u8)(0xFF & checksum)) {
				himax_FlashMode(0);
				printk(KERN_ERR
				"__ding__ Check Success, return 1\n");
				return 1;
			} else {
				himax_FlashMode(0);
				return 0;
			}
		}
	}
	return 0;
}


int fts_hx_ctpm_fw_upgrade_with_i_file(void)
{
	unsigned char *ImageBuffer = CTPM_FW;
	int fullFileLength = sizeof(CTPM_FW);

	int i, j;
	unsigned char cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;


	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength - 2;
		himax_HW_reset(52);

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0,
			&cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		msleep(120);

		himax_unlock_flash();

		cmd[0] = 0x05; cmd[1] = 0x00; cmd[2] = 0x02;
		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
			3, &cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4F,
			0, &cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}
		msleep(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR "[TP] %s: i2c access fail!\n",
					__func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];

				cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x0D; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45,
				4, &cmd[0])) < 0) {
				printk(KERN_ERR "[TP] %s: i2c access fail!\n",
					__func__);
				return 0;
			}

			cmd[0] = 0x01; cmd[1] = 0x0D; cmd[2] = 0x02;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR "[TP] %s: i2c access fail!\n",
					__func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01; cmd[1] = 0x01; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x05; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x01; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				usleep_range(10000, 10100);
				if (i == (FileLength - 1)) {
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult =
			himax_calculateChecksum(ImageBuffer, fullFileLength);
					printk(KERN_ERR
			"__ding__ checksumResult = %d\n", checksumResult);


					himax_lock_flash();

					if (checksumResult) {
						return 1;
					} else if (!checksumResult) {
						return 0;
					} else {
						himax_FlashMode(0);
						himax_ManualMode(0);
					}
				}
			}
		}
	}
	return 0;
}

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
	unsigned char *ImageBuffer = fw;
	int fullFileLength = len;
	int i, j;
	unsigned char cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;


	for (j = 0; j < 3; j++) {
		FileLength = fullFileLength - 2;

		himax_HW_reset(52);

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81,
			0, &cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}

		msleep(120);

		himax_unlock_flash();

		cmd[0] = 0x05; cmd[1] = 0x00; cmd[2] = 0x02;
		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
			3, &cmd[0])) < 0) {
			printk(KERN_ERR
		"[TP] %s: i2c access fail!\n", __func__);
			return 0;
		}

		if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4F, 0,
			&cmd[0])) < 0) {
			printk(KERN_ERR "[TP] %s: i2c access fail!\n",
				__func__);
			return 0;
		}
		msleep(50);

		himax_ManualMode(1);
		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0, prePage = 0; i < FileLength; i++) {
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (prePage != cmd[1] || i == 0) {
				prePage = cmd[1];
				cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x0D; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}
			}

			memcpy(&cmd[0], &ImageBuffer[4*i], 4);
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45,
				4, &cmd[0])) < 0) {
				printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01; cmd[1] = 0x0D; cmd[2] = 0x02;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			cmd[0] = 0x01; cmd[1] = 0x09; cmd[2] = 0x02;
			if ((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43,
				3, &cmd[0])) < 0) {
				printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
				return 0;
			}

			if (last_byte == 1) {
				cmd[0] = 0x01; cmd[1] = 0x01; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x05; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x01; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01; cmd[1] = 0x00; cmd[2] = 0x02;
				if ((i2c_smbus_write_i2c_block_data(touch_i2c,
					0x43, 3, &cmd[0])) < 0) {
					printk(KERN_ERR
			"[TP] %s: i2c access fail!\n", __func__);
					return 0;
				}

				usleep_range(10000, 10100);
				if (i == (FileLength - 1)) {
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult =
			himax_calculateChecksum(ImageBuffer, fullFileLength);

					himax_lock_flash();

					if (checksumResult) {

						return 1;
					} else if (!checksumResult) {

						return 0;
					} else {
						himax_FlashMode(0);
						himax_ManualMode(0);
					}
				}
			}
		}
	}
	return 0;
}
#endif
/******************** Himax: Firmware update function ********************/

/******************** Himax: Debug Tool function ********************/
#ifdef HX_TP_SYS_FS
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}

static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t),
				GFP_KERNEL);
}

static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}

static uint8_t getDebugLevel(void)
{
	return debug_log_level;
}

static uint8_t getDiagCommand(void)
{
	return diag_command;
}

static uint8_t getXChannel(void)
{
	return x_channel;
}

static uint8_t getYChannel(void)
{
	return y_channel;
}

static void setXChannel(uint8_t x)
{
	x_channel = x;
}

static void setYChannel(uint8_t y)
{
	y_channel = y;
}

static ssize_t himax_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t data[96] = { 0 }, loop_i;

	printk(KERN_INFO "[TP] %s: himax_command = %x\n", __func__,
		himax_command);

	if (i2c_smbus_read_i2c_block_data(touch_i2c, himax_command, 96,
		&data[0]) < 0) {
		printk(KERN_WARNING "[TP] %s: read fail\n", __func__);
		return ret;
	}

	ret += snprintf(buf, 15, "command: %x\n", himax_command);
	for (loop_i = 0; loop_i < 96; loop_i++) {
		ret += snprintf(buf + ret, 8, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += snprintf(buf + ret, 4, "\n");
	}
	ret += snprintf(buf + ret, 4, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		if (buf[2] == 'x') {
			uint8_t loop_i;
			uint16_t base = 5;

			memcpy(buf_tmp, buf + 3, 2);
			if (!kstrtoul(buf_tmp, 16, &result))
				himax_command = result;
			for (loop_i = 0; loop_i < 100; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w')
						i2c_smbus_write_i2c_block_data(
			touch_i2c, himax_command, length, &write_da[0]);
					printk(KERN_INFO "CMD: %x, %x, %d\n",
				himax_command, write_da[0], length);
					for (veriLen = 0; veriLen < length;
						veriLen++)
						printk(KERN_INFO "%x ",
				*((&write_da[0])+veriLen));

					printk(KERN_INFO "\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!kstrtoul(buf_tmp, 16,
						&result))
						write_da[loop_i] = result;
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, 0644,
		himax_register_show, himax_register_store);


static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	snprintf(buf, 16, "%s_%s\n", FW_VER_MAJ_FLASH_buff1,
		FW_VER_MIN_FLASH_buff1);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static ssize_t himax_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += snprintf(buf, 8, "%d\n", debug_log_level);

	return count;
}



static ssize_t himax_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef HX_TP_FW_UPDATE
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
#endif

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		debug_log_level = buf[0] - '0';

#ifdef HX_TP_FW_UPDATE
/*If you want to update FW, you should discommented this block
*	if (buf[0] == 't') {
*		memset(fileName, 0, 128);
*
*		snprintf(fileName, count-2, "%s", &buf[2]);
*		printk(KERN_INFO "[TP] %s: upgrade from file(%s) start!\n",
*			__func__, fileName);
*
*		filp = filp_open(fileName, O_RDONLY, 0);
*		if (IS_ERR(filp)) {
*			printk(KERN_ERR "[TP] %s: open firmware file failed\n",
*				__func__);
*			return count;
*		}
*		oldfs = get_fs();
*		set_fs(get_ds());
*
*
*		result = filp->f_op->read(filp, upgrade_fw,
*				sizeof(upgrade_fw), &filp->f_pos);
*		if (result < 0) {
*			printk(KERN_ERR "[TP] %s: read firmware file failed\n",
*				__func__);
*			return count;
*		}
*
*		set_fs(oldfs);
*		filp_close(filp, NULL);
*
*		printk(KERN_INFO
*	"[TP] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__,
*	result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
*
*		if (result > 0) {
*			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw,
*				result) == 0)
*				printk(KERN_INFO
*	"[TP] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
*			else
*				printk(KERN_INFO
*	"[TP] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
*			return count;
*		}
*	}
*
*	if (buf[0] == 'f') {
*		printk(KERN_INFO
*	"[TP] %s: upgrade firmware from kernel image start!\n", __func__);
*		if (1) {
*			printk(KERN_INFO
*	"himax touch isTP_Updated: %d\n", isTP_Updated);
*			if (1) {
*				printk("himax touch firmware upgrade: %d\n",
*					isTP_Updated);
*				if (fts_hx_ctpm_fw_upgrade_with_i_file() == 0)
*					printk(KERN_INFO
*	"himax_marked TP upgrade error, line: %d\n", __LINE__);
*				else
*					printk(KERN_INFO
*	"himax_marked TP upgrade OK, line: %d\n", __LINE__);
*				isTP_Updated = 1;
*			}
*		}
*	}
*/
#endif
	return count;
}

static DEVICE_ATTR(debug_level, 0644,
		himax_debug_level_show, himax_debug_level_dump);


static ssize_t himax_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;
	width = x_channel;
	count += snprintf(buf + count, 25, "Channel: %4d, %4d\n\n",
			x_channel, y_channel);


	if (diag_command >= 1 && diag_command <= 6) {
		if (diag_command < 3) {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += snprintf(buf + count, 8, "%4d",
						diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1)) {
					count += snprintf(buf + count, 8,
		" %3d\n", diag_self[width + loop_i/width]);
				}
			}
			count += snprintf(buf + count, 4, "\n");
			for (loop_i = 0; loop_i < width; loop_i++) {
				count += snprintf(buf + count, 8, "%4d",
						diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
					count += snprintf(buf + count, 4, "\n");
			}
		} else if (diag_command > 4) {
			for (loop_i = 0; loop_i < self_num; loop_i++) {
				count += snprintf(buf + count, 8, "%4d",
						diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) ==
					(width - 1))
					count += snprintf(buf + count, 4, "\n");
			}
		} else {
			for (loop_i = 0; loop_i < mutual_num; loop_i++) {
				count += snprintf(buf + count, 8, "%4d",
				diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
					count += snprintf(buf + count, 4, "\n");
			}
		}
	}

	return count;
}


static ssize_t himax_diag_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0xFC;
	const uint8_t command_ec_128_raw_baseline_flag = 0x02
		| command_ec_128_raw_flag;
	uint8_t new_command[2] = {0x91, 0x00};

	if (buf[0] == '1' || buf[0] == '3' || buf[0] == '5') {

		new_command[1] = command_ec_128_raw_baseline_flag;
		i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1,
				&new_command[1]);
		diag_command = buf[0] - '0';
	} else if (buf[0] == '2' || buf[0] == '4' || buf[0] == '6') {

		new_command[1] = command_ec_128_raw_flag;
		i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1,
			&new_command[1]);
		diag_command = buf[0] - '0';
	} else {
		new_command[1] = command_ec_24_normal_flag;
		i2c_smbus_write_i2c_block_data(touch_i2c, new_command[0], 1,
			&new_command[1]);
		diag_command = 0;
	}
	return count;
}

static DEVICE_ATTR(diag, 0644,
		himax_diag_show, himax_diag_dump);

static struct attribute *himax_attr[] = {
	&dev_attr_register.attr,
	&dev_attr_vendor.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_diag.attr,
	NULL
};

static ssize_t hx8526_ts_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);

	return snprintf(buf, HIMAX_INFO_MAX_LEN, "%s\n", data->ts_info);
}

static DEVICE_ATTR(ts_info, 0444, hx8526_ts_info_show, NULL);

static ssize_t hx8526_mt_protocol_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, 16, "%s\n", "MT Protocal B");
}

static DEVICE_ATTR(mt_protocol_type, 0444, hx8526_mt_protocol_type_show,
			NULL);

static ssize_t hx8526_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (val) {
		data->enable = true;
		himax_ts_resume(data->client);
	} else {
		data->enable = false;
		himax_ts_suspend(data->client, PMSG_SUSPEND);
	}

	return size;
}

static ssize_t hx8526_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *data = dev_get_drvdata(dev);

	if (data->suspended) {
		dev_info(&data->client->dev, "Already in suspend state\n");
		return snprintf(buf, 4, "%s\n", "0");
	}

	return snprintf(buf, 4, "%s\n", data->enable ? "1" : "0");
}

static DEVICE_ATTR(enable, 0664, hx8526_enable_show, hx8526_enable_store);

static ssize_t hx8526_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return snprintf(buf, 8, "%s\n", "N/A\n");

	return 0;
}

static ssize_t hx8526_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(update_fw, 0664, hx8526_update_fw_show,
		hx8526_update_fw_store);

static ssize_t hx8526_force_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, hx8526_update_fw_show,
		hx8526_force_update_fw_store);

static ssize_t hx8526_fw_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 8, "%s\n", "N/A\n");
}

static ssize_t hx8526_fw_name_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(fw_name, 0664, hx8526_fw_name_show,
			hx8526_fw_name_store);


static int debug_dump_info(struct seq_file *m, void *v)
{
	struct himax_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner = THIS_MODULE,
	.open = debugfs_dump_info_open,
	.read = seq_read,
	.release = single_release,
};

static int debug_suspend_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		himax_ts_suspend(data->client, PMSG_SUSPEND);
	else
		himax_ts_resume(data->client);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int debug_suspend_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, debug_suspend_get,
		debug_suspend_set, "%lld\n");

static bool debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("hx8526 reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int debug_data_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
				"Writing into registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int debug_data_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;
	u8 reg = 0;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr)) {
		i2c_himax_read(data->client, data->addr, &reg, 1, 1);
		*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, debug_data_get,
		debug_data_set, "0x%02llX\n");

static int debug_addr_set(void *_data, u64 val)
{
	struct himax_ts_data *data = _data;

	if (debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int debug_addr_get(void *_data, u64 *val)
{
	struct himax_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, debug_addr_get,
		debug_addr_set, "0x%02llX\n");


static int himax_touch_sysfs_init(void)
{
	int ret;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[TP]TOUCH_ERR: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR
	"[TP]TOUCH_ERR: create_file debug_level failed\n");
		return ret;
	}
	himax_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: create_file register failed\n");
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret) {
		printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
		return ret;
	}

	return 0;
}

static void himax_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);

	kobject_del(android_touch_kobj);
}
#endif
/******************** Himax: Debug Tool function ********************/

/******************** Himax: Fundamental function ********************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

static void himax_ts_work_func(struct work_struct *work)
{
	int temx = 0;
	int temy = 0;
#ifdef HX_TP_SYS_FS
	uint8_t *mutual_data;
	uint8_t *self_data;
	int mul_num, self_num;
	int index = 0;
	uint8_t diag_cmd;
#endif
	int ret, i;
	int RawDataLen = 0;
	unsigned int x = 0, y = 0, area = 0, press = 0;
	const unsigned int x_res = DEFAUULT_X_RES;
	const unsigned int y_res = DEFAUULT_Y_RES+100;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
			work);
	unsigned char check_sum_cal = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[128] = {0};
	static int old_status[5] = {0};

	int raw_cnt_max = HX_TP_MAX_FINGER/4;
	int raw_cnt_rmd = HX_TP_MAX_FINGER%4;

	int hx_touch_info_size;

	if (raw_cnt_rmd != 0x00) {
		RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+3)*4) - 1;
/*
*		RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+3)*4);
*/
		hx_touch_info_size = (HX_TP_MAX_FINGER+raw_cnt_max+2)*4;
	} else {
		RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+2)*4) - 1;
/*
*		RawDataLen = 128 - ((HX_TP_MAX_FINGER+raw_cnt_max+2)*4);
*/
		hx_touch_info_size = (HX_TP_MAX_FINGER+raw_cnt_max+1)*4;
	}

	start_reg = HX_CMD_RAE;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
#ifdef HX_TP_SYS_FS
	msg[1].len = hx_touch_info_size + RawDataLen + 4 + 1;
/*
	msg[1].len = hx_touch_info_size + RawDataLen + 4;
*/
#else
	msg[1].len = hx_touch_info_size;
#endif
	msg[1].buf = buf;

#ifdef HX_ISR_ENTER_CHECK
	printk(KERN_ERR "******Himax Touch Controller Trigger ISR test\n");
#endif



	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0)
		memset(buf, 0xff, 128);

	for (i = 0; i < hx_touch_info_size; i++)
		check_sum_cal += buf[i];


	if ((check_sum_cal != 0x00) || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0)
		!= 0xF0) {
		printk(KERN_INFO
	"check_sum_cal: 0x%02X, buf[POINT_CNT] = 0x%02X, POINT_CNT = %d\n",
	check_sum_cal, buf[HX_TOUCH_INFO_POINT_CNT], HX_TOUCH_INFO_POINT_CNT);
		enable_irq(ts->client->irq);
		ts->suspended = false;
		return;
	}

#ifdef HX_TP_SYS_FS

	if (getDebugLevel() & 0x1) {
		printk(KERN_INFO "[TP]%s: raw data:\n", __func__);
		for (i = 0; i < 128; i = i + 8) {
			printk(KERN_INFO
	"%d:0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X,0x%2.2X\n",
	i, buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6],
	buf[i+7]);
		}
	}

	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) {
		printk(KERN_INFO "[TP]%s: Diag Raw Data:\n", __func__);
		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();

		mul_num = getXChannel() * getYChannel();
		self_num = getXChannel() + getYChannel();

		if (buf[hx_touch_info_size] == buf[hx_touch_info_size+1] &&
			buf[hx_touch_info_size+1] == buf[hx_touch_info_size+2]
			&& buf[hx_touch_info_size+2] ==
			buf[hx_touch_info_size+3] &&
			buf[hx_touch_info_size] > 0) {
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;

			for (i = 0; i < RawDataLen; i++) {
				if ((index+i) < mul_num) {
					mutual_data[index + i] = buf[i +
			hx_touch_info_size + 4];
				} else {
					if ((i+index) >= (self_num+mul_num))
						break;
					self_data[i+index-mul_num] = buf[i +
						hx_touch_info_size+4];
				}
			}
		} else {
			printk(KERN_INFO "[TP]%s: header format is wrong!\n",
				__func__);
		}
	}
#endif


	tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT + 2] >> 4);
	if (tpd_key == 0x0F)
		tpd_key = 0xFF;

	p_point_num = hx_point_num;

	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
		hx_point_num = 0;
	else
		hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

	for (i = 0; i < HX_TP_MAX_FINGER; i++) {
		input_mt_slot(ts->input_dev, i);
				y = buf[4 * i + 1] | (buf[4 * i] << 8);
				x = buf[4 * i + 3] | (buf[4 * i + 2] << 8);

				temx = x;
				temy = y;
				x = temy;
				y = temx;
		if (hx_point_num != 0 && buf[4*i] != 0xFF && x <= x_res &&
			y <= y_res) {

					press = buf[4*HX_TP_MAX_FINGER+i];
					area = press;
					if (area > 31)
						area = (area >> 3);

				input_mt_report_slot_state(
						ts->input_dev, MT_TOOL_FINGER, 1);
				input_report_abs(ts->input_dev,
									ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev,
									ABS_MT_POSITION_Y, y);
				input_report_abs(ts->input_dev, ABS_X, x);
				input_report_abs(ts->input_dev, ABS_Y, y);
				/*input_report_key(ts->input_dev, BTN_TOUCH, 1);*/
				old_status[i] = 1;
		} else {
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, 0);
		}
	}
		input_sync(ts->input_dev);

	if (hx_point_num == 0 && tpd_key != 0xFF) {
		if (tpd_key == 1)
			input_report_key(ts->input_dev, KEY_MENU, 1);
		if (tpd_key == 2)
			input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);
		if (tpd_key == 3)
			input_report_key(ts->input_dev, KEY_BACK, 1);
		/*if (tpd_key == 4)
				input_report_key(ts->input_dev, BTN_TOUCH, 0);*/
		input_sync(ts->input_dev);
	}
	if (hx_point_num == 0 && tpd_key == 0xFF) {
		if (tpd_key_old != 0xFF)
				input_report_key(ts->input_dev, 
						tpd_keys_local[tpd_key_old - 1], 0);
		input_sync(ts->input_dev);
	}

#ifdef Gesture_Detemination

	if (hx_point_num == 2 && p_point_num == 2 && temp_x[0] != 0xFFFF &&
		temp_y[0] != 0xFFFF && temp_x[1] != 0xFFFF &&
		temp_y[1] != 0xFFFF) {
		if (temp_x[0] > temp_x[1] && temp_y[0] > temp_y[1]) {
			Dist_Cal_Now = (temp_x[0] - temp_x[1]) +
				(temp_y[0] - temp_y[1]);
		} else if (temp_x[0] > temp_x[1] && temp_y[1] > temp_y[0]) {
			Dist_Cal_Now = (temp_x[0] - temp_x[1]) +
				(temp_y[1] - temp_y[0]);
		} else if (temp_x[1] > temp_x[0] && temp_y[0] > temp_y[1]) {
			Dist_Cal_Now = (temp_x[1] - temp_x[0]) +
				(temp_y[0] - temp_y[1]);
		} else if (temp_x[1] > temp_x[0] && temp_y[1] > temp_y[0]) {
			Dist_Cal_Now = (temp_x[1] - temp_x[0]) +
				(temp_y[1] - temp_y[0]);
		}

		if (Dist_Cal_Now - Dist_Cal_EX > 10)
			ZoomInCnt++;
		else if (Dist_Cal_EX - Dist_Cal_Now > 10)
			ZoomOutCnt++;

		if (ZoomInCnt > 2) {
			ZoomInFlag = 1;
			ZoomOutFlag = 0;
		} else if (ZoomOutCnt > 2) {
			ZoomOutFlag = 1;
			ZoomInFlag = 0;
		}
		Dist_Cal_EX = Dist_Cal_Now;
	} else if (hx_point_num == 2 && p_point_num != 2 &&
		temp_x[0] != 0xFFFF && temp_y[0] != 0xFFFF
			&& temp_x[1] != 0xFFFF && temp_y[1] != 0xFFFF) {
		Dist_Cal_EX = 0;
	} else {
		Dist_Cal_EX = 0xFFFF;
		Dist_Cal_Now = 0xFFFF;
		ZoomInCnt = 0;
		ZoomOutCnt = 0;
		p_point_num = 0xFFFF;
	}

#endif
	tpd_key_old = tpd_key;

	enable_irq(ts->client->irq);
	ts->suspended = false;

	return;
}

static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "[himax] %s\n", __func__);
	disable_irq_nosync(ts->client->irq);
	queue_work(ts->himax_wq, &ts->work);

	return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	err = request_threaded_irq(client->irq, himax_ts_irq_handler,
			NULL, IRQF_TRIGGER_LOW, client->name, ts);
/*
*	err = request_irq(client->irq, himax_ts_irq_handler,
*			IRQF_TRIGGER_FALLING, client->name, ts);
*/

	if (err)
		dev_err(&client->dev, "[himax] %s: request_irq %d failed\n",
				__func__, client->irq);

	return err;
}

#ifdef HX_READ_FW_VERSION
static int himax_ts_sense_on(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[2];
	int ret;

	buf0[0] = HX_CMD_TSSON;

	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);
	return ret;
}
#endif

static int himax_ts_poweron(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[11];
	int ret = 0;
#ifdef HX_85XX_C_SERIES_PWON
	buf0[0] = HX_CMD_MANUALMODE;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SETMICROOFF;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SETROMRDY;
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SET_CACHE_FUN;
	buf0[1] = 0x05;
	buf0[2] = 0x03;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_B9;
	buf0[1] = 0x01;
	buf0[2] = 0x2D;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = 0xE3;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_TSSON;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);

	buf0[0] = HX_CMD_TSSLPOUT;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);
#else
#ifdef HX_85XX_A_SERIES_PWON
	buf0[0] = HX_CMD_MANUALMODE;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);
	buf0[0] = HX_CMD_SETROMRDY;
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SET_CACHE_FUN;
	buf0[1] = 0x08;
	buf0[2] = 0x03;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_CB;
	buf0[1] = 0x01;
	buf0[2] = 0xF5;
	buf0[3] = 0xFF;
	buf0[4] = 0xFF;
	buf0[5] = 0x01;
	buf0[6] = 0x00;
	buf0[7] = 0x05;
	buf0[8] = 0x00;
	buf0[9] = 0x05;
	buf0[10] = 0x00;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 11,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);


	buf0[0] = HX_CMD_EA;
	buf0[1] = 0x00;
	buf0[2] = 0x00;
	buf0[3] = 0x00;
	buf0[4] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 5,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_TSSON;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);

	buf0[0] = HX_CMD_TSSLPOUT;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);
#else
#ifdef HX_85XX_B_SERIES_PWON
	buf0[0] = HX_CMD_MANUALMODE;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SETMICROOFF;
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SETROMRDY;
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_SET_CACHE_FUN;
	buf0[1] = 0x06;
	buf0[2] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_76;
	buf0[1] = 0x01;
	buf0[2] = 0x2D;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	udelay(100);

	buf0[0] = HX_CMD_TSSON;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);

	buf0[0] = HX_CMD_TSSLPOUT;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts_modify->client->addr);
	}
	msleep(120);
#endif

#endif

#endif

	return ret;
}

#ifdef HX_READ_FW_VERSION
static int himax_i2c_read_fw_version(struct himax_ts_data *ts)
{
	uint8_t buf0[5];
	int ret = -1;
	struct i2c_msg msg[2];
	uint8_t start_reg;

	buf0[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(ts->client, buf0, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR "i2c_master_send failed addr = 0x%x\n",
			ts->client->addr);
	}

	msleep(120);



	start_reg = 0x32;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf0;


	ret = i2c_transfer(ts->client->adapter, msg, 2);

	if (ret < 0) {
		printk(KERN_ERR
		"*****HIMAX TEST: Read FW version failed addr = 0x%x\n",
		ts->client->addr);
	} else {
		ret = buf0[0];
		printk(KERN_ERR
		"*****HIMAX TEST: Read FW version OK addr = 0x%x\n",
		ts->client->addr);
		printk(KERN_ERR
		"*****HIMAX TEST: Read FW version OK Version = 0x%x\n", ret);
	}


	return ret;
}
#endif

#ifdef HX_CHECK_I2C_PATH
static int himax_i2c_test_function(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[5];
	int ret = 0;

	buf0[0] = 0xE9;
	buf0[1] = 0x01;
	buf0[2] = 0x01;

	while (1) {
		ret = i2c_himax_master_write(ts_modify->client, buf0, 3,
			DEFAULT_RETRY_CNT);
		if (ret < 0) {
			printk(KERN_ERR
		"*****HIMAX TEST: i2c_master_send failed addr = 0x%x\n",
		ts_modify->client->addr);
		} else {
			printk(KERN_ERR "*****HIMAX TEST: OK addr = 0x%x\n",
		ts_modify->client->addr);
		}

		msleep(200);
	}

	return ret;
}

#endif

void touch_callback(unsigned cable_status) {}

#ifdef HX_TP_FW_UPDATE
static int update_func(void *pointer)
{
	printk(KERN_ERR "__ding__ start update\n");
	if (fts_hx_ctpm_fw_upgrade_with_i_file() == 0)
		printk(KERN_ERR "TP upgrade error, line: %d\n", __LINE__);
	else
		printk(KERN_ERR "TP upgrade OK, line: %d\n", __LINE__);

	himax_HW_reset(52);
	msleep(50);

	return 0;
}
#endif

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts =
		container_of(self, struct himax_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			himax_ts_resume(ts->client);
		else if (*blank == FB_BLANK_POWERDOWN)
			himax_ts_suspend(ts->client, PMSG_SUSPEND);
	}

	return 0;
}
#endif

static struct regulator *vcc_i2c;
static struct regulator *vdd;

static int himax_regulator_configure(struct i2c_client *i2c_client)
{
	int retval;
	struct himax_ts_data *ts = (struct himax_ts_data *)
		i2c_get_clientdata(i2c_client);

	if (ts->regulator_en) {
			vdd = regulator_get(&i2c_client->dev, "vdd");
		if (IS_ERR(vdd)) {
			dev_err(&i2c_client->dev,
				"%s: Failed to get vdd regulator\n", __func__);
			return PTR_ERR(vdd);
		}

		if (regulator_count_voltages(vdd) > 0) {
			retval = regulator_set_voltage(vdd, 2700000, 3300000);
			if (retval) {
				dev_err(&i2c_client->dev,
			"regulator set_vtg failed retval = %d\n", retval);
				goto err_set_vtg_vdd;
			}
		}
	}

	if (ts->i2c_pull_up) {
		vcc_i2c = regulator_get(&i2c_client->dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			dev_err(&i2c_client->dev,
				"%s: Failed to get i2c regulator\n", __func__);
			retval = PTR_ERR(vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(vcc_i2c) > 0) {
			retval = regulator_set_voltage(vcc_i2c,
					1800000, 1800000);
			if (retval) {
				dev_err(&i2c_client->dev,
			"reg set i2c vtg failed retval = %d\n", retval);
				goto err_set_vtg_i2c;
			}
		}
	}
	return 0;

err_set_vtg_i2c:
	if (ts->i2c_pull_up)
		regulator_put(vcc_i2c);
err_get_vtg_i2c:
	if (ts->regulator_en)
		if (regulator_count_voltages(vdd) > 0)
			regulator_set_voltage(vdd, 0, 3300000);
err_set_vtg_vdd:
	if (ts->regulator_en)
		regulator_put(vdd);
	return retval;
};


static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
				regulator_set_optimum_mode(reg, load_uA) : 0;

}

static int himax_power_on(struct i2c_client *i2c_client)
{
	int retval;
	struct himax_ts_data *ts = (struct himax_ts_data *)
		i2c_get_clientdata(i2c_client);

	retval = himax_regulator_configure(i2c_client);
	if (retval)
		return retval;

	if (ts->regulator_en) {
		retval = reg_set_optimum_mode_check(vdd, 15000);
		if (retval < 0) {
			dev_err(&i2c_client->dev,
			"Regulator vdd set_opt failed rc = %d\n", retval);
			return retval;
		}

		retval = regulator_enable(vdd);
		if (retval) {
			dev_err(&i2c_client->dev,
			"Regulator vdd enable failed rc = %d\n", retval);
			goto error_reg_en_vdd;
		}
	}

	if (ts->i2c_pull_up) {
		retval = reg_set_optimum_mode_check(vcc_i2c, 10000);
		if (retval < 0) {
			dev_err(&i2c_client->dev,
			"Regulator vcc_i2c set_opt failed rc = %d\n", retval);
			goto error_reg_opt_i2c;
		}

		retval = regulator_enable(vcc_i2c);
		if (retval) {
			dev_err(&i2c_client->dev,
			"Regulator vcc_i2c enable failed rc = %d\n", retval);
			goto error_reg_en_vcc_i2c;
		}
	}

	return 0;
error_reg_en_vcc_i2c:
	if (ts->i2c_pull_up)
		reg_set_optimum_mode_check(vcc_i2c, 0);
error_reg_opt_i2c:
	if (ts->regulator_en)
		regulator_disable(vdd);
error_reg_en_vdd:
	if (ts->regulator_en)
		reg_set_optimum_mode_check(vdd, 0);
	return retval;
}

struct kobject *himax_virtual_key_properties_kobj;
static ssize_t
himax_virtual_keys_register(struct kobject *kobj,
					struct kobj_attribute *attr,
							char *buf)
{
	return snprintf(buf, 200,
	"%s:%s:110:1315:80:80:%s:%s:350:1315:80:80:%s:%s:590:1315:80:80\n",
	__stringify(EV_KEY), __stringify(KEY_MENU), __stringify(EV_KEY),
	__stringify(KEY_HOMEPAGE), __stringify(EV_KEY), __stringify(KEY_BACK));
/*
*	__stringify(EV_KEY)":"__stringify(KEY_MENU)":110:1315:80:80"":"
*	__stringify(EV_KEY)":"__stringify(KEY_HOMEPAGE)":350:1315:80:80"
*	":"__stringify(EV_KEY)":"__stringify(KEY_BACK)":590:1315:80:80""\n");
*/
}

static struct kobj_attribute himax_virtual_keys_attr = {
	   .attr = {
			   .name = "virtualkeys.himax8526",
			   .mode = S_IRUGO,
	   },
	   .show = &himax_virtual_keys_register,
};

static struct attribute *himax_virtual_key_properties_attrs[] = {
	   &himax_virtual_keys_attr.attr,
	   NULL,
};

static struct attribute_group himax_virtual_key_properties_attr_group = {
	.attrs = himax_virtual_key_properties_attrs,
};

static int himax_parse_dt(struct device *dev, struct himax_ts_data *ts)
{
	struct device_node *np = dev->of_node;
	u32 temp_val, rc;

	ts->name = "himax";
	rc = of_property_read_string(np, "himax,name", &ts->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
	rc = of_property_read_u32(np, "himax,num-max-touches", &temp_val);
	if (!rc)
		ts->num_max_touches = temp_val;
	else
		return rc;

	ts->i2c_pull_up = of_property_read_bool(np, "himax,i2c-pull-up");
	ts->regulator_en = of_property_read_bool(np, "himax,reg-en");
	ts->rst_gpio = of_get_named_gpio_flags(np,
			"himax,reset-gpio", 0, &temp_val);
	ts->intr_gpio = of_get_named_gpio_flags(np,
			"himax,irq-gpio", 0, &temp_val);
	ts->virtual_key = of_property_read_bool(np,
			"himax,virtual_key");

	if (ts->virtual_key) {
		himax_virtual_key_properties_kobj = kobject_create_and_add
												("board_properties", NULL);
		if (himax_virtual_key_properties_kobj) {
			rc = sysfs_create_group(
			himax_virtual_key_properties_kobj,
			&himax_virtual_key_properties_attr_group);
			if (rc)
				return rc;
		}
	}

	return 0;
}

int himax_ic_package_check(struct himax_ts_data *ts_modify)
{
	uint8_t cmd[3];
	uint8_t data[3];

	if (i2c_himax_read(ts_modify->client, 0xD1, cmd, 3,
		DEFAULT_RETRY_CNT) < 0)	{
		return -EIO;
		}

	if (i2c_himax_read(ts_modify->client, 0x31, data, 3,
		DEFAULT_RETRY_CNT) < 0) {
		return -EIO;
		}


	printk(KERN_INFO "himax device id : 0x%x\n", data[0]);
	return 0;
}

static int himax_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int i;
	int err = 0;
	struct dentry *temp;
#ifdef HX_TP_FW_UPDATE
	int isNeedupdate = 0;
#endif
	struct himax_ts_data *ts;

	is_tp_driver_loaded = 0;
	private_ts = NULL;
	touch_i2c = NULL;
	himax_debug_flag = 1;
	hx_point_num = 0;
	tpd_key = 0;
#ifdef Gesture_Detemination
	ZoomInCnt = 0;
	ZoomOutCnt = 0;
	ZoomInFlag = 0;
	ZoomOutFlag = 0;
#endif
#ifdef HX_TP_FW_UPDATE
	isTP_Updated = 0;
#endif
	android_touch_kobj = NULL;
	himax_command = 0;
	debug_log_level = 0;
	diag_mutual = NULL;
	diag_command = 0;

	printk(KERN_ERR "__ding__ himax_ts_probe\n");

	if (is_tp_driver_loaded) {
		printk(KERN_ERR	"himax_ts_probe other driver has been loaded\n");
		return -ENODEV;
	}


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR	"[himax] %s: i2c check functionality error\n",
																__func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR	"[himax] %s: allocate himax_ts_data failed\n",
																__func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->himax_wq = create_singlethread_workqueue("himax_wq");
	if (!ts->himax_wq) {
		printk(KERN_ERR "[himax] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}


	INIT_WORK(&ts->work, himax_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	himax_parse_dt(&client->dev, ts);
	err = himax_power_on(client);
	if (err) {
		printk(KERN_INFO "HIMAX ERR POWER ON\n");
		goto err_power_on;
	}

	err = gpio_request(ts->intr_gpio, "hx8526_irq_gpio");
	if (err) {	
		}

	err = gpio_request(ts->rst_gpio, "hx8526_reset_gpio");
    if (err) {
		}

	gpio_set_value_cansleep(ts->rst_gpio, 0);
	msleep(20);
	gpio_set_value_cansleep(ts->rst_gpio, 1);
	msleep(20);

	sema_init(&pSem, 1);

	ts->status = 1;

	err = himax_ic_package_check(ts);
	if (err) {
		printk(KERN_INFO "HIMAX ERR READ DEVICE ID\n");
		goto free_gpio;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"[himax] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}



/*	This includes the maximum x-y axix and the report information
*	such as area, pressure and x-y point.*/

	ts->input_dev->name = "himax8526";
	ts->input_dev->phys = "himax8526/input0";
	ts->abs_x_max = DEFAUULT_X_RES;
	ts->abs_y_max = DEFAUULT_Y_RES;
	dev_info(&client->dev, "[himax] Max X = %d, Max Y = %d\n",
			ts->abs_x_max, ts->abs_y_max);

	for (i = 0; i < 3; i++) {
		tpd_keys_local[i] = button_value[i];
		set_bit(tpd_keys_local[i], ts->input_dev->keybit);
	}

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(KEY_OK, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
				DEFAUULT_X_RES, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
				DEFAUULT_Y_RES, 0, 0);
	/*input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH); */

	input_set_abs_params(ts->input_dev, ABS_X, 0,
                                DEFAUULT_X_RES, 0, 0);
        input_set_abs_params(ts->input_dev, ABS_Y, 0,
                                DEFAUULT_Y_RES, 0, 0);
	input_mt_init_slots(ts->input_dev, HX_TP_MAX_FINGER, 0);

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
				"[himax]%s: unable to register %s input device\n",
				__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	touch_i2c = client;

#ifdef HX_CHECK_I2C_PATH
	himax_i2c_test_function(ts);
#endif

	himax_ts_poweron(ts);

#ifdef HX_READ_FW_VERSION
	himax_i2c_read_fw_version(ts);
	himax_ts_sense_on(ts);
#endif

#ifdef HX_TP_SYS_FS


	setXChannel(HX_TP_X_CH_NUM);
	setYChannel(HX_TP_Y_CH_NUM);


	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		printk(KERN_ERR
		"[TP] %s: mutual buffer allocate fail failed\n", __func__);
		return -EPERM;
	}


	himax_touch_sysfs_init();

	ts->attrs.attrs = himax_attr;
	err = sysfs_create_group(&client->dev.kobj, &ts->attrs);
	if (err) {
		dev_err(&client->dev,
		"[TP] %s: Not able to create the sysfs\n", __func__);
	}
#endif

	err = device_create_file(&client->dev, &dev_attr_ts_info);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto err_input_register_device_failed;
	}

	err = device_create_file(&client->dev, &dev_attr_mt_protocol_type);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_ts_info;
	}

	err = device_create_file(&client->dev, &dev_attr_enable);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_type;
	}

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_enable;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	ts->ts_info =
		devm_kzalloc(&client->dev, HIMAX_INFO_MAX_LEN, GFP_KERNEL);
	if (!ts->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_force_update_fw_sys;
	}
	ts->enable = true;

	STORE_TS_INFO(ts->ts_info, ts->name,
			ts->num_max_touches,
			ts->virtual_key ? "yes" : "no");

	ts->dir = debugfs_create_dir(DEBUG_DIR_NAME, NULL);
	if (ts->dir == NULL || IS_ERR(ts->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n",
				PTR_ERR(ts->dir));
		err = PTR_ERR(ts->dir);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("addr", S_IRUSR | S_IWUSR, ts->dir,
				ts, &debug_addr_fops);

	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("data", S_IRUSR | S_IWUSR, ts->dir,
				ts, &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, ts->dir,
			ts, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp =
		debugfs_create_file("dump_info", S_IRUSR | S_IWUSR,
				ts->dir,	ts, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	himax_ts_register_interrupt(ts->client);

	if (gpio_get_value(ts->intr_gpio) == 0) {
		printk(KERN_INFO
		"[himax]%s: handle missed interrupt\n", __func__);
		himax_ts_irq_handler(client->irq, ts);
	}





#ifdef HX_TP_FW_UPDATE
	update_firmware_tsk = kthread_create(update_func,
			update_firmware_tsk, "update_firmware");
	if (IS_ERR(update_firmware_tsk)) {
		kfree(update_firmware_tsk);
		return -EINVAL;
	}
	if (isNeedupdate)
		wake_up_process(update_firmware_tsk);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_FB
	ts->suspended = false;
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
				err);
#endif

	private_ts = ts;

	dev_info(&client->dev,
		"[himax] Start touchscreen %s in interrupt mode\n",
			ts->input_dev->name);
	is_tp_driver_loaded = 1;

	return 0;

free_debug_dir:
	debugfs_remove_recursive(ts->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
free_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
free_type:
	device_remove_file(&client->dev, &dev_attr_mt_protocol_type);
free_ts_info:
	device_remove_file(&client->dev, &dev_attr_ts_info);

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	if (ts->himax_wq)
		destroy_workqueue(ts->himax_wq);

err_create_wq_failed:
free_gpio:
	if (gpio_is_valid(ts->rst_gpio))
		gpio_free(ts->rst_gpio);
	if (gpio_is_valid(ts->intr_gpio))
		gpio_free(ts->intr_gpio);
err_power_on:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

	return err;
}

static int himax_ts_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

#ifdef HX_TP_SYS_FS
	himax_touch_sysfs_deinit();
#endif

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);

	if (ts->himax_wq)
		destroy_workqueue(ts->himax_wq);
	input_unregister_device(ts->input_dev);
	wake_lock_destroy(&ts->wakelock);

	kfree(ts);

	return 0;
}

static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct himax_ts_data *ts_modify = i2c_get_clientdata(client);
	uint8_t buf[2] = {0};
	int ret = 0;

	printk(KERN_INFO "[himax] %s: TS suspend\n", __func__);

	buf[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(ts_modify->client, buf, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
			"[himax] %s: I2C access failed addr = 0x%x\n",
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	buf[0] = HX_CMD_TSSLPIN;
	ret = i2c_himax_master_write(ts_modify->client, buf, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
			"[himax] %s: I2C access failed addr = 0x%x\n",
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	buf[0] = HX_CMD_SETDEEPSTB;
	buf[1] = 0x01;
	ret = i2c_himax_master_write(ts_modify->client, buf, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
			"[himax] %s: I2C access failed addr = 0x%x\n",
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	disable_irq(client->irq);
	ts_modify->suspended = true;

	ret = cancel_work_sync(&ts_modify->work);
	if (ret) {
		enable_irq(client->irq);
		ts_modify->suspended = false;
	}

	return 0;
}

static int himax_ts_resume(struct i2c_client *client)
{
	struct himax_ts_data *ts_modify = i2c_get_clientdata(client);
	uint8_t buf[2] = {0};
	int ret = 0;

	printk(KERN_INFO "[himax] %s: TS resume\n", __func__);

	buf[0] = HX_CMD_SETDEEPSTB;
	buf[1] = 0x00;
	ret = i2c_himax_master_write(ts_modify->client, buf, 1,
		DEFAULT_RETRY_CNT);
	if (ret < 0) {
		printk(KERN_ERR
			"[himax] %s: I2C access failed addr = 0x%x\n",
			__func__, ts_modify->client->addr);
	}
	udelay(100);

	himax_ts_poweron(ts_modify);

	enable_irq(client->irq);
	ts_modify->suspended = false;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;

	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;

	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id himax_ts_id[] = {
	{ HIMAX_TS_NAME, 0},
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id himax_match_table[] = {
	{ .compatible = "himax,hx8527-d48",},
	{ },
};
#else
#define himax_match_table NULL
#endif

static struct i2c_driver himax_ts_driver = {
	.probe		 = himax_ts_probe,
	.remove		 = himax_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	 = himax_ts_suspend,
	.resume		 = himax_ts_resume,
#endif
	.id_table   = himax_ts_id,
	.driver       = {
		.name = HIMAX_TS_NAME,
		.owner = THIS_MODULE,
		.of_match_table = himax_match_table,
	},
};

static int __init himax_ts_init(void)
{
	printk(KERN_INFO "[himax] %s\n", __func__);
	return i2c_add_driver(&himax_ts_driver);
}

static void __exit himax_ts_exit(void)
{
	i2c_del_driver(&himax_ts_driver);
	return;
}
/******************** Himax: Fundamental function ********************/

module_init(himax_ts_init);
module_exit(himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");
