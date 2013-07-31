
/*
 * drivers/input/touchscreen/sx8650.c
 *
 * Copyright (c) 2009 Semtech Corp
 *      Wayne Roberts <wroberts@semtech.com>
 *
 * Using code from:
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG	1

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/sx8650.h>


#include "../../../arch/arm/mach-omap2/include/mach/samkoonhmi.h"

/* timeout expires after pen is lifted, no more PENIRQs comming */
//#define TS_TIMEOUT		(8 * 1000000)	/* adjust with POWDLY setting */
#define TS_TIMEOUT		(24 * 1000000)	/* adjust with POWDLY setting */

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_SEQ	7

/* commands */
#define SX8650_WRITE_REGISTER	0x00
#define SX8650_READ_REGISTER	0x40
#define SX8650_SELECT_CH(ch)	(0x80 | ch)
#define SX8650_CONVERT_CH(ch)	(0x90 | ch)
#define SX8650_POWDWN			0xb0	// power down, ignore pen
#define SX8650_PENDET			0xc0	// " " with pen sensitivity
#define SX8650_PENTRG			0xe0	// " " " " and sample channels

/* register addresses */
#define I2C_REG_CTRL0		0x00
#define I2C_REG_CTRL1		0x01
#define I2C_REG_CTRL2		0x02
#define I2C_REG_CTRL3		0x03
#define I2C_REG_CHANMASK	0x04
#define I2C_REG_STAT		0x05
#define I2C_REG_SOFTRESET	0x1f

#define SOFTRESET_VALUE		0xde

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ		0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ		0x40	// I2C_REG_STAT: pen detected

/* sx8650 bits for RegCtrl1 */
#define CONDIRQ         0x20
#define FILT_NONE       0x00    /* no averaging */
#define FILT_3SA        0x01    /* 3 sample averaging */
#define FILT_5SA        0x02    /* 5 sample averaging */
#define FILT_7SA        0x03    /* 7 samples, sort, then average of 3 middle samples */

/* bits for register 2, I2CRegChanMsk */
#define CONV_X		0x80
#define CONV_Y		0x40
#define CONV_Z1		0x20
#define CONV_Z2		0x10
#define CONV_AUX	0x08

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_IMMEDIATE	0x00
#define POWDLY_1_1US		0x01
#define POWDLY_2_2US		0x02
#define POWDLY_4_4US		0x03
#define POWDLY_8_9US		0x04
#define POWDLY_17_8US		0x05
#define POWDLY_35_5US		0x06
#define POWDLY_71US		0x07
#define POWDLY_140US		0x08
#define POWDLY_280US		0x09
#define POWDLY_570US		0x0a
#define POWDLY_1_1MS		0x0b
#define POWDLY_2_3MS		0x0c
#define POWDLY_4_6MS		0x0d
#define POWDLY_9MS			0x0e
#define POWDLY_18MS			0x0f

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#ifdef LCD_ALLINONE_050INC
#define SXRST_PIN  GPIO_TO_PIN(0, 12)
#else
#define SXRST_PIN  GPIO_TO_PIN(0, 5)
#endif

#define MAX_12BIT		       ((1 << 12) - 1)

/* when changing the channel mask, also change the read length appropriately */
#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2)
#define NUM_CHANNELS_SEQ	4
#define CHAN_READ_LENGTH	(NUM_CHANNELS_SEQ * 2)

#define SX8650_MAX_DELTA	40


struct ts_event {
	u16     x;
	u16     y;
	u16     z1, z2;
};

struct sx8650 {
	struct input_dev	*input;
	char		    phys[32];
	struct hrtimer		timer;
	struct ts_event	 tc;

	struct i2c_client       *client;

	struct workqueue_struct	*ts_workq;
	struct work_struct	pen_event_work;

	spinlock_t	      lock;

	u16		     model;
	u16		     y_plate_ohms;

	unsigned		pendown;
	int		     irq;

	int		     (*get_pendown_state)(void);
	void		    (*clear_penirq)(void);
};

unsigned int bckup_x = 0, bckup_y = 0;
u16	x_min, x_max;
u16	y_min, y_max;
#define		W_SCRN		4090


//#define dev_dbg(dev, format, arg...)		\
//	printk(format, ##arg)

static void sx8650_send_event(void *tsc)
{
#if 0
	struct sx8650  *ts = tsc;
	u32	     rt;
	u16	     x, y, z1, z2;

	x = ts->tc.x;
	y = ts->tc.y;
	z1 = ts->tc.z1;
	z2 = ts->tc.z2;
    	printk("%s() in...\n\n", __func__);

	/* range filtering */
	if (y == MAX_12BIT)
		y = 0;

	if (likely(y && z1)) {
		/* compute touch pressure resistance */
		rt = z2;
		rt -= z1;
		rt *= y;
		rt *= ts->y_plate_ohms;
		rt /= z1;
		rt = (rt + 2047) >> 12;
	} else
		rt = 0;

	/* Sample found inconsistent by debouncing or pressure is beyond
	 * the maximum. Don't report it to user space, repeat at least
	 * once more the measurement
	 */
	if (rt > MAX_12BIT) {
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		return;
	}

	/* NOTE: We can't rely on the pressure to determine the pen down
	 * state, even this controller has a pressure sensor.  The pressure
	 * value can fluctuate for quite a while after lifting the pen and
	 * in some cases may not even settle at the expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * timer by reading the pen signal state (it's a GPIO _and_ IRQ).
	 */
	if (rt) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			//dev_dbg(&ts->client->dev, "DOWN\n");
			ts->pendown = 1;
			input_report_key(input, BTN_TOUCH, 1);
		}

		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
		input_report_abs(input, ABS_PRESSURE, rt);

		input_sync(input);

		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			x, y, rt);
	}
#else
	struct sx8650  *ts = tsc;
	u32	     rt;
	u16	     x, y, z1, z2;
	int readx1 = 0, ready1 = 0;
	unsigned int		prev_val_x = ~0, prev_val_y = ~0;
	unsigned int		prev_diff_x = ~0, prev_diff_y = ~0;
	unsigned int		cur_diff_x = 0, cur_diff_y = 0;
	unsigned int		val_x = 0, val_y = 0, diffx = 0, diffy = 0;
	
	x = ts->tc.x;
	y = ts->tc.y;
	z1 = ts->tc.z1;
	z2 = ts->tc.z2;
 //   	printk("%s() in...x=%d,y=%d\n\n", __func__,x,y);

	/* range filtering */
	if (y == MAX_12BIT)
		y = 0;

	if (likely(y && z1)) {
		/* compute touch pressure resistance */
		rt = z2;
		rt -= z1;
		rt *= y;
		rt *= ts->y_plate_ohms;
		rt /= z1;
		rt = (rt + 2047) >> 12;
	} else
		rt = 0;

	/* Sample found inconsistent by debouncing or pressure is beyond
	 * the maximum. Don't report it to user space, repeat at least
	 * once more the measurement
	 */
	if (rt > MAX_12BIT) {
		dev_dbg(&ts->client->dev, "ignored pressure %d\n", rt);
		return;
	}
	
	readx1 =x;
	ready1 = y;

	if (readx1 > prev_val_x)
		cur_diff_x = readx1 - prev_val_x;
	else
		cur_diff_x = prev_val_x - readx1;

	if (cur_diff_x < prev_diff_x) {
		prev_diff_x = cur_diff_x;
		val_x = readx1;
	}

	prev_val_x = readx1;

	if (ready1 > prev_val_y)
		cur_diff_y = ready1 - prev_val_y;
	else
		cur_diff_y = prev_val_y - ready1;

	if (cur_diff_y < prev_diff_y) {
		prev_diff_y = cur_diff_y;
		val_y = ready1;
	}
	prev_val_y = ready1;
#if defined  (LCD_102INC) ||defined (LCD_070INC_3UART) ||defined (LCD_035TINC)
	val_x =W_SCRN -((W_SCRN * (val_x - x_min)) /
				(x_max - x_min));
       val_y =((W_SCRN * (val_y - y_min)) /
				(y_max - y_min));
#else
	val_x =((W_SCRN * (val_x - x_min)) /
				(x_max - x_min));
       val_y =W_SCRN -
			((W_SCRN * (val_y - y_min)) /
				(y_max - y_min));
#endif
	if((val_x>4095)&&(val_x<4200))
		val_x=4090;
	if(val_x>65000)
		val_x=0;
	if((val_y>4095)&&(val_y<4200))
		val_y=4090;
	if(val_y>65000)
		val_y=0;

	if (val_x > bckup_x) {
		diffx = val_x - bckup_x;
		diffy = val_y - bckup_y;
	} else {
		diffx = bckup_x - val_x;
		diffy = bckup_y - val_y;
	}
	bckup_x = val_x;
	bckup_y = val_y;
	


	/* NOTE: We can't rely on the pressure to determine the pen down
	 * state, even this controller has a pressure sensor.  The pressure
	 * value can fluctuate for quite a while after lifting the pen and
	 * in some cases may not even settle at the expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * timer by reading the pen signal state (it's a GPIO _and_ IRQ).
	 */
	if (rt) {
//		if ((diffx < 100) && (diffy <100))
		{
			struct input_dev *input = ts->input;

			if (!ts->pendown) {
			//dev_dbg(&ts->client->dev, "DOWN\n");
				ts->pendown = 1;
				input_report_key(input, BTN_TOUCH, 1);
			}

			input_report_abs(input, ABS_X, val_x);
			input_report_abs(input, ABS_Y, val_y);
			input_report_abs(input, ABS_PRESSURE, rt);

			input_sync(input);

			dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
				val_x, val_y, rt);
		}
	}
#endif
}

static int sx8650_read_values(struct sx8650 *tsc)
{
	s32 data;
	u16 vals[NUM_CHANNELS_SEQ+1];	// +1 for last dummy read
	int length;
	int i, ret = 0;

	/* The protocol and raw data format from i2c interface:
	 * S Addr R A [DataLow] A [DataHigh] A (repeat) NA P
	 * Where DataLow has (channel | [D11-D8]), DataHigh has [D7-D0].
	 */
   // 	printk("%s() in...\n\n", __func__);

	length = i2c_master_recv(tsc->client, (char *)vals, CHAN_READ_LENGTH);

	if (likely(length == CHAN_READ_LENGTH)) {
		length >>= 1;
		for (i = 0; i < length; i++) {
			u16 ch;
			data = swab16(vals[i]);
			if (unlikely(data & 0x8000)) {
				printk(KERN_ERR "hibit @ %d\n", i);
				ret = -1;
				continue;
			}
			ch = data >> 12;
			if (ch == CH_X) {
				tsc->tc.x = data & 0xfff;
			} else if (ch == CH_Y) {
				tsc->tc.y = data & 0xfff;
			} else if (ch == CH_Z1) {
				tsc->tc.z1 = data & 0xfff;
			} else if (ch == CH_Z2) {
				tsc->tc.z2 = data & 0xfff;
			} else {
				printk(KERN_ERR "? %d %x\n", ch, data & 0xfff);
				ret = -1;
			}
		}
	} else {
		printk(KERN_ERR "%d = recv()\n", length);
		ret = -1;
	}

/*	printk(KERN_ERR "X:%03x Y:%03x Z1:%03x Z2:%03x\n",
			tsc->tc.x, tsc->tc.y, tsc->tc.z1, tsc->tc.z2);	*/

	return ret;
}

static enum hrtimer_restart sx8650_timer_handler(struct hrtimer *handle)
{
	struct sx8650 *ts = container_of(handle, struct sx8650, timer);
	struct input_dev *input = ts->input;

//    	printk("%s() in...\n\n", __func__);

	if (unlikely(ts->get_pendown_state())) {
		/* the PENIRQ is low,
		 * meaning the interrupt has not yet been serviced */
		hrtimer_forward_now(&ts->timer, ktime_set(0, TS_TIMEOUT));
		return HRTIMER_RESTART;
	}

	spin_lock(&ts->lock);

	/* This timer expires after PENIRQs havent been coming in for some time.
	 * It means that the pen is now UP. */
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	ts->pendown = 0;

	//dev_info(&ts->client->dev, "UP\n");
	spin_unlock(&ts->lock);

	return HRTIMER_NORESTART;
}

static void sx8650_pen_irq_worker(struct work_struct *work)
{
	struct sx8650 *ts = container_of(work, struct sx8650, pen_event_work);

	/* the pen is down */
   // 	printk("%s() in...\n\n", __func__);
	if (likely(sx8650_read_values(ts) == 0)) {
		/* valid data was read in */

		sx8650_send_event(ts);

	} else
		printk(KERN_ERR "fail\n");

	/* this timer upon expiration will indicate pen UP */
	hrtimer_start(&ts->timer, ktime_set(0, TS_TIMEOUT), HRTIMER_MODE_REL);

//	enable_irq(ts->irq);
}

static irqreturn_t sx8650_irq(int irq, void *handle)
{
	struct sx8650 *ts = handle;
	unsigned long flags;

//	printk("%s() in... %d\n\n", __func__,ts->get_pendown_state());
	if (ts->get_pendown_state()) {
//			printk("%s() in1... ts->pen_event_work\n\n", __func__);

		spin_lock_irqsave(&ts->lock, flags);
//			printk("%s() in2... ts->pen_event_work\n\n", __func__);

//		disable_irq(ts->irq);

		/* the reading of the samples can be time-consuming if using
		 * a slow i2c, so the work is done in a queue */
	//	printk("%s() in3... ts->pen_event_work\n\n", __func__);
		queue_work(ts->ts_workq, &ts->pen_event_work);

		spin_unlock_irqrestore(&ts->lock, flags);
	}

	if (ts->clear_penirq)
		ts->clear_penirq();


	return IRQ_HANDLED;
}


static void Sx8650_ResetTouch()
{
	gpio_direction_output(SXRST_PIN, 1);
	mdelay(100);
	gpio_direction_output(SXRST_PIN, 0);
	mdelay(100);
	gpio_direction_output(SXRST_PIN, 1);

}
static int sx8650_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8650 *ts;
	struct sx8650_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	dev_info(&client->dev, "sx8650_probe()\n");

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}
	Sx8650_ResetTouch();
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct sx8650), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = input_dev;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = sx8650_timer_handler;

	spin_lock_init(&ts->lock);

	ts->model				= pdata->model;
	ts->y_plate_ohms		= pdata->y_plate_ohms;
	ts->get_pendown_state	= pdata->get_pendown_state;
	ts->clear_penirq		= pdata->clear_penirq;

	pdata->init_platform_hw();

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "SX8650 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);


	err = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET, SOFTRESET_VALUE);
	/* soft reset: SX8651 fails to nak at the end, ignore return value */

	/* set mask to convert X, Y, Z1, Z2 for CH_SEQ */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_MASK);
	if (err != 0) {
		dev_err(&client->dev, "write mask fail");
		goto err_free_mem;
	}

	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1, CONDIRQ | FILT_7SA); //FILT_3SA
	if (err != 0) {
		dev_err(&client->dev, "writereg1 fail");
		goto err_free_mem;
	}
	/* set POWDLY settling time -- adjust TS_TIMEOUT accordingly */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, POWDLY_1_1MS);//POWDLY_9MS, POWDLY_4_6MS, POWDLY_9MS, 
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_free_mem;
	}

	ts->ts_workq = create_singlethread_workqueue("sx8650");
	if (ts->ts_workq == NULL) {
		dev_err(&client->dev, "failed to create workqueue\n");
		goto err_free_mem;
	}

	INIT_WORK(&ts->pen_event_work, sx8650_pen_irq_worker);

	ts->irq = client->irq;

	err = request_irq(ts->irq, sx8650_irq, IRQF_TRIGGER_FALLING,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	x_min=110;
	x_max=3950;
	y_min=320;
	y_max=3750;

	dev_info(&client->dev, "registered with irq (%d)\n", ts->irq);

	/* enter pen-trigger mode */
	err = i2c_smbus_write_byte(client, SX8650_PENTRG);
	if (err != 0) {
		dev_err(&client->dev, "enter fail");
		goto err_free_mem;
	}
	return 0;

 err_free_irq:
	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int sx8650_remove(struct i2c_client *client)
{
	struct sx8650  *ts = i2c_get_clientdata(client);
	struct sx8650_platform_data *pdata;

	pdata = client->dev.platform_data;
	pdata->exit_platform_hw();

	cancel_work_sync(&ts->pen_event_work);
	destroy_workqueue(ts->ts_workq);

	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id sx8650_idtable[] = {
	{ "sx8650", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx8650_idtable);

static struct i2c_driver sx8650_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sx8650"
	},
	.id_table       = sx8650_idtable,
	.probe	  = sx8650_probe,
	.remove	 = sx8650_remove,
};

static int __init sx8650_init(void)
{
	return i2c_add_driver(&sx8650_driver);
}

static void __exit sx8650_exit(void)
{
	i2c_del_driver(&sx8650_driver);
}

module_init(sx8650_init);
module_exit(sx8650_exit);

MODULE_AUTHOR("Wayne Roberts <wroberts@semtech.com>");
MODULE_DESCRIPTION("SX8650 TouchScreen Driver");
MODULE_LICENSE("GPL");

