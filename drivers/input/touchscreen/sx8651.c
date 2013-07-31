
/*
 * drivers/input/touchscreen/sx8651.c
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/sx865x.h>


/* timeout expires after pen is lifted, no more PENIRQs comming */
//#define TS_TIMEOUT		(8 * 1000000)	/* adjust with POWDLY setting */
#define TS_TIMEOUT		(90 * 1000000)	/* adjust with POWDLY setting */

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_RX	5
#define CH_RY	6
#define CH_SEQ	7

/* commands */
#define SX8651_WRITE_REGISTER	0x00
#define SX8651_READ_REGISTER	0x40
#define SX8651_SELECT_CH(ch)	(0x80 | ch)
#define SX8651_CONVERT_CH(ch)	(0x90 | ch)
#define SX8651_POWDWN			0xb0	// power down, ignore pen
#define SX8651_PENDET			0xc0	// " " with pen sensitivity
#define SX8651_PENTRG			0xe0	// " " " " and sample channels

/* register addresses */
#define I2C_REG_CTRL0		0x00
#define I2C_REG_CTRL1		0x01
#define I2C_REG_CTRL2		0x02
#define I2C_REG_CTRL3		0x03
#define I2C_REG_CHANMASK	0x04
#define I2C_REG_STAT		0x05
#define I2C_REG_NVM1		0x09
#define I2C_REG_NVMCTRL		0x10
#define I2C_REG_TESTCTRL	0x15
#define I2C_REG_SOFTRESET	0x1f

#define SOFTRESET_VALUE		0xde

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ		0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ		0x40	// I2C_REG_STAT: pen detected

/* sx8651 bits for RegCtrl1 */
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
#define CONV_RX		0x04
#define CONV_RY		0x02

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_IMMEDIATE	0x00
#define POWDLY_1_1US		0x01
#define POWDLY_2_2US		0x02
#define POWDLY_4_4US		0x03
#define POWDLY_8_9US		0x04
#define POWDLY_17_8US		0x05
#define POWDLY_35_5US		0x06
#define POWDLY_71US			0x07
#define POWDLY_140US		0x08
#define POWDLY_280US		0x09
#define POWDLY_570US		0x0a
#define POWDLY_1_1MS		0x0b
#define POWDLY_2_3MS		0x0c
#define POWDLY_4_6MS		0x0d
#define POWDLY_9MS			0x0e
#define POWDLY_18MS			0x0f

/* NVM1 bits */
#define NVM1_MLTTCHEN		0x02

/* internal-silicon definitions */
#define TESTCTRL_UNLOCK_A	0xc3
#define TESTCTRL_UNLOCK_B	0x3c
#define NVMCTRL_UNLOCK_A	0x50
#define NVMCTRL_UNLOCK_B	0xa0

#define MAX_12BIT		       ((1 << 12) - 1)

/* when changing the channel mask, also change the read length appropriately */
#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2 | CONV_RX | CONV_RY )
#define NUM_CHANNELS_SEQ	6
#define CHAN_READ_LENGTH	(NUM_CHANNELS_SEQ * 2)

#define SX8651_MAX_DELTA	40

#ifdef CONFIG_MACH_OMAP3EVM
	#define OMAP3EVM_WIDTH_THRESHOLD	1280 // rx
	#define OMAP3EVM_HEIGHT_THRESHOLD	1070 // ry
#endif

struct ts_event {
	u16     x;
	u16     y;
	u16     z1, z2;
	u16     rx, ry;
};

struct sx8651 {
	struct input_dev	*input;
	char		    phys[32];
	struct hrtimer		timer;
	struct ts_event	 tc;

	struct i2c_client       *client;

	struct sx865x_platform_data	*pdata;

	struct workqueue_struct	*ts_workq;
	struct work_struct	pen_event_work;

	spinlock_t	      lock;

	u16		     model;
	u16		     y_plate_ohms;

	unsigned		pendown;
	int		     irq;

	int		     (*get_pendown_state)(void);
	void		    (*clear_penirq)(void);

	int _ns_count;	//tmp debug
};

#ifdef CONFIG_MACH_OMAP3EVM

#define OMAP3EVM_XMIN		0x136
#define OMAP3EVM_XMAX		0xe84
#define OMAP3EVM_YMIN		0x0d9
#define OMAP3EVM_YMAX		0xec6

#endif


static void sx8651_send_event(void *sx)
{
	struct sx8651  *ts = sx;
	struct sx865x_platform_data *pdata = ts->pdata;
	u32	     rt;
	u16	     x, y, z1, z2;

	x = ts->tc.x;
	y = ts->tc.y;
	z1 = ts->tc.z1;
	z2 = ts->tc.z2;

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
		int finger2_pressed;

		if (!ts->pendown) {
			//dev_dbg(&ts->client->dev, "DOWN\n");
			printk(KERN_ERR "DOWN\n");
			ts->pendown = 1;
			//input_report_key(input, BTN_TOUCH, 1);
		}

		/* temporary hard-coded calibration */
		if (ts->tc.rx > OMAP3EVM_WIDTH_THRESHOLD || ts->tc.ry > OMAP3EVM_HEIGHT_THRESHOLD) {
			finger2_pressed = 1;
			ts->tc.rx -= OMAP3EVM_WIDTH_THRESHOLD;
			ts->tc.ry -= OMAP3EVM_HEIGHT_THRESHOLD;
		} else
			finger2_pressed = 0;

#ifdef CONFIG_MACH_OMAP3EVM
		x = pdata->x_max -
			((pdata->x_max * (x - OMAP3EVM_XMIN)) / (OMAP3EVM_XMAX- OMAP3EVM_XMIN));
		y = pdata->y_max -
			((pdata->y_max * (y - OMAP3EVM_YMIN)) / (OMAP3EVM_YMAX - OMAP3EVM_YMIN));
#endif
		input_report_key(input, BTN_TOUCH, 1);	// BTN_TOUCH is used in KeyInputQueue: mAbs.mDown[0]
		//BTN_2 ignored in KeyInputQueue

		//printk(KERN_ERR "1 rt=%d\n", rt);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, rt);
		input_report_abs(input, ABS_MT_WIDTH_MAJOR, rt >> 2);	// becomes Size
		input_report_abs(input, ABS_MT_POSITION_X, x);
		input_report_abs(input, ABS_MT_POSITION_Y, y);
		input_mt_sync(input);
		if (finger2_pressed) {
			//printk(KERN_ERR "rx=%d ry=%d rt=%d\n", ts->tc.rx, ts->tc.ry, rt);
			//printk(KERN_ERR "2 rt=%d\n", rt+1);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, rt+1);	// must be different than prevous or will be ignored ??
			input_report_abs(input, ABS_MT_WIDTH_MAJOR, (rt >> 2) + 1);	// becomes Size
			input_report_abs(input, ABS_MT_POSITION_X, ts->tc.rx);
			input_report_abs(input, ABS_MT_POSITION_Y, ts->tc.ry);
			input_mt_sync(input);
		}
		input_sync(input);

		dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n",
			x, y, rt);
	}

}

static int sx8651_read_values(struct sx8651 *tsc)
{
	s32 data;
	u16 vals[NUM_CHANNELS_SEQ+1];	// +1 for last dummy read
	int length;
	int i, ret = 0;

	/* The protocol and raw data format from i2c interface:
	 * S Addr R A [DataLow] A [DataHigh] A (repeat) NA P
	 * Where DataLow has (channel | [D11-D8]), DataHigh has [D7-D0].
	 */
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
			} else if (ch == CH_RX) {
				tsc->tc.rx = data & 0xfff;
			} else if (ch == CH_RY) {
				tsc->tc.ry = data & 0xfff;
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

static enum hrtimer_restart sx8651_timer_handler(struct hrtimer *handle)
{
	struct sx8651 *ts = container_of(handle, struct sx8651, timer);
	struct input_dev *input = ts->input;

	if (unlikely(ts->get_pendown_state())) {
		/* the PENIRQ is low,
		 * meaning the interrupt has not yet been serviced */
		ts->_ns_count++;
/*		if (list_empty(&ts->ts_workq->list))
			printk(KERN_ERR "E %d\n", ts->_ns_count);
		else
			printk(KERN_ERR "F %d\n", ts->_ns_count);	*/
		hrtimer_forward_now(&ts->timer, ktime_set(0, TS_TIMEOUT));

		// kernel has been abused: work queue dropped?
		printk(KERN_ERR "F %d\n", ts->_ns_count);
		if (ts->_ns_count > 10) {	
			queue_work(ts->ts_workq, &ts->pen_event_work);
			ts->_ns_count = 0;
		}

		return HRTIMER_RESTART;
	}

	spin_lock(&ts->lock);

	/* This timer expires after PENIRQs havent been coming in for some time.
	 * It means that the pen is now UP. */
	input_report_key(input, BTN_TOUCH, 0);
	//single-touch only: input_report_abs(input, ABS_PRESSURE, 0);
	input_mt_sync(input);
	input_sync(input);

	ts->pendown = 0;

	//dev_info(&ts->client->dev, "UP\n");
	printk(KERN_ERR "UP\n");
	spin_unlock(&ts->lock);

	return HRTIMER_NORESTART;
}

volatile char _sending_events = 0;

static void sx8651_pen_irq_worker(struct work_struct *work)
{
	struct sx8651 *ts = container_of(work, struct sx8651, pen_event_work);

//	printk(KERN_ERR "sxworker\n");
	ts->_ns_count = 0;
	/* the pen is down */

	if (likely(sx8651_read_values(ts) == 0)) {
		/* valid data was read in */

		if (_sending_events)
			printk(KERN_ERR "ASE!\n");
		_sending_events = 1;
		sx8651_send_event(ts);
		_sending_events = 0;

	} else
		printk(KERN_ERR "fail\n");

	/* this timer upon expiration will indicate pen UP */
	hrtimer_start(&ts->timer, ktime_set(0, TS_TIMEOUT), HRTIMER_MODE_REL);

	enable_irq(ts->irq);
}

static irqreturn_t sx8651_irq(int irq, void *handle)
{
	struct sx8651 *ts = handle;
	unsigned long flags;

//	printk(KERN_ERR "sxirq\n");
	if (ts->get_pendown_state()) {
		spin_lock_irqsave(&ts->lock, flags);
		disable_irq(ts->irq);

		/* the reading of the samples can be time-consuming if using
		 * a slow i2c, so the work is done in a queue */
		queue_work(ts->ts_workq, &ts->pen_event_work);

		spin_unlock_irqrestore(&ts->lock, flags);
	}

	if (ts->clear_penirq)
		ts->clear_penirq();


	return IRQ_HANDLED;
}

static int sx865x_read_register(struct sx8651 *ts, unsigned char reg_addr)
{
	struct i2c_msg msg[2];
	char buf0, buf1;
	int ret;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf0;
	buf0 = reg_addr | SX8651_READ_REGISTER;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf1;
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		return ret;
	} else
		return buf1;
}

static int sx8651_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx8651 *ts;
	struct sx865x_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	char buf1;

	dev_info(&client->dev, "sx8651_probe()\n");

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct sx8651), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = input_dev;

	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = sx8651_timer_handler;

	spin_lock_init(&ts->lock);

	ts->pdata				= pdata;
	ts->model				= pdata->model;
	ts->y_plate_ohms		= pdata->y_plate_ohms;
	ts->get_pendown_state	= pdata->get_pendown_state;
	ts->clear_penirq		= pdata->clear_penirq;

	pdata->init_platform_hw();

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "SX8651 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(EV_SYN, input_dev->evbit);

	/* only for single-touch: ABS_X, ABS_Y, ABS_PRESSURE */
	/* ignored by KeyInputQueue: ABS_HAT0X, ABS_HAT0Y, BTN_2 */

	// register as multitouch device: set ABS_MT_TOUCH_MAJOR, ABS_MT_POSITION_X, ABS_MT_POSITION_Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 256, 0, 0);	// becomes Size

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	/****** hardware init.. **********/

	err = i2c_smbus_write_byte_data(client, I2C_REG_SOFTRESET, SOFTRESET_VALUE);
	/* soft reset: SX8651 fails to nak at the end, ignore return value */

	/* read NVM1 to find if the chip has touch-width/touch-height enabled */
	err = sx865x_read_register(ts, I2C_REG_NVM1);
	if (err < 0) {
		printk(KERN_ERR "i2c_transfer failed\n");
		goto err_free_mem;
	}
	buf1 = err;

	if ((buf1 & NVM1_MLTTCHEN) == 0) {
		printk(KERN_ERR "need unlock, buf1: %x\n", buf1);

		/* SX8650 unlock to enable width, height of touch */
		err = i2c_smbus_write_byte_data(client, I2C_REG_TESTCTRL, TESTCTRL_UNLOCK_A);
		if (err != 0) {
			dev_err(&client->dev, "test unlock_a fail");
			goto err_free_mem;
		}
		err = i2c_smbus_write_byte_data(client, I2C_REG_TESTCTRL, TESTCTRL_UNLOCK_B);
		if (err != 0) {
			dev_err(&client->dev, "test unlock_b fail");
			goto err_free_mem;
		}

		err = i2c_smbus_write_byte_data(client, I2C_REG_NVMCTRL, NVMCTRL_UNLOCK_A);
		if (err != 0) {
			dev_err(&client->dev, "nvm unlock_a fail");
			goto err_free_mem;
		}
		err = i2c_smbus_write_byte_data(client, I2C_REG_NVMCTRL, NVMCTRL_UNLOCK_B);
		if (err != 0) {
			dev_err(&client->dev, "nvm unlock_b fail");
			goto err_free_mem;
		}

		// enable chip to be sx8651
		err = i2c_smbus_write_byte_data(client, I2C_REG_NVM1, buf1 | NVM1_MLTTCHEN);
		if (err != 0) {
			dev_err(&client->dev, "write nvm1 fail");
			goto err_free_mem;
		}

		err = sx865x_read_register(ts, I2C_REG_NVM1);
		if (err < 0) {
			printk(KERN_ERR "i2c_transfer failed\n");
			goto err_free_mem;
		}
		buf1 = err;
		printk(KERN_ERR "unlocked: %x\n", buf1);
		
	} // ..need unlock 8650->8651

	/* set mask to select channels to be converted */
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
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, POWDLY_2_3MS);//POWDLY_9MS, POWDLY_4_6MS, POWDLY_9MS, 
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_free_mem;
	}

	ts->ts_workq = create_singlethread_workqueue("sx8651");
	if (ts->ts_workq == NULL) {
		dev_err(&client->dev, "failed to create workqueue\n");
		goto err_free_mem;
	}

	INIT_WORK(&ts->pen_event_work, sx8651_pen_irq_worker);

	ts->irq = client->irq;

	err = request_irq(ts->irq, sx8651_irq, IRQF_TRIGGER_FALLING,
			client->dev.driver->name, ts);
	if (err < 0) {
		dev_err(&client->dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	dev_info(&client->dev, "registered with irq (%d)\n", ts->irq);


	/* enter pen-trigger mode */
	err = i2c_smbus_write_byte(client, SX8651_PENTRG);
	if (err != 0) {
		dev_err(&client->dev, "enter fail");
		goto err_free_mem;
	}

	ts->_ns_count = 0;
	return 0;

 err_free_irq:
	free_irq(ts->irq, ts);
	hrtimer_cancel(&ts->timer);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int sx8651_remove(struct i2c_client *client)
{
	struct sx8651  *ts = i2c_get_clientdata(client);
	struct sx865x_platform_data *pdata;

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

static struct i2c_device_id sx8651_idtable[] = {
	{ "sx8651", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx8651_idtable);

static struct i2c_driver sx8651_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sx8651"
	},
	.id_table       = sx8651_idtable,
	.probe	  = sx8651_probe,
	.remove	 = sx8651_remove,
};

static int __init sx8651_init(void)
{
	return i2c_add_driver(&sx8651_driver);
}

static void __exit sx8651_exit(void)
{
	i2c_del_driver(&sx8651_driver);
}

module_init(sx8651_init);
module_exit(sx8651_exit);

MODULE_AUTHOR("Wayne Roberts <wroberts@semtech.com>");
MODULE_DESCRIPTION("SX8651 TouchScreen Driver");
MODULE_LICENSE("GPL");

