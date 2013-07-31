/*
 * SX8652 based touchscreen and sensor driver
 *
 *  Copyright (c) 2010 Wayne Roberts
 *
 * Using code from:
 *  Copyright (c) 2005 David Brownell
 *  Copyright (c) 2006 Nokia Corporation
 *  Various changes: Imre Deak <imre.deak@nokia.com>
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
*	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/sx8652.h>
#include <linux/hwmon.h>


#define	MAX_12BIT	((1<<12)-1)

#define SX8652_TS_PENUP_TIME	100

/* analog channels */
#define CH_X	0
#define CH_Y	1
#define CH_Z1	2
#define CH_Z2	3
#define CH_AUX	4
#define CH_SEQ	7

/* commands */
#define SX8652_CMD_WRITEREG		0x00
#define SX8652_CMD_READCHAN		0x20
#define SX8652_CMD_READREG		0x40
#define SX8652_CMD_SELECT		0x80
#define SX8652_CMD_CONVERT		0x90
#define SX8652_CMD_MANAUTO		0xb0
#define SX8652_CMD_PENDET		0xc0
#define SX8652_CMD_PENTRG		0xe0

/* register addresses */
#define SX8652_REG_CTRL0	0x00
#define SX8652_REG_CTRL1	0x01
#define SX8652_REG_CTRL2	0x02
#define SX8652_REG_CHANMSK	0x04
#define SX8652_REG_STATUS	0x05
#define SX8652_REG_RESET	0x1f

/* for POWDLY or SETDLY: */
#define DLY_0_5US	0x00
#define DLY_1_1US	0x01
#define DLY_2_2US	0x02
#define DLY_4_4US	0x03
#define DLY_9US		0x04
#define DLY_18US	0x05
#define DLY_35uS	0x06
#define DLY_71US	0x07
#define DLY_140US	0x08
#define DLY_280US	0x09
#define DLY_570US	0x0a
#define DLY_1_1MS	0x0b
#define DLY_2_3MS	0x0c
#define DLY_4_5MS	0x0d
#define DLY_9MS		0x0e
#define DLY_18MS	0x0f

// RegCtrl1
#define CONDIRQ 	0x20
#define FILT_NONE	0x00
#define FILT_3SA	0x01
#define FILT_5SA	0x02
#define FILT_7SA	0x03

#define CONV_X		0x80
#define CONV_Y		0x40
#define CONV_Z1		0x20
#define CONV_Z2		0x10
#define CONV_AUX	0x08

#define CHAN_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2)

#define RESET_VALUE	0xde

#define NUM_READ_REGS	4	/* count of words to read */


#define		X_UP		3830//3556//3875
#define		X_DN		100//544//200

// 3840 - 240
#define		H_SCRN		4090

#define		Y_UP		3830//3456//3840
#define		Y_DN		384//627//200


struct sx8652 {
	struct input_dev	*input;
	char			phys[32];
	struct spi_device	*spi;
	spinlock_t		lock;
	struct mutex	mutex;
	unsigned		disabled:1;
	struct timer_list	penup_timer;
	u16			model;
	u16			y_plate_ohms;
	u16			pressure_max;

	u8 pen_down;

#if 0//#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	struct attribute_group	*attr_group;
	struct device		*hwmon;
#endif


	struct spi_message	read_msg;
	struct spi_transfer	read_xfer;
	u8 read_cmd;
	u8 data[(NUM_READ_REGS << 1) + 1];
	u8 spi_active;

	int			(*get_pendown_state)(void);
	void			(*wait_for_sync)(void);
};


static struct attribute *sx8652_attributes[] = {
	NULL,
};

static struct attribute_group sx8652_attr_group = {
	.attrs = sx8652_attributes,
};

//#define dev_dbg(dev, format, arg...)		\
//	printk(format, ##arg)

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define SCK_PIN  GPIO_TO_PIN(0, 2)
#define MOSI_PIN  GPIO_TO_PIN(0, 3)
#define MISO_PIN  GPIO_TO_PIN(0, 4)
#define CS_PIN  GPIO_TO_PIN(0, 5)
#define IRQ_PIN  GPIO_TO_PIN(3, 15)
#define SPIRESET_PIN  GPIO_TO_PIN(3, 16)

unsigned int bckup_x = 0, bckup_y = 0;
u16	x_min, x_max;
u16	y_min, y_max;

static void sx8652_async_rx(void *ads/*,u16 data_x,u16 data_y*/);



void SPI_SCK(u8 flag)
{
	if(flag)
		gpio_direction_output(SCK_PIN, 1);
	else
		gpio_direction_output(SCK_PIN, 0);
}

void SPI_MOSI(u8 flag)
{
	if(flag)
		gpio_direction_output(MOSI_PIN, 1);
	else
		gpio_direction_output(MOSI_PIN, 0);
}

u8 SPI_MISO()
{
	return gpio_get_value(MISO_PIN);
}

void SPI_CS(u8 flag)
{
	if(flag)
		gpio_direction_output(CS_PIN, 1);
	else
		gpio_direction_output(CS_PIN, 0);
}
void Delay_200NS()
{
	u8 i,j;
	for(i=0;i<10;i++)
		for(j=0;j<2;j++)
			;
}

void SxSpiWriteByte(u8 data)
{
	u8 i, buf;

	buf = 0;

	// 写命令, output
	for( i = 0; i < 8; i++ )
	{
		buf = (data >> (7-i)) & 0x1 ;	//MSB在前,LSB在后

       	 // write
        	if (buf)
        	{
        		// bit 1
        		SPI_MOSI(1);		//时钟上升沿锁存DIN
        	}
        	else
        	{
        		// bit 0
        		SPI_MOSI(0);		//时钟上升沿锁存DIN
        	} 
		Delay_200NS();		
		SPI_SCK(1); 		//开始发送命令字
		
		Delay_200NS();		
		SPI_SCK(0); 		//时钟脉冲，一共8个		
	}

}





u8 SxSpiReadDByte()
{
	u8 i, buf;

	buf = 0;

	// 读数据input
	for( i = 0; i < 8; i++ )
	{
        	Delay_200NS();
		SPI_SCK(1);

        	buf = (buf << 1) | ((SPI_MISO()) ? 1 : 0);
		Delay_200NS();
		
		SPI_SCK(0); 		
	}
    	Delay_200NS();

	return( buf );	
	
}




u16 SxGpioReadReg(u8 reg)
{
    	u8 i, buf;
	u8 data;

	// send command
	buf = 0;
	data = (SX8652_CMD_READREG | reg) & 0xFF;

	SPI_SCK(0);	
	SPI_CS(0);	      		//芯片启动
	Delay_200NS();

	// 写命令, output
	for( i = 0; i < 8; i++ )
    	{
    		buf = (data >> (7-i)) & 0x1 ;   //MSB在前,LSB在后
				if (buf)
        	{
            		SPI_MOSI(1);        //时钟上升沿锁存DIN
        	}
        	else
        	{
            		SPI_MOSI(0);        //时钟上升沿锁存DIN
        	}
        
        	Delay_200NS();        
        	SPI_SCK(1);         //开始发送命令字
        
       	Delay_200NS();        
        	SPI_SCK(0);         //时钟脉冲，一共8个     
    	}

    	Delay_200NS();

    	// 读数据input    
    	buf = 0;
    	for( i = 0; i < 8; i++ )
    	{
       	 Delay_200NS();
        	SPI_SCK(1);

        	buf = (buf << 1) | ((SPI_MISO()) ? 1 : 0);
        
        	Delay_200NS();
        	SPI_SCK(0);
    	}
    
    	Delay_200NS();
    	SPI_CS(1);
	
	return ( buf );
}

void SxGpioSpiStart( )
{
    SPI_SCK(0);
    Delay_200NS();
    
    SPI_CS(1);
    
    SPI_SCK(1);
    Delay_200NS();
    
    SPI_CS(0);	      		//芯片启动
    
    SPI_SCK(0);
    Delay_200NS();
}



void SxGpioSpiStop( )
{
    SPI_SCK(0);
    Delay_200NS();
    
    SPI_CS(1);    
    
    SPI_SCK(1);
    Delay_200NS();
    
    SPI_CS(1);	      		//芯片停止
    
    SPI_SCK(0);
    Delay_200NS();
}




void SxGpioResetTouch()
{
	gpio_direction_output(SPIRESET_PIN, 1);
	mdelay(100);
	gpio_direction_output(SPIRESET_PIN, 0);
	mdelay(100);
	gpio_direction_output(SPIRESET_PIN, 1);
	
	SxGpioSpiStart();

	printk("new reset 1\n");
	SPI_SCK(0);
	SPI_CS(0);              //芯片启动
	
	SxSpiWriteByte(SX8652_CMD_WRITEREG|SX8652_REG_RESET);
	SxSpiWriteByte(0xDE);    

	SPI_CS(1);          // 停止芯片
	SPI_SCK(0);
	printk("2\n");
}

void SxGpioSetMode(u8 data)
{
	// 
	//#define MANAUTO 	0xB0 	// 1 0 1 1 x x x x Enter manual or automatic mode.
	//#define PENDET 	0xC0 	// 1 1 0 0 x x x x Enter pen detect mode.
	//#define PENTRG 	0xE0 	// 1 1 1 0 x x x x Enter pen trigger mode.
	//
   
   SPI_SCK(0);
   SPI_CS(0);              //芯片启动
   SxSpiWriteByte(data);
   SPI_CS(1);          // 停止芯片
   SPI_SCK(0);

}



void SxGpioConfigReg(u8 reg, u8 data)
{
	SPI_SCK(0);
	SPI_CS(0);              //芯片启动

  	SxSpiWriteByte(reg);
  	SxSpiWriteByte(data);

	SPI_CS(1);          // 停止芯片
  	SPI_SCK(0);

}


// send the data to the spi mosi port
// 此函数读写正确
// 本芯片支持 5 us  完成一次数据读取
void SxGpioReadChannelData(void *ads/*short  *pData_x, short *pData_y*/)
{
    u8 i, j, buf;
	u8 data;
    u8 uByte[8];
	struct sx8652 *ts = ads;
	u16 *data_ptr;

	// send command
    buf = 0;
    data = SX8652_CMD_READCHAN;

    SPI_SCK(0);	
    Delay_200NS();
    SPI_CS(0);	      		//芯片启动

    // 写命令, output
    for( i = 0; i < 8; i++ )
    {
        buf = (data >> (7-i)) & 0x1 ;   //MSB在前,LSB在后

        if (buf)
        {
            SPI_MOSI(1);        //时钟上升沿锁存DIN
        }
        else
        {
            SPI_MOSI(0);        //时钟上升沿锁存DIN
        }
        
        Delay_200NS();        
        SPI_SCK(1);         //开始发送命令字
        
        Delay_200NS();        
        SPI_SCK(0);         //时钟脉冲，一共8个     
    }

    Delay_200NS();

    // 读数据input    
    for( i = 0; i < 4; i++ )    
    {
    	uByte[i] = 0;
        for( j = 0; j < 8; j++ )
        {
	        Delay_200NS();
	        SPI_SCK(1);

	        uByte[i] = (uByte[i] << 1) | ((SPI_MISO()) ? 1 : 0);
	        
	        Delay_200NS();
	        SPI_SCK(0);
        }        
        Delay_200NS();
    }

    SPI_CS(1);
    SPI_SCK(0);
    Delay_200NS();
	data_ptr = (u16 *)&ts->data[0];

   data_ptr[0]  = uByte[0] << 8;
    data_ptr[0] += uByte[1] ;
   data_ptr[1]  = uByte[2] << 8;
    data_ptr[1] += uByte[3] ;
   data_ptr[2]  = uByte[4] << 8;
    data_ptr[2] += uByte[5] ;
   data_ptr[3]  = uByte[6] << 8;
    data_ptr[3] += uByte[7] ;

}


static void sx8652_ts_penup_timer_handler(unsigned long data)
{
    struct sx8652 *ts = (struct sx8652 *)data;

//	printk( "penup_timer_handler %d spi_active=%d\n", ts->pen_down, ts->spi_active);
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);

	ts->pen_down = 0;
}


static int __devinit setup_pendown(struct spi_device *spi, struct sx8652 *ts)
{
	struct sx8652_platform_data *pdata = spi->dev.platform_data;
	int err;

	if (!pdata->get_pendown_state && !gpio_is_valid(pdata->gpio_pendown)) {
		dev_err(&spi->dev, "no get_pendown_state nor gpio_pendown?\n");
		return -EINVAL;
	}

	if (pdata->get_pendown_state) {
		ts->get_pendown_state = pdata->get_pendown_state;
		printk(KERN_ERR "get_pendown_state from pdata: %p\n", ts->get_pendown_state);
		return 0;
	}

	err = gpio_request(pdata->gpio_pendown, "sx8652_pendown");
	if (err) {
		dev_err(&spi->dev, "failed to request pendown GPIO%d\n",
				pdata->gpio_pendown);
		return err;
	}

	return 0;
}

static irqreturn_t sx8652_irq(int irq, void *handle)
{
	struct sx8652 *ts = handle;
	unsigned long flags;

		
	/* If insufficient pullup resistor on nIRQ line:
	 * may need to make sure that pen is really down here, due to spurious interrupts  */
	if (likely(ts->get_pendown_state())) {
		if (ts->spi_active)
			return IRQ_HANDLED;

		ts->spi_active = 1;
		SxGpioReadChannelData(ts);// spi_async(ts->spi, &ts->read_msg);
		spin_lock_irqsave(&ts->lock, flags);
		sx8652_async_rx(ts);
		spin_unlock_irqrestore(&ts->lock, flags);

	    /* kick pen up timer */
		mod_timer(&ts->penup_timer,
			jiffies + msecs_to_jiffies(SX8652_TS_PENUP_TIME));
	} else
		printk(KERN_ERR "irq: pen up\n");

	return IRQ_HANDLED;
}

#if 0//#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
static void sx8652_hwmon_unregister(struct spi_device *spi,
				     struct sx8652 *ts)
{
	if (ts->hwmon) {
		sysfs_remove_group(&spi->dev.kobj, ts->attr_group);
		hwmon_device_unregister(ts->hwmon);
	}
}
#endif /* ...HWMON */

static void null_wait_for_sync(void)
{
}

/* sx8652_async_rx(): callback, when spi readchannels is complete */
static void sx8652_async_rx(void *ads)
{
	struct sx8652 *ts = ads;
//	unsigned long flags;
	u16 *data_ptr;
	u32 rt;
	int i;
	int readx1 = 0, ready1 = 0, z1 = 0, z2 = 0;
	unsigned int		prev_val_x = ~0, prev_val_y = ~0;
	unsigned int		prev_diff_x = ~0, prev_diff_y = ~0;
	unsigned int		cur_diff_x = 0, cur_diff_y = 0;
	unsigned int		val_x = 0, val_y = 0, diffx = 0, diffy = 0;
	

	// first byte is command of readchan

	data_ptr = (u16 *)&ts->data[0];
	readx1 = data_ptr[0] & 0xfff;
	ready1 = data_ptr[1] & 0xfff;
	z1 = data_ptr[2] & 0xfff;
	z2 = data_ptr[3] & 0xfff;

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

	val_x =((x_max * (val_x - x_min)) /
				(x_max - x_min));
       val_y =y_max -
			((y_max * (val_y - y_min)) /
				(y_max - y_min));

	if (val_x > bckup_x) {
		diffx = val_x - bckup_x;
		diffy = val_y - bckup_y;
	} else {
		diffx = bckup_x - val_x;
		diffy = bckup_y - val_y;
	}
	bckup_x = val_x;
	bckup_y = val_y;

	rt = z2;
	rt -= z1;
	rt *= val_y;
	rt *= ts->y_plate_ohms;
	rt /= z1;
	rt = (rt + 2047) >> 12;
	if (rt > MAX_12BIT) {
		dev_err(&ts->spi->dev, "ignored pressure %d\n", rt);
		goto end;
	}


	if (rt)
	{
		if ((diffx < 15) && (diffy < 15))
		{
			if (!ts->pen_down) 
			{
	//			printk( "pendown\n");
				input_report_key(ts->input, BTN_TOUCH, 1);
				ts->pen_down = 1;
			}

			input_report_abs(ts->input, ABS_X, val_x);
			input_report_abs(ts->input, ABS_Y, val_y);
			input_report_abs(ts->input, ABS_PRESSURE, rt);

			input_sync(ts->input);

			dev_dbg(&ts->spi->dev, "point(%4d,%4d), pressure (%4u)\n",
			val_x, val_y, rt);
//			printk("point(%4d,%4d), pressure (%4u)\n",val_x, val_y, rt);
		}
	}

end:
	ts->spi_active = 0;

    /* kick pen up timer - to make sure it expires again(!) */
	mod_timer(&ts->penup_timer, jiffies + msecs_to_jiffies(SX8652_TS_PENUP_TIME));

}

void SxGpioReadSpiReg( )
{
	u16 r_data = 0;

	printk(("Enter SxGpioReadSpiReg ------ NEW ------- \n"));


	Delay_200NS;

	r_data = SxGpioReadReg(SX8652_REG_CTRL0);
	printk("The RACTL0 data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_CTRL0), r_data);

	r_data = SxGpioReadReg(SX8652_REG_CTRL1);
	printk("The RACTL1 data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_CTRL1), r_data);

	r_data = SxGpioReadReg(SX8652_REG_CTRL2);
	printk("The RACTL2 data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_CTRL2), r_data);

	r_data = SxGpioReadReg(SX8652_REG_CHANMSK);
	printk("The RACMSK data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_CHANMSK), r_data);

	r_data = SxGpioReadReg(SX8652_REG_STATUS);
	printk("The RASTAT data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_STATUS), r_data);

	r_data = SxGpioReadReg(SX8652_REG_RESET);
	printk("The RASRST data is: [%08X]=[%08X]\n", (SX8652_CMD_READREG | SX8652_REG_RESET), r_data);

	// =========================

	printk(("Leave SxGpioReadSpiReg\n"));	  

}

static int __devinit sx8652_probe(struct spi_device *spi)
{
	struct sx8652 *ts;
	struct input_dev *input_dev;
	struct sx8652_platform_data	*pdata = spi->dev.platform_data;
	int err = -1;
	struct spi_message		*m;
	struct spi_transfer		*x;
	u8 txbuf[2];

	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* don't exceed max specified SCLK frequency */
	if (spi->max_speed_hz > 5000000) {
		dev_dbg(&spi->dev, "SCLK %d KHz?\n", spi->max_speed_hz/1000);
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct sx8652), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);

	ts->spi = spi;
	ts->input = input_dev;

	/* Send a software reset command */


    	init_timer(&ts->penup_timer);
    	setup_timer(&ts->penup_timer, sx8652_ts_penup_timer_handler,
            (unsigned long)ts);

	spin_lock_init(&ts->lock);
	mutex_init(&ts->mutex);

	ts->model = pdata->model ? : 8652;
	ts->y_plate_ohms = pdata->y_plate_ohms ? : 400;
	ts->pressure_max = pdata->pressure_max ? : ~0;

	err = setup_pendown(spi, ts);
	if (err)
		goto err_free_mem;

	ts->wait_for_sync = pdata->wait_for_sync ? : null_wait_for_sync;
	//for older kernel: snprintf(ts->phys, sizeof(ts->phys), "%s/input0", spi->dev.bus_id);
	//for 2.6.32:
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&spi->dev));

	input_dev->name = "SX8652 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, pdata->pressure_max, 0, 0);
	x_min=pdata->x_min;
	x_max=pdata->x_max;
	y_min=pdata->y_min;
	y_max=pdata->y_max;

	m =	&ts->read_msg;
	x = &ts->read_xfer;

	m->context = ts;

	if (request_irq(spi->irq, sx8652_irq, IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, ts)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}
	dev_info(&spi->dev, "touchscreen, irq %d\n", spi->irq);


	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr_group;
	
	SxGpioResetTouch();
	udelay(150);
	SxGpioConfigReg(SX8652_CMD_WRITEREG|SX8652_REG_CTRL0, 0x0b);//0x0E);
	SxGpioConfigReg(SX8652_CMD_WRITEREG|SX8652_REG_CTRL1, 0x25);//0x35);
 	SxGpioConfigReg(SX8652_CMD_WRITEREG|SX8652_REG_CTRL2, 0x00);
  	SxGpioConfigReg(SX8652_CMD_WRITEREG|SX8652_REG_CHANMSK, 0xC0);

	SxGpioSetMode(SX8652_CMD_PENTRG);
	SxGpioReadSpiReg();
//	while(1);
	return 0;

 err_remove_attr_group:
	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);
	free_irq(spi->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}


/* Must be called with ts->lock held */
static void sx8652_disable(struct sx8652 *ts)
{
	if (ts->disabled)
		return;
	SxGpioSpiStop();
	disable_irq(ts->spi->irq);
	ts->disabled = 1;

	/* if timer is running, wait for it to finish */
	while (ts->pen_down) {
		msleep(5);
	}

}

static int sx8652_suspend(struct spi_device *spi, pm_message_t message)
{
	struct sx8652 *ts = dev_get_drvdata(&spi->dev);

	mutex_lock(&ts->mutex);
	sx8652_disable(ts);
	mutex_unlock(&ts->mutex);

	return 0;
}

static int __devexit sx8652_remove(struct spi_device *spi)
{
	struct sx8652		*ts = dev_get_drvdata(&spi->dev);

#if 0//#if defined(CONFIG_HWMON) || defined(CONFIG_HWMON_MODULE)
	sx8652_hwmon_unregister(spi, ts);
#endif
	input_unregister_device(ts->input);

	sx8652_suspend(spi, PMSG_SUSPEND);

	sysfs_remove_group(&spi->dev.kobj, &sx8652_attr_group);

	free_irq(ts->spi->irq, ts);
	kfree(ts);

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	return 0;
}

/* Must be called with ts->lock held */
static void sx8652_enable(struct sx8652 *ts)
{
	if (!ts->disabled)
		return;

	ts->disabled = 0;
	enable_irq(ts->spi->irq);
}

static int sx8652_resume(struct spi_device *spi)
{
	struct sx8652 *ts = dev_get_drvdata(&spi->dev);

	mutex_lock(&ts->mutex);
	sx8652_enable(ts);
	mutex_unlock(&ts->mutex);

	return 0;
}

static struct spi_driver sx8652_driver = {
	.driver = {
		.name	= "sx8652",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= sx8652_probe,
	.remove		= __devexit_p(sx8652_remove),
	.suspend	= sx8652_suspend,
	.resume		= sx8652_resume,
};

static int __init sx8652_init(void)
{
	printk("%s() \n\n", __func__);	
	return spi_register_driver(&sx8652_driver);
}
module_init(sx8652_init);

static void __exit sx8652_exit(void)
{
	printk("%s() \n\n", __func__);
	spi_unregister_driver(&sx8652_driver);
}
module_exit(sx8652_exit);

MODULE_DESCRIPTION("SX8652 TouchScreen Driver");
MODULE_LICENSE("GPL");
