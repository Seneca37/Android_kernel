/*====================================================================================
 * Copyright (C) 2011 Multek display, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>

#include <linux/i2c/mxt224.h>
#include "../../../arch/arm/mach-omap2/include/mach/samkoonhmi.h"


#define TP_TIMER_TIMEOUT    (((HZ)*10)/100)     /* 100 ms */

#ifdef	LCD_070INC
#define TP_XSIZE_MAX        800
#define TP_YSIZE_MAX        480
#elif defined(LCD_102INC)
#define TP_XSIZE_MAX        800
#define TP_YSIZE_MAX        480
#elif defined(LCD_035INC)
#define TP_XSIZE_MAX        320
#define TP_YSIZE_MAX        240
#elif defined(LCD_040INC)
#define TP_XSIZE_MAX        320
#define TP_YSIZE_MAX        240
#elif defined(LCD_043INC)||defined(LCD_043INC_2UART)
#define TP_XSIZE_MAX        480
#define TP_YSIZE_MAX        272
#elif defined(LCD_050INC)
#define TP_XSIZE_MAX        480
#define TP_YSIZE_MAX        272
#elif defined(LCD_057INC)
#define TP_XSIZE_MAX        640
#define TP_YSIZE_MAX        480
#endif


#define TP_READBUFF_SIZE    20

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#ifdef LCD_ALLINONE_050INC
#define SXRST_PIN  GPIO_TO_PIN(0, 12)
#else
#define SXRST_PIN  GPIO_TO_PIN(0, 5)
#endif


struct tsxmt224_dev {
    struct i2c_client   *client;
    struct input_dev    *input;
    char                phys[32];
    unsigned int        irq;
    int                 point_pos;
    int                 point_len;
    struct work_struct  work;
    struct timer_list   timer;
	struct workqueue_struct *workqueue;
};

static int calibration_env(struct i2c_client *client);

static ssize_t tsxmt224_cal_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Do touch panel hardware"
                "calibration by write \"calibration\"\n");
}

static ssize_t tsxmt224_cal_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
    struct tsxmt224_dev *priv = dev_get_drvdata(dev);
    struct i2c_client *client = priv->client;
    const char *str = "calibration";

    if (strncmp(buf, str, strlen(str)) == 0) {
        calibration_env(client);
        return count;
    } else
        return -EINVAL;
}

static DEVICE_ATTR(calibrate, 0664, tsxmt224_cal_show, tsxmt224_cal_store);

static struct attribute *tsxmt224_attributs[] = {
    &dev_attr_calibrate.attr,
    NULL
};

static const struct attribute_group tsxmt224_attr_group = {
    .attrs = tsxmt224_attributs,
};

/* Mass Production Calibration function.
 *
 * The touch panel is calibrated with the system environmnet
 *
 * Notes:
 * - Only suggested to be used for mass production purpose.
 * - When use this command, it very important to avoid any touch
     object surrounding the whole system
 * - need some time to execute, it takes about 5 seconds to be
     finished.
 */
static int calibration_env(struct i2c_client *client)
{
    dev_info(&client->dev, "calibration started\n");
    dev_info(&client->dev, "calibration finished\n");
	
    return 0;
}

#define tsxmt224_PERLINE_DATAS     24
#define tsxmt224_PERLINE_BUFFSIZE  (tsxmt224_PERLINE_DATAS * 3 + 8)
void tsxmt224_print_buffer(u8 *p_buffer, int len)
{
    int         i, lineLen, dispLen;
    char        prn_buff[tsxmt224_PERLINE_BUFFSIZE];
    char        *p_prn;
    u8          *p_buff;
	
    sprintf(prn_buff, "   ");
    dispLen = len;
    p_buff = p_buffer;
    while (dispLen > 0) {
        lineLen = dispLen;
        if (lineLen > tsxmt224_PERLINE_DATAS) 
            lineLen = tsxmt224_PERLINE_DATAS;
		
        p_prn = prn_buff + 3;
        for (i = 0;  i < lineLen;  i++) {
            sprintf(p_prn, " %02x", *p_buff++);
            p_prn += 3;
        }
        printk(KERN_INFO "%s\n", prn_buff);
        dispLen -= lineLen;
    }
}

//#define I2C_READPACKET_TRACEADDR
//#define I2C_READPACKET_TRACEDATA
static int tsxmt224_i2c_read_packet(struct i2c_client *client, 
                    char *p_data, int read_addr, int data_len)
{
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	struct i2c_msg msg[2] = { { client->addr, client->flags, 2, msgbuf0 },
	                          { client->addr, client->flags | I2C_M_RD, 0, msgbuf1 }
	                        };
	int status, i;
	
	msgbuf0[0] = read_addr & 0xff;
	msgbuf0[1] = (read_addr >> 8) & 0xff;
	
    msg[1].len = data_len;
    if (msg[1].len > I2C_SMBUS_BLOCK_MAX) msg[1].len = I2C_SMBUS_BLOCK_MAX;
	#ifdef I2C_READPACKET_TRACEADDR
	printk(KERN_INFO "addr=%02x, buf0[0]=0x%02x, buf0[1]=0x%02x, flags=0x%04x, len=%d\n",
	        client->addr, msgbuf0[0], msgbuf0[1], msg[1].flags, msg[1].len);
	#endif
	mdelay(10);

	status = i2c_transfer(client->adapter, msg, 2);
	if (status < 0) {
	    memset(p_data, 0xff, data_len);
	}
	else {
        status = data_len;
        for (i = 0;  i < msg[1].len;  i++) {
            p_data[i] = msgbuf1[i];
        }
    }
	#ifdef I2C_READPACKET_TRACEDATA
    tsxmt224_print_buffer(p_data, data_len);
	#endif
    
	return status;
}

#define IICTS_TOUCHNUM_MAXVAL	5
typedef struct _TTSPOINT {
	u8          fingerID;
	u8          stat;
	u16         updated;
	u16         x;
	u16         y;
}	TTSPOINT, *PTSPOINT;

typedef struct _TTSDEV {
    int         hasPressed;
    int         touchNum;
    int         touchUpNum;
    
    TTSPOINT    points[IICTS_TOUCHNUM_MAXVAL];
}	TTSDEV, *PTSDEV;

static TTSDEV   IICTS_VAR_dev;

void tsxmt224_devInit(PTSDEV pDev)
{
	register int    i;
	
	pDev->hasPressed = 0;
	pDev->touchNum = 0;
	pDev->touchUpNum = 0;
	for (i = 0;  i < IICTS_TOUCHNUM_MAXVAL;  i++) {
		pDev->points[i].fingerID = 0;
		pDev->points[i].stat = 0x20;
		pDev->points[i].updated = 0;
		pDev->points[i].x = 0;
		pDev->points[i].y = 0;
	}
}

int tsxmt224_devIsEqual(PTSDEV cmp1, PTSDEV cmp2)
{
	int     i, mov_pos;
	
	if (cmp1->hasPressed != cmp2->hasPressed) return 0;
	if (cmp1->touchNum != cmp2->touchNum) return 0;
	for (i = 0;  i < cmp1->touchNum;  i++) {
		if (cmp1->points[i].fingerID != cmp2->points[i].fingerID)
			return 0;
        mov_pos = abs(cmp1->points[i].x - cmp2->points[i].x);
        mov_pos += abs(cmp1->points[i].y - cmp2->points[i].y);
        if (mov_pos >= 3) return 0;
    }
	
	return 1;
}

TTSDEV      newPos;

void tsxmt224_devUpdateNew(void)
{
	register int    i;
	
	for (i = 0;  i < IICTS_TOUCHNUM_MAXVAL;  i++) {
		if ((0 == (newPos.points[i]).updated)  \
			&& (0x20 != ((IICTS_VAR_dev.points[i]).stat & 0x20)) ){
			(newPos.points[i]).updated = 1;
			(newPos.points[i]).fingerID = (IICTS_VAR_dev.points[i]).fingerID;
			(newPos.points[i]).stat = (IICTS_VAR_dev.points[i]).stat;
			(newPos.points[i]).x = (IICTS_VAR_dev.points[i]).x;
			(newPos.points[i]).y = (IICTS_VAR_dev.points[i]).y;
			(newPos.touchNum)++;
		}
	}
}

//#define TSXMT224_DEVSNDEVT_TRACEPOS
//#define TSXMT224_DEVSNDEVT_TRACEBTN
int tsxmt224_devSendEvent(struct input_dev *input)
{
	register int    i, single_id;

    single_id = -1;
    for (i = 0;  i < IICTS_TOUCHNUM_MAXVAL;  i++) {
        if ((newPos.points[i]).updated) {
            (newPos.points[i]).updated = 0;
            #ifdef TSXMT224_DEVSNDEVT_TRACEPOS
            printk(KERN_INFO "Point(%x): %-2d %02x %5d %5d\n", i,
                    (newPos.points[i]).fingerID, (newPos.points[i]).stat,
                    (newPos.points[i]).x, (newPos.points[i]).y);
            #endif
			
            if (0x20 != ((newPos.points[i]).stat & 0x20)) {
                if (-1 == single_id) single_id = i;
                input_event(input, EV_ABS, ABS_MT_POSITION_X, (newPos.points[i]).x);
                input_event(input, EV_ABS, ABS_MT_POSITION_Y, (newPos.points[i]).y);
                input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
                input_mt_sync(input);
            }
            else {
                input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(input);
            }
        }
    }
    
    if (-1 == single_id) {
        #ifdef TSXMT224_DEVSNDEVT_TRACEBTN
        printk(KERN_INFO "report BTN_TOUCH(0)\n");
        #endif
    	input_report_key(input, BTN_TOUCH, 0);
    }
    else {
        #ifdef TSXMT224_DEVSNDEVT_TRACEBTN
        printk(KERN_INFO "report BTN_TOUCH(1)\n");
        #endif
    	input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_X, (newPos.points[single_id]).x);
		input_report_abs(input, ABS_Y, (newPos.points[single_id]).y);
    }
	
    input_sync(input);
    
    newPos.touchUpNum = 0;
    IICTS_VAR_dev = newPos;
    tsxmt224_devInit(&newPos);
    
    return 0;
}

//#define TSXMT224_CHKDATA_TRACEPOS
int tsxmt224_checkData(u8 *p_data, struct input_dev *input)
{
    int         i;
    u8          id, stat;
    u16         x, y;
    
    id = p_data[0];
    stat = p_data[1];
    x = ((u16)(p_data[2]) << 2) + ((p_data[4] & 0xf0) >> 6);
    y = ((u16)(p_data[3]) << 2) + ((p_data[4] & 0x0f) >> 2);
#if 0		
    x = ((int)(x) * 799) / 1023;
    y = ((int)(y) * 479) / 1023;
#else
//    x = ((int)(x) * (TP_XSIZE_MAX-1)) / 1023;
//    y = ((int)(y) * (TP_YSIZE_MAX-1)) / 1023;
	x = TP_XSIZE_MAX - (x);
	y = TP_YSIZE_MAX - (y);
#endif
	i = (int)(id - 2);
	#ifdef TSXMT224_CHKDATA_TRACEPOS
    printk(KERN_INFO "checkData: %-2d %02x %5d %5d\n", id, stat, x, y);
	#endif
    
    if (0x20 == (stat & 0x20)) {
        if ((newPos.points[i]).updated) {
            if (IICTS_VAR_dev.touchNum > newPos.touchNum) {
                tsxmt224_devUpdateNew();
            }
            tsxmt224_devSendEvent(input);
        }
    	
    	IICTS_VAR_dev.touchNum--;
		(newPos.points[i]).fingerID = id;
		(newPos.points[i]).stat = stat;
		(newPos.points[i]).x = x;
		(newPos.points[i]).y = y;
    	if (newPos.touchNum < 1) {
    		newPos.touchUpNum++;
			(newPos.points[i]).updated = 1;
    	}
    	
    	if (IICTS_VAR_dev.touchNum == newPos.touchNum) {
    		tsxmt224_devSendEvent(input);
    	}
    } 
    else {
    	if ((newPos.points[i]).updated) {
    		if (IICTS_VAR_dev.touchNum > newPos.touchNum) {
    			tsxmt224_devUpdateNew();
    		}
    		tsxmt224_devSendEvent(input);
    	}
    	
    	newPos.hasPressed = 1;
    	(newPos.touchNum)++;
		(newPos.points[i]).updated = 1;
		(newPos.points[i]).fingerID = id;
		(newPos.points[i]).stat = stat;
		(newPos.points[i]).x = x;
		(newPos.points[i]).y = y;
    }
    
	return 0;
}

//#define TSXMT224_WORK_TRACEBGN
//#define TSXMT224_WORK_TRACEEND
#define TSXMT224_WORK_TRACEERR
//#define TSXMT224_WORK_TRACEOBJS
//#define TSXMT224_WORK_TRACET5ERR
static void tsxmt224_work(struct work_struct *work)
{
    struct tsxmt224_dev *tsmultek224 = container_of(work, struct tsxmt224_dev, work);
    struct i2c_client *client = tsmultek224->client;
    struct multek_ts_platform_data *pdata = client->dev.platform_data;
    struct input_dev *input = tsmultek224->input;
    int                     read_pos, read_len;
    char                    data[TP_READBUFF_SIZE];

    #ifdef TSXMT224_WORK_TRACEBGN
    printk(KERN_INFO "tsxmt224_work::entry\n");
    #endif
    if (tsmultek224->point_len < 1) {
        int     i, obj_cnt;
        
        read_pos = 0;
        read_len = 7;
        tsxmt224_i2c_read_packet(client, (char *)(data), read_pos, read_len);
        if ( !((0x81 == data[0]) && (0x01 == data[1])) ) {
            #ifdef TSXMT224_WORK_TRACEERR
            printk(KERN_INFO "tsxmt224_work:HeadDataError(addr=0x%02x)\n",
                client->addr);
            tsxmt224_print_buffer(data, read_len);
            #endif
            goto tsxmt224_work_endedwork;
        }
    
        read_pos += read_len;
        read_len = 6;
        obj_cnt = data[6];
        for (i = 0;  i < obj_cnt;  i++) {
            tsxmt224_i2c_read_packet(client, (char *)(data), 
                    read_pos, read_len);
            #ifdef TSXMT224_WORK_TRACEOBJS
            printk(KERN_INFO "T%-2d %5d %3d %02x %02x\n", data[0],
                    (data[2] << 8) + data[1], data[3], data[4], data[5]);
            #endif
            if (0x05 == data[0]) {
                tsmultek224->point_pos = (data[2] << 8) + data[1];
                tsmultek224->point_len = data[3];
            }
            read_pos += read_len;
        }
        
        if (tsmultek224->point_len < 1) {
            #ifdef TSXMT224_WORK_TRACEERR
            printk(KERN_INFO "tsxmt224_work:can't find T5 object\n");
            #endif
            goto tsxmt224_work_endedwork;
        }
    }
    
    read_pos = tsmultek224->point_pos;
    read_len = tsmultek224->point_len;
    while (!pdata->hw_status()) {
        tsxmt224_i2c_read_packet(client, (char *)(data), 
                read_pos, read_len);
        if ((2 > data[0]) || (11 < data[0])) {
            #ifdef TSXMT224_WORK_TRACET5ERR
            printk(KERN_INFO "tsxmt224_work: error point location\n");
            tsxmt224_print_buffer(data, read_len);
            #endif
            continue;
        }
        tsxmt224_checkData(data, input);
    }
    
tsxmt224_work_endedwork:
    if (!tsmultek224->irq) {
        tsmultek224->timer.expires = jiffies + TP_TIMER_TIMEOUT;
        add_timer( &(tsmultek224->timer) );
    }
    #ifdef TSXMT224_WORK_TRACEEND
    printk(KERN_INFO "tsxmt224_work::exit\n");
    #endif
}

static irqreturn_t tsxmt224_irq(int irq, void *handle)
{
    struct tsxmt224_dev  *mXT224;
	
    mXT224 = (struct tsxmt224_dev *)(handle);
	queue_work(mXT224->workqueue, &mXT224->work);
    //schedule_work( &(mXT224->work) );
    
	return IRQ_HANDLED;
}

static void tsxmt224_timer(unsigned long data)
{
    struct tsxmt224_dev  *mXT224;
	
    mXT224 = (struct tsxmt224_dev *)(data);
	queue_work(mXT224->workqueue, &mXT224->work);
    //schedule_work( &(mXT224->work) );
}

#define TSXMT224_CHKCFG_TRACEERR
//#define TSXMT224_CHKCFG_TRACEOBJS
//#define TSXMT224_CHKCFG_TRACET5
static void tsxmt224_config_check(struct tsxmt224_dev *tsmultek224)
{
    struct i2c_client *client = tsmultek224->client;
    struct multek_ts_platform_data *pdata = client->dev.platform_data;
    int                 read_pos, read_len;
    int                 i, obj_cnt;
    unsigned char      data[TP_READBUFF_SIZE];
    
    tsmultek224->point_pos = 0;
    tsmultek224->point_len = 0;
    read_pos = 0;
    read_len = 7;
    tsxmt224_i2c_read_packet(client, (char *)(data), read_pos, read_len);
    if ( !((0x81 == data[0]) && (0x01 == data[1])) ) {
        #ifdef TSXMT224_CHKCFG_TRACEERR
        printk(KERN_INFO "tsxmt224_config_check:HeadDataError(addr=0x%02x)\n",
                client->addr);
        tsxmt224_print_buffer(data, read_len);
        #endif
        return;
    }
    
    read_pos += read_len;
    read_len = 6;
    obj_cnt = data[6];
    for (i = 0;  i < obj_cnt;  i++) {
        tsxmt224_i2c_read_packet(client, (char *)(data), 
                read_pos, read_len);
        #ifdef TSXMT224_CHKCFG_TRACEOBJS
        printk(KERN_INFO "T%-2d %5d %3d %02x %02x\n", data[0],
                (data[2] << 8) + data[1], data[3], data[4], data[5]);
        #endif
        if (0x05 == data[0]) {
            tsmultek224->point_pos = (data[2] << 8) + data[1];
            tsmultek224->point_len = data[3];
        }
        read_pos += read_len;
    }
    if (tsmultek224->point_len < 1) {
        #ifdef TSXMT224_CHKCFG_TRACEERR
        printk(KERN_INFO "tsxmt224_config_check:can't find T5 object\n");
        #endif
        return;
    }
    
    read_pos = tsmultek224->point_pos;
    read_len = tsmultek224->point_len;
    while (!pdata->hw_status()) {
        tsxmt224_i2c_read_packet(client, (char *)(data), 
                read_pos, read_len);
        #ifdef TSXMT224_CHKCFG_TRACET5
        tsxmt224_print_buffer(data, read_len);
        #endif
    }
}

static void Sx8650_ResetTouch()
{
	gpio_direction_output(SXRST_PIN, 1);
	mdelay(100);
	gpio_direction_output(SXRST_PIN, 0);
	mdelay(100);
	gpio_direction_output(SXRST_PIN, 1);

}

#define TSXMT224_PROBE_TRACECLI
#define TSXMT224_PROBE_TRACENAME
static int __devinit tsxmt224_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int     ret;    //, xmax, ymax;
    struct tsxmt224_dev *tsmultek224;
    struct input_dev    *input_dev;
    struct multek_ts_platform_data *pdata = client->dev.platform_data;

    #ifdef TSXMT224_PROBE_TRACECLI
    printk( KERN_INFO "tsxmt224_probe: i2c_client(addr=%02x, irq=%d)\n",
            client->addr, client->irq );
    #endif
    tsxmt224_devInit(&IICTS_VAR_dev);
    tsxmt224_devInit(&newPos);
    if (!i2c_check_functionality(client->adapter,
                            I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
        dev_err(&client->dev, "I2C don't support enough function");
        return -EIO;
    }

	if (!pdata || !pdata->hw_status) {
		dev_err(&client->dev, "No hw status function!\n");
		return -EIO;
	}

	Sx8650_ResetTouch();

    tsmultek224 = kzalloc(sizeof(struct tsxmt224_dev), GFP_KERNEL);
    if (!tsmultek224)
        return -ENOMEM;

    input_dev = input_allocate_device();
    if (!input_dev) {
        ret = -ENOMEM;
        goto err_free_mem;
    }

    tsmultek224->client = client;
    tsmultek224->irq = client->irq;
    tsmultek224->point_pos = 0;
    tsmultek224->point_len = 0;
    tsmultek224->input = input_dev;
	tsmultek224->workqueue = create_singlethread_workqueue("mxt224");
    INIT_WORK(&tsmultek224->work, tsxmt224_work);
    if (NULL == tsmultek224->workqueue) {
		dev_err(&client->dev, "couldn't create workqueue\n");
		ret = -ENOMEM;
		goto err_free_dev;
    }

    snprintf(tsmultek224->phys, sizeof(tsmultek224->phys), "i2c-ts");  //, dev_name(&client->dev)
    input_dev->name = "Multek-Touchscreen";
    input_dev->phys = tsmultek224->phys;
    input_dev->id.bustype = BUS_I2C;
    #ifdef TSXMT224_PROBE_TRACENAME
    printk(KERN_INFO "name: %s,   phys: %s\n", 
            input_dev->name, input_dev->phys);
    #endif
    input_set_drvdata(input_dev, tsmultek224);

 	//__set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
	//set_bit(KEY_HOME, input_dev->keybit);
	//set_bit(KEY_MENU, input_dev->keybit);
	//set_bit(KEY_BACK, input_dev->keybit);
	//set_bit(KEY_SEARCH, input_dev->keybit);

	/* For single touch */
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_dev->keybit[BIT_WORD (BTN_TOUCH)]= BIT_MASK (BTN_TOUCH);
    input_set_abs_params(input_dev, ABS_X, 0, TP_XSIZE_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, TP_YSIZE_MAX, 0, 0);

	/* For multi touch */
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TP_XSIZE_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TP_YSIZE_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);

    ret = input_register_device(input_dev);
    if (ret)
        goto err_free_wq;

    i2c_set_clientdata(client, tsmultek224);

    ret = sysfs_create_group(&client->dev.kobj, &tsxmt224_attr_group);
    if (ret)
        goto err_free_wq;
    
    tsxmt224_config_check(tsmultek224);

    if (tsmultek224->irq) {
	    // set irq type to edge falling
//        set_irq_type(tsmultek224->irq, IRQF_TRIGGER_FALLING);
        ret = request_irq(tsmultek224->irq, tsxmt224_irq, IRQF_TRIGGER_FALLING,
                client->dev.driver->name, tsmultek224);
        if (ret < 0) {
            dev_err(&client->dev, "failed to register irq %d!\n",
                tsmultek224->irq);
            goto err_unreg_dev;
        }
    }
    else {
        printk(KERN_INFO "init touchscreen timer\n");
        init_timer(&(tsmultek224->timer));
        (tsmultek224->timer).function = tsxmt224_timer;
        (tsmultek224->timer).data = (unsigned long)(tsmultek224);
        //(tsmultek224->timer).expires = jiffies + TP_TIMER_TIMEOUT;
        (tsmultek224->timer).expires = jiffies + (HZ) * 15;	//delay 5s to start readtimer
        add_timer( &(tsmultek224->timer) );
    }

	return 0;
	
err_unreg_dev:
	input_unregister_device(input_dev);
err_free_wq:
	destroy_workqueue(tsmultek224->workqueue);
err_free_dev:
	input_free_device(input_dev);
err_free_mem:
	kfree(tsmultek224);
	
	return ret;
}

static int __devexit tsxmt224_remove(struct i2c_client *client)
{
	struct tsxmt224_dev *tsmultek224 = i2c_get_clientdata(client);
	
    del_timer( &(tsmultek224->timer) );
	cancel_work_sync(&tsmultek224->work);
	destroy_workqueue(tsmultek224->workqueue);
	input_unregister_device(tsmultek224->input);
	input_free_device(tsmultek224->input);
	kfree(tsmultek224);

	return 0;
}

static const struct i2c_device_id tsxmt224_idtable[] = {
	{"multek_ts", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tsxmt224_idtable);

static struct i2c_driver tsxmt224_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "multek_ts",
		   },
	.id_table = tsxmt224_idtable,
	.probe = tsxmt224_probe,
	.remove = __devexit_p(tsxmt224_remove),
};

static int __init tsxmt224_init(void)
{
	return i2c_add_driver(&tsxmt224_driver);
}

static void __exit tsxmt224_exit(void)
{
	i2c_del_driver(&tsxmt224_driver);
}

module_init(tsxmt224_init);
module_exit(tsxmt224_exit);

MODULE_AUTHOR("Multek display, Inc.");
MODULE_DESCRIPTION("Atmel mXT224 multitouch driver");
MODULE_LICENSE("GPL");
