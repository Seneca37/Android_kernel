/*
 * File:        drivers/char/skcommon.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>

#ifndef ON
#define ON 		1
#endif

#ifndef OFF
#define OFF 	0
#endif

static struct class *skcommon_dev_class;

#define DEV_NAME "sk_common"


#define SK_MAJOR_NUM 234
#define SK_MINOR_NUM 1

#define IOCTL_SET_BACKLIGHTON   	_IO(SK_MAJOR_NUM, 100)
#define IOCTL_SET_BACKLIGHTOFF		_IO(SK_MAJOR_NUM, 101)
#define IOCTL_GET_SKMODE0   			_IO(SK_MAJOR_NUM, 102)
#define IOCTL_GET_SKMODE1   			_IO(SK_MAJOR_NUM, 103)
#define IOCTL_GET_SKMODE2   			_IO(SK_MAJOR_NUM, 104)

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define SK_BACKLIGHT_PIN  GPIO_TO_PIN(3, 19)
#define SK_MODE0_PIN  GPIO_TO_PIN(3, 21)
#define SK_MODE1_PIN  GPIO_TO_PIN(3, 16)
#define SK_MODE2_PIN  GPIO_TO_PIN(3, 15)


typedef struct {
	int module;
	int TMP1;
	int TMP2;
} sk_common_info_t;


void Sk_Backlight_On(void)
{
	gpio_direction_output(SK_BACKLIGHT_PIN, 1);
}

void Sk_Backlight_Off(void)
{
	gpio_direction_output(SK_BACKLIGHT_PIN, 0);
}

static int sk_common_dev_open(struct inode *inode, struct file *file)
{
		printk("sk_common_dev_open\n");
		return 0;
}

static int sk_common_dev_release(struct inode *inode, struct file *file)
{
		printk("sk_common_dev_release\n");
		return 0;
}

static int sk_common_get_info(sk_common_info_t* arg,int module_t)
{
	sk_common_info_t *info_t;
//	int module_t;
	
	info_t = (sk_common_info_t *)arg;
	copy_from_user(info_t, (sk_common_info_t *)arg, sizeof(sk_common_info_t));

//	module_t = 0;

	printk("Samkoon module[0x%x]\n", module_t);

	info_t->module = module_t;

	copy_to_user((sk_common_info_t *)arg, info_t, sizeof(sk_common_info_t));

	return 0;
}


int sk_common_dev_ioctl(struct file *file,unsigned int cmd, void *arg)
{
    	int *parm1;
    	int mode0,mode1,mode2,module;

    	memset(&parm1, 0, sizeof(int));
 //   	printk("Samkoon sk_common_dev_ioctl cmd[%d] arg[%d]\n", cmd, arg);

			mode0 = gpio_get_value(SK_MODE0_PIN);
			mode1 = gpio_get_value(SK_MODE1_PIN);
			mode2 = gpio_get_value(SK_MODE2_PIN);

		
    	switch(cmd)
    	{
	    case IOCTL_SET_BACKLIGHTON:			
//	    		printk("Samkoon IOCTL_SET_BACKLIGHTON cmd[%d]\n", cmd);
					Sk_Backlight_On();
					break;
					
	    case IOCTL_SET_BACKLIGHTOFF:			
//	    		printk("Samkoon IOCTL_SET_BACKLIGHTOFF cmd[%d]\n", cmd);
					Sk_Backlight_Off();
					break;
	    
	    case IOCTL_GET_SKMODE0:
	    		printk("Samkoon IOCTL_GET_SKMODE0 cmd[%d]\n", cmd);
//					if((mode0==1)&&(mode1==0)&&(mode2==0))
					if(mode0==1)
						module=1;
					else
						module=0;
	    		sk_common_get_info((sk_common_info_t*)arg,module);
	    		break;

	    case IOCTL_GET_SKMODE1:
	    		printk("Samkoon IOCTL_GET_SKMODE1 cmd[%d]\n", cmd);
//					if((mode0==0)&&(mode1==1)&&(mode2==0))
					if(mode1==1)
						module=1;
					else
						module=0;
	    		sk_common_get_info((sk_common_info_t*)arg,module);
	    		break;

	    case IOCTL_GET_SKMODE2:
	    		printk("Samkoon IOCTL_GET_SKMODE2 cmd[%d]\n", cmd);
//					if((mode0==0)&&(mode1==0)&&(mode2==1))
					if(mode2==1)
						module=1;
					else
						module=0;
	    		sk_common_get_info((sk_common_info_t*)arg,module);
	    		break;
      
      	default :
    			printk("Samkoon sk_common_dev_ioctl cmd[%d]\n", cmd);
	      	break;
    }

    return 0;
}

struct file_operations sk_common_dev_ops = {
    .owner      = THIS_MODULE,
    .unlocked_ioctl = sk_common_dev_ioctl,
    .open       = sk_common_dev_open,
    .release    = sk_common_dev_release,
};


int init_module(void)
{
    	int ret;

    	printk("Samkoon init_module\n");
    	ret = register_chrdev(SK_MAJOR_NUM, DEV_NAME, &sk_common_dev_ops);
    	skcommon_dev_class = class_create(THIS_MODULE, DEV_NAME);
    	device_create(skcommon_dev_class, NULL, MKDEV(SK_MAJOR_NUM, SK_MINOR_NUM), NULL, DEV_NAME);
	
			gpio_request(SK_MODE0_PIN, "SK_MODE0_PIN");
			gpio_request(SK_MODE1_PIN, "SK_MODE1_PIN");
			gpio_request(SK_MODE2_PIN, "SK_MODE2_PIN");

			if(ret < 0){
        printk("Samkoon [%d]fail to register the character device\n", ret);
        return ret;
    	}

    return 0;
}

void cleanup_module(void)
{
	printk("Samkoon cleanup_module\n");
	unregister_chrdev(SK_MAJOR_NUM, DEV_NAME);
}


late_initcall(init_module);
module_exit(cleanup_module);

MODULE_AUTHOR("Samkoon Inc.Robert Lee");
MODULE_DESCRIPTION("sk_common_dev");
MODULE_LICENSE("GPL");

