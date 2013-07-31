/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>    /* msleep_interruptible */

#include <linux/kthread.h>  /* thread */
#include "../../../arch/arm/mach-omap2/include/mach/samkoonhmi.h"

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

static struct task_struct *poll_task;

#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define BACKLIGHT_PIN  GPIO_TO_PIN(3, 19)


extern void powersave(void); 
extern void powersave_open(void); 
extern void powersave_init(void); 


static int powersave_thread()
{
    printk("%s() in...\n\n", __func__);

    while(!kthread_should_stop()) 
    {
        /* polling time is 1000ms */
        msleep_interruptible(5000);
	powersave_open();
    }
    printk("%s() out...\n\n", __func__);

    return 0;
}
extern int omap_irq_pending(void);

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	int index;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
		
//	printk("%s() state1 %d \n\n", __func__,state);	
 #ifdef LCD_035TINC
//	printk("%s() LCD_035TINC %d \n\n", __func__,state);	
//	msleep_interruptible(1);
	mdelay(1);
#else
//	msleep_interruptible(5);
	mdelay(5);
#endif
	state = gpio_get_value(data->gpio);
	
//	printk("%s() state2 %d \n\n", __func__,state);	
	if(state==0)
	{
	//	printk("turn off backlight\n", __func__);
	//	local_irq_disable();
	//	local_fiq_disable();
	//	gpio_direction_output(BACKLIGHT_PIN, 0);
	//	omap_irq_pending();
		powersave();
	}
//	printk("%s() state3 %d \n\n", __func__,state);
	switch_set_state(&data->sdev, state);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;
	
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;
	printk("registewr %s() \n\n", __func__);

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	enable_irq(switch_data->irq);

	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, gpio_irq_handler,
			  IRQF_TRIGGER_FALLING/*IRQF_TRIGGER_HIGH*/, pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	powersave_init();
/*	
	poll_task = (struct task_struct *)(-EINVAL);
  poll_task = kthread_create(powersave_thread, NULL, "powersave-thread");
  if (IS_ERR(poll_task)) {
      printk("\powersave-thread create error: %p\n", poll_task);
  		ret = PTR_ERR(poll_task);
			poll_task = NULL;
      kfree(switch_data);
     return ret;
   }
  wake_up_process(poll_task);
*/	
	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
