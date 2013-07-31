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

#include <asm/io.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>

#include <linux/kthread.h>  /* thread */
#include <linux/delay.h>    /* msleep_interruptible */

#define SAMKOON_VOLTAGE_HIGH  (1 << 0)

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define CPUREFRESH_PIN  GPIO_TO_PIN(2, 1)


//#undef samkoon_dbg(fmt,arg...)  
#if 1
#define samkoon_dbg(fmt,arg...)  
#else
#define samkoon_dbg(fmt,arg...)      printk(fmt,##arg)
#endif

struct samkoon_gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
    unsigned int state_val;
	int irq;
	struct work_struct work;
    struct task_struct *poll_task;
};


static int gJack_disconnect_count = 0;

static void samkoon_gpio_switch_work(struct work_struct *work)
{
	struct samkoon_gpio_switch_data	*data =
		container_of(work, struct samkoon_gpio_switch_data, work);

	switch_set_state(&data->sdev, data->state_val);
}

static ssize_t switch_samkoon_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct samkoon_gpio_switch_data	*switch_data =
		container_of(sdev, struct samkoon_gpio_switch_data, sdev);

    return sprintf(buf, "%d\n", switch_data->state_val);
}

void SetRefreshState(void)
{
	gJack_disconnect_count=1;
}

int GetRefreshState(void)
{
	return gJack_disconnect_count;
}
static int samkoon_gpio_thread(void* _switch_data)
{
    struct samkoon_gpio_switch_data *switch_data = (struct samkoon_gpio_switch_data *)_switch_data;
    unsigned int state = 0;
    unsigned int state_old = 0;
   unsigned long ret;
    unsigned int change = 0;

    samkoon_dbg("%s() in...\n\n", __func__);


    /* GPIO setting for voltage dectection */
	ret =gpio_request(CPUREFRESH_PIN, "CPUREFRESH_PIN");
	gpio_direction_output(CPUREFRESH_PIN, 1);

    while(!kthread_should_stop()) 
    {
        /* polling time is 2ms */
        msleep_interruptible(100);

	 state = GetRefreshState();
	 samkoon_dbg("%s() state =%d\n", __func__,state);
	 if(state==1)
	 {
		change=(change+1)%2;
		if(change)
			gpio_direction_output(CPUREFRESH_PIN, 1);
		else
			gpio_direction_output(CPUREFRESH_PIN, 0);
	 }

      }
    samkoon_dbg("%s() out...\n\n", __func__);

    return 0;
}

static int samkoon_gpio_switch_probe(struct platform_device *pdev)
{
	struct samkoon_gpio_switch_data *switch_data;
	int ret = 0;

    samkoon_dbg("gpio_switch_probe()...in \n\n");

	switch_data = kzalloc(sizeof(struct samkoon_gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

    switch_data->sdev.name = "samkoon_hmi";
    switch_data->state_val = 0;
    switch_data->name_on = "h2w_name_on";
    switch_data->name_off = "h2w_name_off";
    switch_data->state_on = "2";
    switch_data->state_off = "1";
	switch_data->sdev.print_state = switch_samkoon_gpio_print_state;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, samkoon_gpio_switch_work);

    switch_data->poll_task = (struct task_struct *)(-EINVAL);
    switch_data->poll_task = kthread_create(samkoon_gpio_thread, switch_data, "samkoon-gpio-poll-thread");
    if (IS_ERR(switch_data->poll_task)) {
        printk("\nsamkoon-gpio-poll-thread create error: %p\n", switch_data->poll_task);
        kfree(switch_data);
        return (-EINVAL);
    }
    wake_up_process(switch_data->poll_task);

	/* Perform initial detection */
	samkoon_gpio_switch_work(&switch_data->work);

    samkoon_dbg("gpio_switch_probe()...out \n\n");

	return 0;

//err_request_irq:
//err_detect_irq_num_failed:
//err_set_gpio_input:
	gpio_free(switch_data->gpio);
//err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit samkoon_gpio_switch_remove(struct platform_device *pdev)
{
	struct samkoon_gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver samkoon_gpio_switch_driver = {
	.probe		= samkoon_gpio_switch_probe,
	.remove		= __devexit_p(samkoon_gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio-voltage-detect",
		.owner	= THIS_MODULE,
	},
};

static int __init samkoon_gpio_switch_init(void)
{
   samkoon_dbg("\n%s()...\n\n", __func__);
	return platform_driver_register(&samkoon_gpio_switch_driver);
}

static void __exit samkoon_gpio_switch_exit(void)
{
	platform_driver_unregister(&samkoon_gpio_switch_driver);
}

module_init(samkoon_gpio_switch_init);
module_exit(samkoon_gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
