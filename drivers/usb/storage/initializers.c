/* Special Initializers for certain USB Mass Storage devices
 *
 * Current development and maintenance by:
 *   (c) 1999, 2000 Matthew Dharm (mdharm-usb@one-eyed-alien.net)
 *
 * This driver is based on the 'USB Mass Storage Class' document. This
 * describes in detail the protocol used to communicate with such
 * devices.  Clearly, the designers had SCSI and ATAPI commands in
 * mind when they created this document.  The commands are all very
 * similar to commands in the SCSI-II and ATAPI specifications.
 *
 * It is important to note that in a number of cases this class
 * exhibits class-specific exemptions from the USB specification.
 * Notably the usage of NAK, STALL and ACK differs from the norm, in
 * that they are used to communicate wait, failed and OK on commands.
 *
 * Also, for certain devices, the interrupt endpoint is used to convey
 * status of a command.
 *
 * Please see http://www.one-eyed-alien.net/~mdharm/linux-usb for more
 * information about this driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/errno.h>
#include <linux/slab.h>
#include "usb.h"
#include "initializers.h"
#include "debug.h"
#include "transport.h"

#define RESPONSE_LEN 1024
#undef US_DEBUGP
#define US_DEBUGP printk
/* This places the Shuttle/SCM USB<->SCSI bridge devices in multi-target
 * mode */
int usb_stor_euscsi_init(struct us_data *us)
{
	int result;

	US_DEBUGP("Attempting to init eUSCSI bridge...\n");
	us->iobuf[0] = 0x1;
	result = usb_stor_control_msg(us, us->send_ctrl_pipe,
			0x0C, USB_RECIP_INTERFACE | USB_TYPE_VENDOR,
			0x01, 0x0, us->iobuf, 0x1, 5000);
	US_DEBUGP("-- result is %d\n", result);

	return 0;
}
static int usb_stor_init_ignor(struct us_data *us)
{
	static struct timeval pre_time = {0, 0};
    static int pre_devnum = -1;
	struct timeval now_time;
    int ret;

    jiffies_to_timeval(jiffies, &now_time);

    if (us->ifnum > 1)
    {
        US_DEBUGP("USB device has been switched, ifnum %d \n", us->ifnum);
        ret = 1;
        goto done;
    }

    if ((pre_devnum == us->pusb_dev->devnum) &&
        (now_time.tv_sec - pre_time.tv_sec) < 2)
    {
        US_DEBUGP("USB device is switching.\n");
        ret = 1;
        goto done;
    }

    ret = 0;

done:
    pre_devnum = us->pusb_dev->devnum;
    jiffies_to_timeval(jiffies, &pre_time);
    return ret;

}
static int usb_stor_dev_init(struct us_data *us, char *usb_msg, int msg_len, int resp)
{

	char *buffer;
	int result;
	buffer = kzalloc(RESPONSE_LEN, GFP_KERNEL);
	if (buffer == NULL)
		return USB_STOR_TRANSPORT_ERROR;

	memcpy(buffer, usb_msg, msg_len);
	result = usb_stor_bulk_transfer_buf(us,
			us->send_bulk_pipe,
			buffer, msg_len, NULL);
	if (result != USB_STOR_XFER_GOOD) {
		result = USB_STOR_XFER_ERROR;
		goto out;
	}

	/* Some of the devices need to be asked for a response, but we don't
	 * care what that response is.
	 */
	 if(resp)
	 {
        US_DEBUGP("Receive USB device response. \n");
    	usb_stor_bulk_transfer_buf(us,
    			us->recv_bulk_pipe,
    			buffer, RESPONSE_LEN, NULL);
	 }

	/* Read the CSW */
	usb_stor_bulk_transfer_buf(us,
			us->recv_bulk_pipe,
			buffer, 13, NULL);

	result = USB_STOR_XFER_GOOD;

out:
	kfree(buffer);

	US_DEBUGP("USB device switch result: %d \n", result);
	return result;
}


/* This function is required to activate all four slots on the UCR-61S2B
 * flash reader */
int usb_stor_ucr61s2b_init(struct us_data *us)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap*) us->iobuf;
	struct bulk_cs_wrap *bcs = (struct bulk_cs_wrap*) us->iobuf;
	int res;
	unsigned int partial;
	static char init_string[] = "\xec\x0a\x06\x00$PCCHIPS";

	US_DEBUGP("Sending UCR-61S2B initialization packet...\n");

	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->Tag = 0;
	bcb->DataTransferLength = cpu_to_le32(0);
	bcb->Flags = bcb->Lun = 0;
	bcb->Length = sizeof(init_string) - 1;
	memset(bcb->CDB, 0, sizeof(bcb->CDB));
	memcpy(bcb->CDB, init_string, sizeof(init_string) - 1);

	res = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, bcb,
			US_BULK_CB_WRAP_LEN, &partial);
	if (res)
		return -EIO;

	US_DEBUGP("Getting status packet...\n");
	res = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe, bcs,
			US_BULK_CS_WRAP_LEN, &partial);
	if (res)
		return -EIO;

	return 0;
}

/* This places the HUAWEI E220 devices in multi-port mode */
int usb_stor_huawei_e220_init0000(struct us_data *us)
{
	int result;

	result = usb_stor_control_msg(us, us->send_ctrl_pipe,
				      USB_REQ_SET_FEATURE,
				      USB_TYPE_STANDARD | USB_RECIP_DEVICE,
				      0x01, 0x0, NULL, 0x0, 1000);
	US_DEBUGP("Huawei mode set result is %d\n", result);
	return 0;
}

int usb_stor_huawei_e1750_init(struct us_data *us)
{
	int result;
	int actual_length;
	
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
							 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);
	return result;
}

int usb_stor_huawei_e173_init(struct us_data *us)
{
	int result;
	int actual_length;
	
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
							 0x06, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);
	printk("----------------After usb_stor_huawei_e173_init function!-----------------\n");
	return result;
}


int usb_stor_huawei_e173_init2(struct us_data *us)
{
	int result;
	int actual_length;
	//MessageContent="55534243123456780000 0000 0000 00 11 0620000 001 0000000 0000000000000"
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
							 0x06, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);
	//printk("----------------After usb_stor_huawei_e173_init2 function!-----------------\n");
	return result;
}
/////////////sspemail 2012.3.13
int usb_stor_huawei_e353_init(struct us_data *us)
{
	int result;
	int actual_length;
	//MessageContent="55534243123456780000 0000 0000 0a 11 06 20 00 00 00 00 00 01 0 0000000000000"
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x11,
							 0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);
	//printk("----------------After usb_stor_huawei_e353_init function!-----------------\n");
	return result;
}
int usb_stor_zte_mu351_init(struct us_data *us)
{
	int result;
	int actual_length;
//MessageContent="55 53 42 43 12 34 56 78 00 00 00 00 00 00 06 1e 00 0000000000000000000000000000"
//MessageContent2="5553424312345679000000000000061b000000020000000000000000000000"
// mf110 mu531 mf112 mf190 mf192 
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1e,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);

	//printk("----------------After usb_stor_zte_mu351_init function!-----------------\n");
	return result;
}
int usb_stor_zte_mu351_init2(struct us_data *us)
{
	int result;
	int actual_length;
//MessageContent="55 53 42 43 12 34 56 78 00 00 00 00 00 00 06 1e 00 0000000000000000000000000000"
//MessageContent2="55 53 42 43 12 34 56 79 00000000000006 1b000000020000000000000000000000"
// mf110 mu531 mf112 mf190 mf192 mf652 
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x79,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1b,
							 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);

	//printk("----------------After usb_stor_zte_mu351_init2 function!-----------------\n");
	return result;
}

/////////////end

int usb_stor_zte_init(struct us_data *us)
{
	int result;
	int actual_length;

	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x08, 0xc9, 0x7e, 0x89,
							 0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x85,
							 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);

	//printk("----------------After usb_stor_zte_init function!-----------------\n");
	return result;
}

int usb_stor_zte2_init(struct us_data *us)
{
	int result;
	int actual_length;
//MessageContent="55 53 42 43 12 34 56 78000000000000061b000000020000000000000000000000"
	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1b,
							 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);

	//printk("----------------After usb_stor_zte2_init function!-----------------\n");
	return result;
}

int usb_stor_others_init(struct us_data *us)
{
	int result;
	int actual_length;

	char msg[] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x06,
							 0xf5, 0x04, 0x02, 0x52, 0x70, 0x00, 0x00, 0x00,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("others 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);

/*	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);
*/
	printk("----------------After usb_stor_other_init function!-----------------\n");
	return result;
}

int usb_stor_speedup_init(struct us_data *us)
{
	int result;
	int actual_length;

	//char *MessageContent="55 53 42 43 12345678000000000000061b000000020000000000000000000000";
	//55 53 42 43  70 28 81 8a  10 00 00 00  80 00 0a 42  02 40 01 00  00 00 00 10  00 00 00 00  00 00 00

	static unsigned char cmd[32] = {0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
							 0x10, 0x00, 0x00, 0x00, 0x80, 0x00, 0x0a, 0x42,
							 0x02, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10,
							 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe, cmd,
			31, &actual_length);

	//printk("----------------After usb_stor_speedup_init function!-----------------\n");
	return result;
}

///////////////////////////////
/* This places the HUAWEI E220 devices in multi-port mode */
#if 1
int usb_stor_huawei_e220_init(struct us_data *us)
{
	int result;

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	result = usb_stor_control_msg(us, us->send_ctrl_pipe,
				      USB_REQ_SET_FEATURE,
				      USB_TYPE_STANDARD | USB_RECIP_DEVICE,
				      0x01, 0x0, NULL, 0x0, 1000);
	US_DEBUGP("usb_control_msg performing result is %d\n", result);
	return (result ? 0 : -1);
}

int usb_stor_ZTE_AC580_init(struct us_data *us) // PID = 0x0026
{
int result = 0;
int act_len = 0;

unsigned char cmd[32] = {
0x55, 0x53, 0x42, 0x43, 0x28, 0x4e, 0xbc, 0x88,
0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x85,
0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
result = usb_stor_control_msg(us, us->send_ctrl_pipe,USB_REQ_SET_FEATURE,
USB_TYPE_STANDARD | USB_RECIP_DEVICE,0x01, 0x0, NULL, 0x0, 1000);
US_DEBUGP("usb_stor_control_msg performing result is %d\n", result);
printk("====AC580===>usb_stor_control_msg performing result is %d\n", result);

result |= usb_stor_bulk_transfer_buf (us, us->send_bulk_pipe, cmd, 31, &act_len);
US_DEBUGP("usb_stor_bulk_transfer_buf performing result12 is %d, transfer the actual length=%d\n", result, act_len);
printk("usb_stor_bulk_transfer_buf performing result is %d, transfer the actual length=%d\n", result, act_len);

return (result ? 0 : -ENODEV);
}

int usb_stor_Qualcomm_HD360_init(struct us_data *us) // PID = 0x0026
{
int result = 0;
int act_len = 0;

unsigned char cmd[32] = {
#if 1
0x55, 0x53, 0x42, 0x43, 0x68, 0x03, 0x2c, 0x88,
0x24, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x12,
0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#else
0x55, 0x53, 0x42, 0x43, 0x80, 0x32, 0x38, 0x8a,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
};
/*
result = usb_stor_control_msg(us, us->send_ctrl_pipe,USB_REQ_SET_FEATURE,
USB_TYPE_STANDARD | USB_RECIP_DEVICE,0x01, 0x0, NULL, 0x0, 1000);
US_DEBUGP("usb_stor_control_msg performing result is %d\n", result);
printk("====HD360===>usb_stor_control_msg performing result is %d\n", result);
*/
result |= usb_stor_bulk_transfer_buf (us, us->send_bulk_pipe, cmd, 31, &act_len);
US_DEBUGP("usb_stor_bulk_transfer_buf performing result12 is %d, transfer the actual length=%d\n", result, act_len);
printk("usb_stor_bulk_transfer_buf performing result is %d, transfer the actual length=%d\n", result, act_len);

return (result ? 0 : -ENODEV);
}
int usb_stor_zte_mu318_init(struct us_data *us)
{
	char msg[] = {
	  0x55, 0x53, 0x42, 0x43, 0x08, 0x80, 0xd1, 0x88,
	  0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x0a, 0x85,
	  0x01, 0x01, 0x01, 0x18, 0x01, 0x01, 0x01, 0x01,
	  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("ZTE MU318: %s", "DEVICE MODE SWITCH\n");
    return usb_stor_dev_init(us, msg, sizeof(msg), 1);
}

int usb_stor_eject_cd(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1b,
        0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("eject cd: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}

int usb_stor_eject_cd_resp(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1b,
        0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("eject cd: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 1);
    return 0;
}

int usb_stor_tenda_w6(struct us_data *us)
{
    char msg1[] = {
        0x55, 0x53, 0x42, 0x43, 0xa0, 0xdc, 0xf8, 0x88,
        0x80, 0x80, 0x00, 0x00, 0x80, 0x01, 0x0a, 0x66,
        0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    char msg2[] = {
        0x55, 0x53, 0x42, 0x43, 0xa0, 0xdc, 0xf8, 0x88,
        0x80, 0x80, 0x00, 0x00, 0x80, 0x01, 0x0a, 0x77,
        0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("Tenda W6: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg1, sizeof(msg1), 1);
    usb_stor_dev_init(us, msg2, sizeof(msg2), 1);

    return 0;
}

int usb_stor_huawei2_init(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("Huawei2 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}

int usb_stor_huawei3_init(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
        0x06, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("Huawei3 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}

int usb_stor_nokia1_init(struct us_data *us)
{
    printk("usb_stor_nokia1_init\n");
	char msg[] = {
#if 0	
	  0x55, 0x53, 0x42, 0x43, 0x08, 0x80, 0xd1, 0x88,
	  0x20, 0x00, 0x00, 0x00, 0x80, 0x00, 0x0a, 0x85,
	  0x01, 0x01, 0x01, 0x18, 0x01, 0x01, 0x01, 0x01,
	  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#else
/*
	  0x55, 0x53, 0x42, 0x43, 0x60, 0xEC, 0x3D, 0x07,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xFF,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//*/
	  0x55, 0x53, 0x42, 0x43, 0x60, 0xCC, 0xDB, 0x06,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xFF,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

#endif
	};

    if(usb_stor_init_ignor(us))
    {
        return 0;
    }

	US_DEBUGP("nokia1 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    return usb_stor_dev_init(us, msg, sizeof(msg), 1);
}


#endif

int usb_stor_huawei4_init(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
        0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

   if(usb_stor_init_ignor(us))
   {
       return 0;
    }

	US_DEBUGP("Huawei1 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}

int usb_stor_huawei1_init(struct us_data *us)
{
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11,
        0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

   if(usb_stor_init_ignor(us))
   {
       return 0;
    }

	US_DEBUGP("Huawei1 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}
int usb_stor_sev759_init(struct us_data *us)
{
//MessageContent="55 53 42 43 12 34 56 78 c0 00 00 00 80 00 06 71 01 0000000000000000000000000000
    char msg[] = {
        0x55, 0x53, 0x42, 0x43, 0x12, 0x34, 0x56, 0x78,
        0xc0, 0x00, 0x00, 0x00, 0x80, 0x00, 0x06, 0x71,
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

   if(usb_stor_init_ignor(us))
   {
       return 0;
    }

	US_DEBUGP("Huawei1 3G Dongle: %s", "DEVICE MODE SWITCH\n");
    usb_stor_dev_init(us, msg, sizeof(msg), 0);
    return 0;
}

//////sspemail 2012.314
