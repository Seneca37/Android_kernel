/*

* powersave.c

*

* Created on: 2012-08-07

* Author: Robert Lee(Samkoon)

* Email:

*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include "../../../arch/arm/mach-omap2/include/mach/samkoonhmi.h"


#define FILE_PATH_WRITE "/powersave.bin"

#define FILE_PATH_READ "/default.prop"


char *file_buf; //保存开辟的内存空间的地址的指针变量
	struct file * cfile=NULL;  


#define PMEM_BASE 0x9ff00000 
#define PMEM_BASE_SIZE SZ_1M
#define PMEM_ADSP_BASE 0x9ff00000 
#define PMEM_ADSP_BASE_SIZE SZ_1M

int PowerSaveAddr;

void powersave_init(void)
{

	int i;
 #if 0//def LCD_035TINC
	PowerSaveAddr=ioremap_nocache(PMEM_BASE, SZ_8K);//SZ_1M);	
#else
	PowerSaveAddr=ioremap_nocache(PMEM_BASE, SZ_128K);//SZ_1M);	
#endif
	printk("PowerSaveAddr 0x%x,%s() \n\n", PowerSaveAddr,__func__);
   	file_buf=PowerSaveAddr;  
}

void powersave_open(void)
{
	uint32_t err = 0;  
	struct file * readfile=NULL;  
	if(cfile!=NULL)
		return;
	readfile=filp_open(FILE_PATH_READ, O_RDWR, 0);
	if (IS_ERR(readfile))  
	{
      printk("cat not open default.prop/n");  
  		return;  	 
	}
   	
  cfile = filp_open(FILE_PATH_WRITE, O_CREAT | O_SYNC|O_RDWR | O_LARGEFILE, 0644);  
  if (IS_ERR(cfile))  
  		return PTR_ERR(cfile);  
      
  if (!cfile->f_op || (!cfile->f_op->read && !cfile->f_op->aio_read)) {  
       printk("alloc_device: cache file not readable/n");  
       err = -EINVAL;  
       goto err_open;  
   }  
    
   if (!cfile->f_op->write && !cfile->f_op->aio_write) {  
        printk("alloc_device: cache file not writeable/n");  
        err = -EINVAL;  
        goto err_open;  
   }
    printk("powersave.bin create success/n");  
    return ;  
err_open:  
    filp_close(cfile, NULL);  
    return ;    	
}

int powersave(void) //powersave 的主函数
{
//	struct file * cfile;  
	uint32_t i,err = 0;  
	mm_segment_t old_fs;  
	ssize_t tx;  
	//uint8_t buf[16] = "Hello World!";  
  loff_t pos = 0;  

 //  printk("enter powersave.bin \n");  
//	PowerSaveAddr=ioremap_nocache(PMEM_BASE, SZ_1M);	
//	printk("PowerSaveAddr 0x%x,%s() \n\n", PowerSaveAddr,__func__);
 //  file_buf=PowerSaveAddr;  
    
//	if(cfile==NULL)
	{
//    printk("create powersave.bin \n");  
   	cfile = filp_open(FILE_PATH_WRITE, O_CREAT | O_SYNC|O_RDWR |O_TRUNC| O_LARGEFILE, 0644);  
  	if (IS_ERR(cfile))  
  		return PTR_ERR(cfile);  
      
  	if (!cfile->f_op || (!cfile->f_op->read && !cfile->f_op->aio_read)) {  
        printk("alloc_device: cache file not readable/n");  
        err = -EINVAL;  
        goto err_close;  
   	}  
    
   	if (!cfile->f_op->write && !cfile->f_op->aio_write) {  
        printk("alloc_device: cache file not writeable/n");  
        err = -EINVAL;  
        goto err_close;  
    }
  }  

       
//   printk("Open the file powersave.bin \n");  
   old_fs = get_fs();  
   set_fs(get_ds());
/*
		for(i=0;i<256;i++)
		{
			printk("file_buf[%d]=%d",i,file_buf[i]);
		}
 */
 #ifdef LCD_035TINC
    tx = vfs_write(cfile, (char __user *)file_buf, 8*1024, &pos);  
 #else
   tx = vfs_write(cfile, (char __user *)file_buf, 128*1024, &pos);  
#endif
   set_fs(old_fs);  
      
err_close:  
    filp_close(cfile, NULL);  
    return err;  
}
