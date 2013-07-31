/*
 * drivers/char/tcc_mem.c
 *
 * TCC MEM driver
 *
 * Copyright (C) 2009 Telechips, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/syscalls.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#define DEV_MAJOR	213
#define DEV_MINOR	1
#define DEV_NAME	"skmem"


#define dprintk(msg...)	if (0) { printk( "skmem: " msg); }

#define SKMEM_BASE		0x9FF00000
#define SKMEM_SIZE		0x100000  //Last 1 Mib of Mem Area

struct allow_region {
	unsigned long	start;
	unsigned long	len;
};

struct allow_region AllowRegion[] = {
	{ SKMEM_BASE, SKMEM_SIZE},	
};

#define SIZE_TABLE_MAX (sizeof(AllowRegion)/sizeof(struct allow_region))

static int skmem_open(struct inode *inode, struct file *filp)
{
	dprintk("call skmem_open\n");
    if(filp->private_data != NULL){
	    dprintk("file->private_data is not null\n");
	}
	return 0;
}

static int skmem_release(struct inode *inode, struct file *filp)
{
	dprintk("call skmem_release\n");

	return 0;
}

static void mmap_mem_open(struct vm_area_struct *vma)
{
	/* do nothing */
}

static void mmap_mem_close(struct vm_area_struct *vma)
{
	/* do nothing */
}

static struct vm_operations_struct skmem_mmap_ops = {
	.open = mmap_mem_open,
	.close = mmap_mem_close,
};

static inline int uncached_access(struct file *file, unsigned long addr)
{
	if (file->f_flags & O_SYNC)
		return 1;
	return addr >= __pa(high_memory);
}

static pgprot_t phys_skmem_access_prot(struct file *file, unsigned long pfn,
				     unsigned long size, pgprot_t vma_prot)
{
	unsigned long offset = pfn << PAGE_SHIFT;

	if (uncached_access(file, offset))
		return pgprot_noncached(vma_prot);
	return vma_prot;
}

int range_is_allowed(unsigned long pfn, unsigned long size)
{
	int i;
	unsigned long request_start, request_end;

	request_start = pfn << PAGE_SHIFT;
	request_end = request_start + size;
	dprintk("Req: 0x%lx - 0x%lx, 0x%lx \n", request_start, request_end, size);

	for(i=0; i<SIZE_TABLE_MAX; i++)
	{
		if((AllowRegion[i].start <= request_start) && ((AllowRegion[i].start+AllowRegion[i].len) >= request_end))
		{
			dprintk("Allowed : 0x%lx <= 0x%lx && 0x%lx >= 0x%lx \n", AllowRegion[i].start, request_start, (AllowRegion[i].start+AllowRegion[i].len), request_end);
			return 1;
		}
	}
	
	return -1;
}
EXPORT_SYMBOL(range_is_allowed);

static int skmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	if(range_is_allowed(vma->vm_pgoff, size) < 0){
		printk(KERN_ERR	 "skmem: this address is not allowed, 0x%lx\n",vma->vm_pgoff);
		return -EPERM;
	}

	vma->vm_page_prot = phys_skmem_access_prot(file, vma->vm_pgoff, size, vma->vm_page_prot);
	vma->vm_ops = &skmem_mmap_ops;

	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot)) {
		printk(KERN_ERR	 "skmem: remap_pfn_range failed \n");
		return -EAGAIN;
	}
	return 0;
}

static struct file_operations skmem_fops = {
	.open		= skmem_open,
	.release	= skmem_release,
	.mmap		= skmem_mmap,
};

static void __exit skmem_cleanup(void)
{
	unregister_chrdev(DEV_MAJOR,DEV_NAME);
}

static struct class *skmem_class;

static int __init skmem_init(void)
{
	if (register_chrdev(DEV_MAJOR, DEV_NAME, &skmem_fops))
		printk(KERN_ERR "skmem: unable to get major %d for tMEM device\n", DEV_MAJOR);
	skmem_class = class_create(THIS_MODULE, DEV_NAME);
	device_create(skmem_class, NULL, MKDEV(DEV_MAJOR, DEV_MINOR), NULL, DEV_NAME);
	return 0;
}

module_init(skmem_init)
module_exit(skmem_cleanup)
