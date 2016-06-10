
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <mach/mt_clkmgr.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <mach/eint.h>
#include <mach/eint_drv.h>
#include "tpd.h"

#include "teei_client.h"
#include "teei_common.h"
#include "teei_id.h"
#include "teei_debug.h"
#include "smc_id.h"
#include "TEEI.h"
#include "tz_service.h"
#include "nt_smc_call.h"
#include "teei_admin_main.h"

#define TEEI_CONFIG_FULL_PATH_DEV_NAME "/dev/teei_config"
#define TEEI_CONFIG_DEV "teei_config"
#define TEEI_CONFIG_IOC_MAGIC 0x775B777E /* "TEEI Client" */


#define MAX_BUFF_SIZE		(4096)
#define NQ_SIZE			(4096)
#define CTL_BUFF_SIZE		(4096)
#define VDRV_MAX_SIZE		(0x80000)
#define NQ_VALID		1

#define F_CREATE_NQ_ID		0x01
#define F_CREATE_CTL_ID		0x02
#define F_CREATE_VDRV_ID	0x04

#define MESSAGE_SIZE		(4096)
#define VALID_TYPE		(1)
#define INVALID_TYPE		(0)

#define FAST_CALL_TYPE		(0x100)
#define STANDARD_CALL_TYPE	(0x200)
#define TYPE_NONE		(0x300)

#define FAST_CREAT_NQ		(0x40)
#define FAST_ACK_CREAT_NQ	(0x41)
#define FAST_CREAT_VDRV		(0x42)
#define FAST_ACK_CREAT_VDRV	(0x43)
#define FAST_CREAT_SYS_CTL	(0x44)
#define FAST_ACK_CREAT_SYS_CTL	(0x45)

#define NQ_CALL_TYPE            (0x60)
#define VDRV_CALL_TYPE          (0x61)
#define SCHD_CALL_TYPE		(0x62)
#define FDRV_ACK_TYPE           (0x63)

#define STD_INIT_CONTEXT	(0x80)
#define	STD_ACK_INIT_CONTEXT	(0x81)
#define STD_OPEN_SESSION	(0x82)
#define STD_ACK_OPEN_SESSION	(0x83)
#define STD_INVOKE_CMD		(0x84)
#define STD_ACK_INVOKE_CMD	(0x85)
#define STD_CLOSE_SESSION	(0x86)
#define STD_ACK_CLOSE_SESSION	(0x87)
#define STD_CLOSE_CONTEXT	(0x88)
#define STD_ACK_CLOSE_CONTEXT	(0x89)

#define GLSCH_NEG		(0x03)
#define GLSCH_NONE		(0x00)
#define GLSCH_LOW		(0x01)
#define GLSCH_HIGH		(0x02)

#define BOOT_IRQ		(283)
#define SCHED_IRQ		(284)
#define TEEI_CONFIG_IOCTL_INIT_TEEI _IOWR(TEEI_CONFIG_IOC_MAGIC, 3, int)

static struct work_queue *tz_wq;
struct semaphore tz_sem;

static int init_teei_framework(void);

int switch_enable_flag = 0;
int forward_call_flag = 0;


unsigned long sys_ctl_buffer = NULL;
unsigned long vdrv_buffer = NULL;

/*
 * structures and MACROs for NQ buffer
 */
#define NQ_BUFF_SIZE    (4096)
#define NQ_BLOCK_SIZE   (32)
#define BLOCK_MAX_COUNT (NQ_BUFF_SIZE / NQ_BLOCK_SIZE - 1)


#define STD_NQ_ACK_ID	0x01

#define TEE_NAME_SIZE	(255)


static DEFINE_MUTEX(nt_t_NQ_lock);
static DEFINE_MUTEX(t_nt_NQ_lock);

struct semaphore smc_lock;


struct NQ_head {
	unsigned int start_index;
	unsigned int end_index;
	unsigned int Max_count;
	unsigned char reserve[20];
};

struct NQ_entry {
	unsigned int valid_flag;
	unsigned int length;
	unsigned int buffer_addr;
	unsigned char reserve[20];
};

static void *tz_malloc(size_t size, int flags)
{
	void *ptr = kmalloc(size, flags|GFP_ATOMIC);
	return ptr;
}

static struct class *driver_class;
static dev_t teei_config_device_no;
static struct cdev teei_config_cdev;


struct teei_smc_cdata teei_smc_cd[NR_CPUS];

struct semaphore boot_sema;

#if 1


/*********************************************************************************
 *  global_fn			Global schedule thread function.
 *
 *  switch_enable_flag:		Waiting for the n_switch_to_t_os_stage2 finished
 *  forward_call_flag:		Forward direction call flag
 *
 *********************************************************************************/


int global_fn(void)
{
	forward_call_flag = GLSCH_NONE;

	while (1) {
		down(&smc_lock);

		if (forward_call_flag == GLSCH_HIGH) {
			pr_info("[%s][%d]**************************\n", __func__, __LINE__);
			nt_sched_t();
			msleep(200);
		} else if (forward_call_flag == GLSCH_LOW) {
			pr_info("[%s][%d]**************************\n", __func__, __LINE__);
			nt_sched_t();
			msleep(200);
		} else {
			up(&smc_lock);
			pr_info("[%s][%d]**************************\n", __func__, __LINE__);
			msleep(200);
		}
	}
}

/**
 * @brief	Map the vma with the free pages
 *
 * @param filp
 * @param vma
 *
 * @return	0: success
 *		EINVAL: Invalid parament
 *		ENOMEM: No enough memory
 */
static int teei_config_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return 0;
}

/**
 * @brief
 *
 * @param file
 * @param cmd
 * @param arg
 *
 * @return
 */
static unsigned int teei_flags;
static long teei_config_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int retVal = 0;
	void *argp = (void __user *) arg;

	switch (cmd) {

	case TEEI_CONFIG_IOCTL_INIT_TEEI:
		if (teei_flags == 1)
			break;
		else {
			init_teei_framework();
			teei_flags = 1;
		}

		break;

	default:
		pr_info("[%s][%d] command not found!\n", __func__, __LINE__);
		retVal = -EINVAL;
	}

	return retVal;
}

/**
 * @brief		The open operation of /dev/teei_config device node.
 *
 * @param inode
 * @param file
 *
 * @return		ENOMEM: no enough memory in the linux kernel
 *			0: on success
 */

static int teei_config_open(struct inode *inode, struct file *file)
{
	return 0;
}

/**
 * @brief		The release operation of /dev/teei_config device node.
 *
 * @param		inode: device inode structure
 * @param		file:  struct file
 *
 * @return		0: on success
 */
static int teei_config_release(struct inode *inode, struct file *file)
{
	return 0;
}

#endif


#if 0
void NQ_init(unsigned long NQ_buff)
{
	memset((char *)NQ_buff, 0, NQ_BUFF_SIZE);
}

static __always_inline unsigned int get_end_index(struct NQ_head *nq_head)
{
	if (nq_head->end_index == BLOCK_MAX_COUNT)
		return 1;
	else
		return nq_head->end_index + 1;
}
#endif

static irqreturn_t nt_sched_irq_handler(void)
{
	up(&smc_lock);
	return IRQ_HANDLED;
}


int register_sched_irq_handler(void)
{
	int retVal = 0;
	/* retVal = request_irq(SWITCH_IRQ, nt_switch_irq_handler,
				0, "tz_drivers_service", (void *)register_switch_irq_handler); */
	retVal = request_irq(SCHED_IRQ, nt_sched_irq_handler, 0, "tz_drivers_service", NULL);

	if (retVal)
		pr_info("ERROR for request_irq %d error code : %d.\n", SCHED_IRQ, retVal);
	else
		pr_info("request irq [ %d ] OK.\n", SCHED_IRQ);

	return 0;

}



static irqreturn_t nt_boot_irq_handler(void)
{
	if (forward_call_flag == GLSCH_NONE)
		forward_call_flag = GLSCH_NEG;
	else
		forward_call_flag = GLSCH_NONE;

	up(&smc_lock);
	up(&(boot_sema));

	return IRQ_HANDLED;
}



int register_boot_irq_handler(void)
{
	int retVal = 0;
	/* retVal = request_irq(SWITCH_IRQ, nt_switch_irq_handler,
				0, "tz_drivers_service", (void *)register_switch_irq_handler); */
	retVal = request_irq(BOOT_IRQ, nt_boot_irq_handler, 0, "tz_drivers_service", NULL);

	if (retVal)
		pr_info("ERROR for request_irq %d error code : %d.\n", BOOT_IRQ, retVal);
	else
		pr_info("request irq [ %d ] OK.\n", BOOT_IRQ);

	return 0;

}


int switch_to_t_os_stages2(void)
{

	/* down the boot_sema. */
	down(&(boot_sema));
	down(&(smc_lock));

	/* N_INVOKE_TO_T_OS_STAGE2 to TOS */
	n_switch_to_t_os_stage2();

	/* start HIGH level glschedule. */
	if (forward_call_flag == GLSCH_NONE)
		forward_call_flag = GLSCH_HIGH;
	else if (forward_call_flag == GLSCH_NEG)
		forward_call_flag = GLSCH_NONE;
	else
		return -1;

	/* block here until the TOS ack N_SWITCH_TO_T_OS_STAGE2 */
	down(&(boot_sema));
	up(&(boot_sema));

	return 0;
}


int t_os_load_image(void)
{
	/* down the boot_sema. */
	down(&(boot_sema));

	/* N_INVOKE_T_LOAD_TEE to TOS */
	set_sch_load_img_cmd();
	down(&smc_lock);
	n_invoke_t_load_tee(0, 0, 0);

	/* start HIGH level glschedule. */

	if (forward_call_flag == GLSCH_NONE)
		forward_call_flag = GLSCH_LOW;
	else if (forward_call_flag == GLSCH_NEG)
		forward_call_flag = GLSCH_NONE;
	else
		return -1;

	/* block here until the TOS ack N_SWITCH_TO_T_OS_STAGE2 */
	down(&(boot_sema));
	up(&(boot_sema));

	return 0;
}
static void init_smc_work(void)
{
	sema_init(&tz_sem, 0);
	tz_wq = create_workqueue("TZ SMC WORK");
}

/**
 * @brief
 */
static const struct file_operations teei_config_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = teei_config_ioctl,
	.open = teei_config_open,
	.mmap = teei_config_mmap,
	.release = teei_config_release
};

/**
 * @brief  init TEEI Framework
 * init Soter OS
 * init Global Schedule
 * init Forward Call Service
 * init CallBack Service
 * @return
 */

static int init_teei_framework(void)
{
	long retVal = 0;
	sema_init(&(smc_lock), 1);


	pr_info("================== Create SUB thread ===========================\n");
	kernel_thread(global_fn, NULL, CLONE_KERNEL);

	pr_info("================== Create SUB thread END===========================\n");


	TINFO("init Soter OS ...");

	/* register the boot stage IRQ */
	sema_init(&(boot_sema), 1);


	register_boot_irq_handler();
	register_sched_irq_handler();

	/* start TOS stages II. */
	switch_to_t_os_stages2();

	pr_info("[%s][%d] ===========================\n", __func__, __LINE__);
	retVal = create_cmd_buff();

	if (retVal < 0) {
		pr_info("[%s][%d] create_cmd_buff failed !\n", __func__, __LINE__);
		return retVal;
	}

	/* Init the smc queue. */
	pr_info("[%s][%d] ===========================\n", __func__, __LINE__);
	init_smc_work();

	pr_info("[%s][%d] ===========================\n", __func__, __LINE__);
	teei_service_init();

#if 1
	pr_info("[%s][%d] ===========================\n", __func__, __LINE__);
	t_os_load_image();
#endif
	TINFO("init Soter OS ...[OK]");

}
/**
 * @brief TEEI Agent Driver 初始化
 * 初始化TEEI 环境
 * 初始化TEEI 服务框架
 * @return
 */
static int teei_config_init(void)
{
	int ret_code = 0;
	long retVal = 0;
	struct device *class_dev = NULL;

	unsigned long irq_status = 0;

	pr_info("TEEI Config Driver Module Init ...\n");
	/***************************************************************************/
	ret_code = alloc_chrdev_region(&teei_config_device_no, 0, 1, TEEI_CONFIG_DEV);

	if (ret_code < 0) {
		TERR("alloc_chrdev_region failed %x", ret_code);
		return ret_code;
	}

	driver_class = class_create(THIS_MODULE, TEEI_CONFIG_DEV);

	if (IS_ERR(driver_class)) {
		ret_code = -ENOMEM;
		TERR("class_create failed %x", ret_code);
		goto unregister_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, teei_config_device_no, NULL, TEEI_CONFIG_DEV);

	if (NULL == class_dev) {
		TERR("class_device_create failed %x", ret_code);
		ret_code = -ENOMEM;
		goto class_destroy;
	}

	cdev_init(&teei_config_cdev, &teei_config_fops);
	teei_config_cdev.owner = THIS_MODULE;

	ret_code = cdev_add(&teei_config_cdev, MKDEV(MAJOR(teei_config_device_no), 0), 1);

	if (ret_code < 0) {
		TERR("cdev_add failed %x", ret_code);
		goto class_device_destroy;
	}

	/***************************************************************************/
	/* init_teei_framework(); */


	goto return_fn;

class_device_destroy:
	device_destroy(driver_class, teei_config_device_no);
class_destroy:
	class_destroy(driver_class);
unregister_chrdev_region:
	unregister_chrdev_region(teei_config_device_no, 1);
return_fn:
	TINFO("TEEI Config Driver Module Init ...[OK]");
	return ret_code;
}

/**
 * @brief
 */
static void teei_config_exit(void)
{
	TINFO("teei_config exit");

	device_destroy(driver_class, teei_config_device_no);
	class_destroy(driver_class);
	unregister_chrdev_region(teei_config_device_no, 1);
}


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TEEI <www.microtrust.com>");
MODULE_DESCRIPTION("TEEI Config");
MODULE_VERSION("1.00");

module_init(teei_config_init);

module_exit(teei_config_exit);


