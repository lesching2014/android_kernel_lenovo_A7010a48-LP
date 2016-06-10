/*
* Copyright (C) 2012 lenovo, Inc.
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

#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/ctype.h>
#include <linux/uaccess.h> /* sys_sync */
#include <linux/rtc.h> /* sys_sync */
/* yangjq, 2011-12-16, Add for vreg, START */
#include <linux/platform_device.h>
/* yangjq, 2011-12-16, Add for vreg, END */
#include <linux/err.h>
////#include <mach/pmic.h>
#include <linux/regulator/consumer.h>
#include <mt_spm.h>

static struct kobject *sysfs_private_kobj;

#define private_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

extern unsigned int pmlog_get_cpufreq_freq(void);
extern int wakelock_dump_info(char* buf);
static ssize_t pm_status_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	char *s = buf;
	unsigned int rate; // khz
	int cpu;

	// show CPU clocks
	rate = pmlog_get_cpufreq_freq();
	for (cpu = 0; cpu < nr_cpu_ids; cpu++) {
		s += sprintf(s, "APPS[%d]:", cpu);
		if (cpu_online(cpu)) {
			s += sprintf(s, "(%3u MHz); \n", rate/1000);
		} else {
			s += sprintf(s, "sleep; \n");
		}
	}

	s += wakelock_dump_info(s);
	
	return (s - buf);
}

static ssize_t pm_status_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	printk(KERN_ERR "%s: no support yet.\n", __func__);

	return -EPERM;
}


private_attr(pm_status);


static struct attribute *g_private_attr[] = {
	&pm_status_attr.attr,
	NULL,
};

static struct attribute_group private_attr_group = {
	.attrs = g_private_attr,
};

#define SLEEP_LOG
#ifdef SLEEP_LOG
#define WRITE_SLEEP_LOG
#define MAX_WAKEUP_IRQ 32

enum {
	DEBUG_SLEEP_LOG = 1U << 0,
	DEBUG_WRITE_LOG = 1U << 1,
	DEBUG_WAKEUP_IRQ = 1U << 2,
	DEBUG_RPM_SPM_LOG = 1U << 3,
	DEBUG_RPM_CXO_LOG = 1U << 4,
	DEBUG_ADSP_CXO_LOG = 1U << 5,
	DEBUG_MODEM_CXO_LOG = 1U << 6,
	DEBUG_WCNSS_CXO_LOG = 1U << 7,
};
static int debug_mask = DEBUG_WRITE_LOG |DEBUG_SLEEP_LOG |DEBUG_WAKEUP_IRQ ;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

struct sleep_log_t {
	char time[18];
	long timesec;
	unsigned int log;
	int wakeup_irq[MAX_WAKEUP_IRQ];
//31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//bit1-0=00 :try to sleep; bit 1-0 = 01 : leave from sleep    ;bit1-0=10:fail to sleep
//bit31-bit24 : return value
};

#define TRY_TO_SLEEP  (0)
#define LEAVE_FORM_SLEEP  (1)
#define FAIL_TO_SLEEP  (2)

#define SLEEP_LOG_LENGTH 80

/* define [x] = " XXX" */
#undef SPM_WAKE_SRC
#define SPM_WAKE_SRC(id, name)	\
	[id] = " " #name
static const char *wakesrc_str[32] = SPM_WAKE_SRC_LIST;

struct sleep_log_t sleep_log_array[SLEEP_LOG_LENGTH];
int sleep_log_pointer = 0;
int sleep_log_count = 0;
int enter_times = 0;

static int irq_wakeup_saved = MAX_WAKEUP_IRQ;
static int irq_wakeup_irq[MAX_WAKEUP_IRQ];

char sleep_log_name[60];
struct file *sleep_log_file = NULL;

#ifdef WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	char buf[256];
	char *p, *p0;
	int i, j, pos;
	mm_segment_t old_fs;
	p = buf;
	p0 = p;

	if (sleep_log_file == NULL)
		sleep_log_file = filp_open(sleep_log_name, O_RDWR | O_APPEND | O_CREAT,
				0644);
	if (IS_ERR(sleep_log_file)) {
		printk("error occured while opening file %s, err=%ld, exiting...\n",
				sleep_log_name, (unsigned long)sleep_log_file);
		return 0;
	}

	if (sleep_log_count > 1) {
		for (i = 0; i < 2; i++) {
			if (sleep_log_pointer == 0)
				pos = SLEEP_LOG_LENGTH - 2 + i;
			else
				pos = sleep_log_pointer - 2 + i;
			switch (sleep_log_array[pos].log & 0xF) {
			case TRY_TO_SLEEP:
				p += sprintf(p, ">[%ld]%s\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time);
				break;
			case LEAVE_FORM_SLEEP:
				p += sprintf(p, "<[%ld]%s(",
						sleep_log_array[pos].timesec,
						sleep_log_array[pos].time);
				for (j = 0; j < MAX_WAKEUP_IRQ && sleep_log_array[pos].wakeup_irq[j]; j++)
				{
                                             if (sleep_log_array[pos].wakeup_irq[j] < MAX_WAKEUP_IRQ)
                                             {
					    p += sprintf(p, " %s,", wakesrc_str[sleep_log_array[pos].wakeup_irq[j]]);
                                             }
                                             else
                                             {
					    p += sprintf(p, " %d", sleep_log_array[pos].wakeup_irq[j]);
                                             }
				}

				p += sprintf(p, ")\n");
				break;
			case FAIL_TO_SLEEP:
				p += sprintf(p, "^[%ld]%s(%d)\n", sleep_log_array[pos].timesec,
						sleep_log_array[pos].time,
						(char) (sleep_log_array[pos].log >> 24));
				break;
			}
		}
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sleep_log_file->f_op->write(sleep_log_file, p0, p - p0,
			&sleep_log_file->f_pos);
	set_fs(old_fs);

	if (sleep_log_file != NULL) {
		filp_close(sleep_log_file, NULL);
		sleep_log_file = NULL;
	}
	return 0;
}
#else //WRITE_SLEEP_LOG
static int sleep_log_write(void)
{
	return 0;
}
#endif //WRITE_SLEEP_LOG

int save_wakeup_reson(int irq)
{
	int i;
	int ret;

	ret = 0;
         //printk("%s(), %d,irq=%d\n", __func__,irq_wakeup_saved,  irq);
	if (irq_wakeup_saved < MAX_WAKEUP_IRQ ) {
	    ret = irq_wakeup_irq[irq_wakeup_saved++] = irq;
	}
	return ret;
}

static void clear_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ) {
		memset(irq_wakeup_irq, 0, sizeof(irq_wakeup_irq));
		irq_wakeup_saved = 0;
	}
}

static void set_irq_wakeup_saved(void)
{
	if (debug_mask & DEBUG_WAKEUP_IRQ)
		irq_wakeup_saved = MAX_WAKEUP_IRQ;
}

void log_suspend_enter(void)
{
	extern void smem_set_reserved(int index, int data);
	struct timespec ts_;
	struct rtc_time tm_;

	//Turn on/off the share memory flag to inform RPM to record spm logs
	//smem_set_reserved(6, debug_mask & DEBUG_WAKEUP_IRQ ? 1 : 0);
//	smem_set_reserved(6, debug_mask);

	if (debug_mask & DEBUG_SLEEP_LOG) {
		printk("%s(), APPS try to ENTER sleep mode>>>\n", __func__);

		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);

		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		if (strlen(sleep_log_name) < 1) {
			sprintf(sleep_log_name,
					"/data/local/log/aplog/sleeplog%d%02d%02d_%02d%02d%02d.txt",
					tm_.tm_year + 1900, tm_.tm_mon + 1, tm_.tm_mday, tm_.tm_hour,
					tm_.tm_min, tm_.tm_sec);
			printk("%s(), sleep_log_name = %s \n", __func__, sleep_log_name);
		}
                  printk("%s(), sleep_log_name1 = %s \n", __func__, sleep_log_name);

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;
		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log = TRY_TO_SLEEP;
		sleep_log_pointer++;
		sleep_log_count++;
		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;
	}

	clear_irq_wakeup_saved();
}

void log_suspend_exit(int error)
{
	struct timespec ts_;
	struct rtc_time tm_;
	uint32_t smem_value;
	int i;

	if (debug_mask & DEBUG_SLEEP_LOG) {
                  printk("%s(), APPS try to EXIT sleep mode>>>\n", __func__);
		getnstimeofday(&ts_);
		rtc_time_to_tm(ts_.tv_sec + 8 * 3600, &tm_);
		sprintf(sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].time,
				"%d-%02d-%02d %02d:%02d:%02d", tm_.tm_year + 1900, tm_.tm_mon + 1,
				tm_.tm_mday, tm_.tm_hour, tm_.tm_min, tm_.tm_sec);

		sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].timesec = ts_.tv_sec;

		if (error == 0) {
			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					LEAVE_FORM_SLEEP;

			for (i = 0; i < irq_wakeup_saved; i++)
			{
				sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].wakeup_irq[i] =
					irq_wakeup_irq[i];
                                    printk("%s(), log irq=%d\n", __func__, irq_wakeup_irq[i]);
			}
		} else {
			printk("%s(), APPS FAIL to enter sleep^^^\n", __func__);

			sleep_log_array[sleep_log_pointer % SLEEP_LOG_LENGTH].log =
					FAIL_TO_SLEEP | (error << 24);
		}

		sleep_log_pointer++;
		sleep_log_count++;

		if (sleep_log_pointer == SLEEP_LOG_LENGTH)
			sleep_log_pointer = 0;

		if (debug_mask & DEBUG_WRITE_LOG) {
			enter_times++;
			if (enter_times < 5000)
				sleep_log_write();
		}
	}

	set_irq_wakeup_saved();
}
#else //SLEEP_LOG
void log_suspend_enter(void)
{
	clear_irq_wakeup_saved();
}

void log_suspend_exit(int error)
{
	set_irq_wakeup_saved();
}
#endif //SLEEP_LOG

static int __init sysfs_private_init(void)
{
	int result;

	printk("%s(), %d\n", __func__, __LINE__);

	sysfs_private_kobj = kobject_create_and_add("private", NULL);
	if (!sysfs_private_kobj)
		return -ENOMEM;

	result = sysfs_create_group(sysfs_private_kobj, &private_attr_group);
	printk("%s(), %d, result=%d\n", __func__, __LINE__, result);

#ifdef SLEEP_LOG
	strcpy (sleep_log_name, "");
	sleep_log_pointer = 0;
	sleep_log_count = 0;
	enter_times = 0;
#endif

	return result;
}

static void __exit sysfs_private_exit(void)
{
	printk("%s(), %d\n", __func__, __LINE__);
	sysfs_remove_group(sysfs_private_kobj, &private_attr_group);

	kobject_put(sysfs_private_kobj);
}

module_init(sysfs_private_init);
module_exit(sysfs_private_exit);
