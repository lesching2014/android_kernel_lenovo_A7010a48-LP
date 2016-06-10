#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <mach/mt_ccci_common.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include "sw_tx_power.h"

//linyf test #define TEST_MODE

DECLARE_WAIT_QUEUE_HEAD(wq_swtp);

extern int swtp_mod_eint_init(void);
static int swtp_mode_update_handler();
static int swtp_state_machine(void *arg);
static int swtp_set_tx_power();
static int switch_MD1_Tx_Power_TEST(unsigned int mode);

static unsigned int rf_cable_state = 0;
static unsigned int swtp_is_ready = 0;
static unsigned int current_mode = 21;

static int __init swtp_mod_init(void)
{
	int ret = 0;
	printk("swtp_mod_init.\n");

#ifndef TEST_MODE
	swtp_mod_eint_init();
#endif
	
	if(get_modem_is_enabled(0))
	{
		ret = register_ccci_sys_call_back(0, MD_SW_MD1_TX_POWER_REQ, swtp_mode_update_handler);
		printk("Horizon: swtp register MD1 callback, [%d]\n", ret);
	}
	else
	{
		printk("Horizon: swtp_mod_init, modem 0 is not enabled\n");
	}

	kthread_run(swtp_state_machine, 0, "swtp kthread");
	
	return ret;
}

extern int switch_MD1_Tx_Power(unsigned int mode);

static int swtp_set_tx_power()
{
	if(rf_cable_state == SWTP_MODE_ON)
	{
		current_mode = 42;
#ifdef TEST_MODE
		switch_MD1_Tx_Power_TEST(42);
#else
    switch_MD1_Tx_Power(42);
#endif
	}
	else if(rf_cable_state == SWTP_MODE_OFF)
	{
		current_mode = 21;
#ifdef TEST_MODE
		switch_MD1_Tx_Power_TEST(21);
#else
    switch_MD1_Tx_Power(21);
#endif
    
	}
	return 0;
}

//linyf modify for SWTP func 2015-07-31 begin
extern int swtp_mod_eint_read_lenovo(void);

static int swtp_mode_update_handler()
{
	swtp_is_ready = 1;
	rf_cable_state = swtp_mod_eint_read_lenovo();
	swtp_set_tx_power();
	return 1;
}
//linyf modify for SWTP func 2015-07-31 end

static int swtp_state_machine(void *arg)
{
	for(;;)
	{
		wait_event_interruptible(wq_swtp, 0);
		printk("Horizon: swtp_state_machine()\n");
		if(swtp_is_ready == 1)
		{
			swtp_set_tx_power();
		}
#ifdef TEST_MODE
		else
		{
			swtp_set_tx_power();
		}
#endif
	}
	return 0;
}


int swtp_set_mode_unlocked(unsigned int ctrid, unsigned int enable)
{
	rf_cable_state = enable;

	printk("Horizon: swtp_set_mode_unlocked, set rfcable = %d\n", rf_cable_state);
	
//	wake_up_interruptible(&wq_swtp);
	swtp_set_tx_power();
	return 0;
}


//just for test
int swtp_set_mode(unsigned int ctrid, unsigned int enable)
{
	rf_cable_state = enable;
	printk("Horizon: swtp_set_mode, set rfcable = %d\n", rf_cable_state);
	
	wake_up_interruptible(&wq_swtp);
	printk("Horizon: after swtp_set_mode\n");
	
	return 0;
}

unsigned int swtp_get_mode()
{
	printk("Horizon: swtp_get_mode, rfcable = %d\n", rf_cable_state);
	return rf_cable_state;
}

static int switch_MD1_Tx_Power_TEST(unsigned int mode)
{
	printk("Horizon: switch_Tx_Power, mode = %d\n", mode);
	return 0;
}

int swtp_rfcable_tx_power(void)
{
	return 0;
}

int swtp_reset_tx_power(void)
{
	return 0;
}
//end test

module_init(swtp_mod_init);


