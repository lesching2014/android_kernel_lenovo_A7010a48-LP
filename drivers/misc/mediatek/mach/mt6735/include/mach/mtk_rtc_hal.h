/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
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

#ifndef _MTK_RTC_HAL_H_
#define _MTK_RTC_HAL_H_

#include <linux/kernel.h>
#include <linux/rtc.h>
#include <rtc-mt.h>		/* custom file */


#define PMIC_REGISTER_INTERRUPT_ENABLE		//register rtc interrupt
#ifdef PMIC_REGISTER_INTERRUPT_ENABLE
#define RTC_INTERRUPT_NUM		9
#endif

#ifdef VRTC_PWM_ENABLE
#define RTC_PWM_ENABLE_POLLING_TIMER		30*60	//30 min 
#endif

#define RTC_GPIO_USER_MASK	(((1U << 13) - 1) & 0xff00)

/* RTC registers */
#define	RTC_BASE					(0x4000)

extern u16 hal_rtc_get_register_status(const char * cmd);
extern void hal_rtc_set_register_status(const char * cmd, u16 val);
extern void hal_rtc_set_gpio_32k_status(u16 user, bool enable);
extern void hal_rtc_set_writeif(bool enable);
extern void hal_rtc_mark_mode(const char *cmd);
extern u16 hal_rtc_rdwr_uart(u16 *val);
extern void hal_rtc_bbpu_pwdn(void);
extern void hal_rtc_get_pwron_alarm(struct rtc_time *tm, struct rtc_wkalrm *alm);
extern void hal_rtc_set_pwron_alarm(void);
extern bool hal_rtc_is_lp_irq(void);
extern void hal_rtc_reload_power(void);
extern void hal_rtc_get_tick_time(struct rtc_time *tm);
extern void hal_rtc_set_tick_time(struct rtc_time *tm);
extern bool hal_rtc_check_pwron_alarm_rg(struct rtc_time *nowtm, struct rtc_time *tm);
extern void hal_rtc_get_alarm_time(struct rtc_time *tm, struct rtc_wkalrm *alm);
extern void hal_rtc_set_alarm_time(struct rtc_time *tm);
extern void hal_rtc_clear_alarm(struct rtc_time *tm);
extern void hal_rtc_set_lp_irq(void);
extern void hal_rtc_read_rg(void);
extern void hal_rtc_save_pwron_time(bool enable, struct rtc_time *tm, bool logo);
//lenovo-sw mahj2 modify for timezone at 20141204 Begin
#ifdef CONFIG_LENOVO_RTC_SAVE_TIMEZONE_SUPPORT
extern void hal_rtc_set_spare_timezone_value(int tz_minuteswest);
extern int hal_rtc_get_spare_timezone_value(void);
#endif
//lenovo-sw mahj2 modify for timezone at 20141204 End
#ifdef VRTC_PWM_ENABLE
extern void hal_rtc_pwm_enable(void);
#endif
#endif
