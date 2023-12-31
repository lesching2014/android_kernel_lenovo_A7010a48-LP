#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
#define SOC_BY_HW_FG
//lenovo-sw mahj2 modify for GM2.0 Begin
//#define HW_FG_FORCE_USE_SW_OCV
//lenovo-sw mahj2 modify for GM2.0 End
//#define SOC_BY_SW_FG


//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25


/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

/* lenovo-sw mahj2 support meter charger current use charger ic Begin*/
#define CHAGER_CURRENT_USE_SWITCHIC_METER
#ifdef CHAGER_CURRENT_USE_SWITCHIC_METER
#define CHARGER_IC_RLIM 240
#define CHARGER_IC_KLIM 435
#define CHARGER_CURRENT_ADC 12 //DCT has not adc2 , so define here
#endif
/* lenovo-sw mahj2 support meter charger current use charger ic End*/
			
#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

//lenovo-sw mahj2 modify for FG meter resistance Begin
#define FG_METER_RESISTANCE 	5
//lenovo-sw mahj2 modify for FG meter resistance End
/* Begin, lenovo-sw mahj2 update battery  ZCV param at 20150521*/
/* Qmax for battery  */
//#ifdef CONFIG_MTK_HAFG_20
#if 1
#define Q_MAX_POS_50	3300
#define Q_MAX_POS_25	3300
#define Q_MAX_POS_0		3102
#define Q_MAX_NEG_10	1761

#define Q_MAX_POS_50_H_CURRENT	3234
#define Q_MAX_POS_25_H_CURRENT	3234
#define Q_MAX_POS_0_H_CURRENT	  3040
#define Q_MAX_NEG_10_H_CURRENT	1726
#else
#define Q_MAX_POS_50	3300
#define Q_MAX_POS_25	3300
#define Q_MAX_POS_0		3300
#define Q_MAX_NEG_10	3201

#define Q_MAX_POS_50_H_CURRENT	3300
#define Q_MAX_POS_25_H_CURRENT	3300
#define Q_MAX_POS_0_H_CURRENT	  3205
#define Q_MAX_NEG_10_H_CURRENT	2450
#endif
/* End, lenovo-sw mahj2 update battery  ZCV param at 20150521*/

/* Discharge Percentage */
#define OAM_D5		 1		//  1 : D5,   0: D2


/* battery meter parameter */
#define CHANGE_TRACKING_POINT
#ifdef CONFIG_MTK_HAFG_20
#define CUST_TRACKING_POINT  0
#else
//lenovo-sw mahj2 modify for check point from 1 to 20 Begin
#define CUST_TRACKING_POINT  20
//lenovo-sw mahj2 modify for check point from 1 to 20 End
#endif
#define CUST_R_SENSE         68
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
//lenovo-sw mahj2 modify Begin
#define CAR_TUNE_VALUE		99 //1.00
//lenovo-sw mahj2 modify End


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			10 // mOhm, base is 20

/* fg 2.0 */
#define DIFFERENCE_HWOCV_RTC		30
#define DIFFERENCE_HWOCV_SWOCV		10
#define DIFFERENCE_SWOCV_RTC		10
#define MAX_SWOCV			3

#define DIFFERENCE_VOLTAGE_UPDATE	20
#define AGING1_LOAD_SOC			70
#define AGING1_UPDATE_SOC		30
//lenovo-sw mahj2 modify for 100% and 1% Begin
#define BATTERYPSEUDO100		98
#define BATTERYPSEUDO1			1
//lenovo-sw mahj2 modify for 100% and 1% End

#define Q_MAX_BY_SYS			//8. Qmax varient by system drop voltage.
//lenovo-sw mahj2 modify for add by MTK
#define Q_MAX_SYS_VOLTAGE 3350
//lenovo-sw mahj2 modify for add by MTK
#define SHUTDOWN_GAUGE0
#define SHUTDOWN_GAUGE1_XMINS
#define SHUTDOWN_GAUGE1_MINS		60

#define SHUTDOWN_SYSTEM_VOLTAGE		3400
#define CHARGE_TRACKING_TIME		60
#define DISCHARGE_TRACKING_TIME		10

#define RECHARGE_TOLERANCE		10
/* SW Fuel Gauge */
#define MAX_HWOCV			5
#define MAX_VBAT			90
#define DIFFERENCE_HWOCV_VBAT		30

/* fg 1.0 */
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	40
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30
#define CUST_POWERON_DELTA_HW_SW_OCV_CAPACITY_TOLRANCE	10


/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#define INIT_SOC_BY_SW_SOC
//#define SYNC_UI_SOC_IMM			//3. UI SOC sync to FG SOC immediately
#define MTK_ENABLE_AGING_ALGORITHM	//6. Q_MAX aging algorithm
#define MD_SLEEP_CURRENT_CHECK	//5. Gauge Adjust by OCV 9. MD sleep current check
//#define Q_MAX_BY_CURRENT		//7. Qmax varient by current loading.

#define FG_BAT_INT
 /*lenovo-sw mahj2 optim  code for dod init  Begin 2015-05-27 */ 
//#define IS_BATTERY_REMOVE_BY_PMIC
 /*lenovo-sw mahj2 optim  code for dod init  end 2015-05-27 */ 
/*lenovo-sw mahj2 modify for disable check Rfg exist  Begin 2015-05-27 */ 
#define DISABLE_RFG_EXIST_CHECK
/*lenovo-sw mahj2 modify for disable check Rfg exist  End 2015-05-27 */ 

#endif	//#ifndef _CUST_BATTERY_METER_H
