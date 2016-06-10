/*******************************************************************************
* mt_pwm.c PWM Drvier                                                     
*                                                                                             
* Copyright (c) 2012, Media Teck.inc                                           
*                                                                             
* This program is free software; you can redistribute it and/or modify it     
* under the terms and conditions of the GNU General Public Licence,            
* version 2, as publish by the Free Software Foundation.                       
*                                                                              
* This program is distributed and in hope it will be useful, but WITHOUT       
* ANY WARRNTY; without even the implied warranty of MERCHANTABITLITY or        
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for     
* more details.                                                                
*                                                                              
*                                                                              
********************************************************************************
* Author : How Wang (how.wang@mediatek.com)                              
********************************************************************************
*/

#include <linux/kernel.h>
#include <generated/autoconf.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/mt_pwm_hal_pub.h>
#include <mach/mt_pwm_hal.h>
#include <mach/mt_pwm_prv.h>

enum {                                                                                     
	PWM_CON,
	PWM_HDURATION,
	PWM_LDURATION,
	PWM_GDURATION,
	PWM_BUF0_BASE_ADDR,
	PWM_BUF0_SIZE,
	PWM_BUF1_BASE_ADDR,
	PWM_BUF1_SIZE,
	PWM_SEND_DATA0,
	PWM_SEND_DATA1,
	PWM_WAVE_NUM,
	PWM_DATA_WIDTH,		
	PWM_THRESH,			
	PWM_SEND_WAVENUM,	
	PWM_VALID
}PWM_REG_OFF;

#ifdef CONFIG_OF
extern void __iomem *pwm_base;
unsigned long PWM_register[PWM_NUM]={
//	(pwm_base+0x0010),     //PWM1 register base,   15 registers
//	(pwm_base+0x0050),     //PWM2 register base    15 registers
//	(pwm_base+0x0090),     //PWM3 register base    15 registers
};
#else
unsigned long PWM_register[PWM_NUM]={
	(PWM_BASE+0x0010),     //PWM0 register base,   15 registers
	(PWM_BASE+0x0050),     //PWM1 register base    15 registers
	(PWM_BASE+0x0090),     //PWM2 register base    15 registers
	(PWM_BASE+0x00d0),     //PWM3 register base    13 registers
	(PWM_BASE+0x0110),     //PWM4 register base    13 registers
	(PWM_BASE+0x0150),	   //PWM5 register base    13 registers
	(PWM_BASE+0x0190),	   //PWM6 register base    13 registers
};
#endif

extern void __iomem *pericfg_base_t;
extern void __iomem *dcmcfg_base;

static int pwm_power_id[] = {
	MT_CG_PERI_PWM1,
	MT_CG_PERI_PWM2,
	MT_CG_PERI_PWM3,
	MT_CG_PERI_PWM4,
        MT_CG_PERI_PWM5,
	MT_CG_PERI_PWM6,
	MT_CG_PERI_PWM7,
	MT_CG_PERI_PWM
};

//pmic_pad : useless on 6595
#define PWM_CG 7
void mt_pwm_power_on_hal(U32 pwm_no, BOOL pmic_pad, unsigned long* power_flag)
{
	if(0 == (*power_flag)){
	//	printk("enable_clock: main\n");//
		enable_clock(pwm_power_id[PWM_CG], "PWM");
		set_bit(PWM_CG, power_flag);
	}
	if (!test_bit(pwm_no, power_flag)) {
	//	printk("enable_clock: %d\n", pwm_no);//

	//	printk("ap pad\n");//
		enable_clock(pwm_power_id[pwm_no], "PWM");  //enable clock 

		set_bit(pwm_no,power_flag);
	//	printk("enable_clock PWM%d\n", pwm_no+1);
	}
}

void mt_pwm_power_off_hal(U32 pwm_no, BOOL pmic_pad, unsigned long* power_flag)
{
	if (test_bit(pwm_no, power_flag)) {
	//	printk("disable_clock: %d\n", pwm_no);//

	//	printk("ap pad\n");//
		disable_clock(pwm_power_id[pwm_no], "PWM");  //disable clock 

		clear_bit(pwm_no,power_flag);
	//	printk("disable_clock PWM%d\n", pwm_no+1);
	}
	if(BIT(PWM_CG) == (*power_flag)){
	//	printk("disable_clock: main\n");//
		disable_clock(pwm_power_id[PWM_CG], "PWM");
		clear_bit(PWM_CG, power_flag);
	}
}
void mt_pwm_init_power_flag(unsigned long* power_flag)
{
#ifdef CONFIG_OF
	PWM_register[PWM1] = (unsigned long)pwm_base+0x0010;	   //PWM1 register base,   15 registers
	PWM_register[PWM2] = (unsigned long)pwm_base+0x0050;	   //PWM2 register base,   15 registers
	PWM_register[PWM3] = (unsigned long)pwm_base+0x0090;	   //PWM3 register base,   15 registers
	PWM_register[PWM4] = (unsigned long)pwm_base+0x00d0;	   //PWM4 register base,   15 registers
	PWM_register[PWM5] = (unsigned long)pwm_base+0x0110;	   //PWM5 register base,   15 registers
#endif
/*
    U32 idx=0;
    for(;idx<sizeof(pwm_power_id)/sizeof(pwm_power_id[0]);idx++)
    {
        if(clock_is_on(pwm_power_id[idx]))
        {
            set_bit(idx,power_flag);
        }
        else
        {
            clear_bit(idx, power_flag);
        }
    }
    printk("mt_pwm_init_power_flag 0x%x\n", *power_flag);
*/
}
S32 mt_pwm_sel_pmic_hal(U32 pwm_no)
{
//	printk("mt_pwm_sel_pmic\n");//
	return EINVALID;
}

S32 mt_pwm_sel_ap_hal(U32 pwm_no)
{
//	printk("mt_pwm_sel_ap\n");//
	return EINVALID;
}

void mt_set_pwm_enable_hal(U32 pwm_no)
{
	SETREG32(PWM_ENABLE, 1 << pwm_no);
}

#if 1

void mt_set_pwm_enable_ahb_hal(U32 bit)
{
	SETREG32(CK_SEL, 1 << bit);
}

void ultra_16_hal(U32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 16);
}
void ultra_24_hal(U32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 24);
}

void ultra_4_hal(U32 bit)
{
	SETREG32(PWM_ULTRA, 1 << 4);
}

void dcm_hal(U32 bit)
{
	CLRREG32(DCM_ENABLE, 1 << 0);
}

void dcm_cfg_hal(U32 bit)
{
	CLRREG32(DCM_CFG, 1 << 7);
}


void mt_set_pwm_seqmode_multiport_enable_hal(U32 pwm_no_sclk, U32 pwm_no_mosi)
{
	U32 res = 1 << PWM_ENABLE_SEQ_OFFSET;
	res |= 1 << pwm_no_sclk;
	res |= 1 << pwm_no_mosi;
//	printk("\r\n[PWM_ENABLE is:%x]\n\r ", res);
	SETREG32(PWM_ENABLE, res);
	
}

#endif 
void mt_set_pwm_disable_hal(U32 pwm_no)
{
	CLRREG32 ( PWM_ENABLE, 1 << pwm_no );
}
void mt_set_pwm_enable_seqmode_hal(void)
{
	SETREG32 ( PWM_ENABLE, 1 << PWM_ENABLE_SEQ_OFFSET );
}
void mt_set_pwm_disable_seqmode_hal(void)
{
	CLRREG32 ( PWM_ENABLE,1 << PWM_ENABLE_SEQ_OFFSET );
}
S32 mt_set_pwm_test_sel_hal(U32 val)
{
	return 0;
}

void mt_set_pwm_clk_hal (U32 pwm_no, U32 clksrc, U32 div )
{
	unsigned long reg_con;

	reg_con = PWM_register [pwm_no] + 4* PWM_CON;	
	MASKREG32 ( reg_con, PWM_CON_CLKDIV_MASK, div );	
	if ( (clksrc&0x80000000)!=0 ) {
		clksrc &= ~(0x80000000);		
		if (clksrc == CLK_BLOCK_BY_1625_OR_32K) {	// old mode: 26M/1625 = 16KHz
			CLRREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET );	//bit 4: 0 
			SETREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OFFSET );	// bit 3: 1	
		}
		else {	 //old mode 32k clk	
			SETREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET ); 	
			SETREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OFFSET );	
		} 
	} 
	else {		
		CLRREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OLD_OFFSET );		
		if (clksrc == CLK_BLOCK) {			
			CLRREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OFFSET );		
		} 
		else if (clksrc == CLK_BLOCK_BY_1625_OR_32K) {			
			SETREG32 ( reg_con, 1 << PWM_CON_CLKSEL_OFFSET );		
		}	
	}
}
S32 mt_get_pwm_clk_hal(U32 pwm_no)
{
	S32 clk, clksrc, clkdiv;
	unsigned long reg_con, reg_val,reg_en;

	reg_con = PWM_register[pwm_no] + 4*PWM_CON;

	reg_val = INREG32 (reg_con);
	reg_en = INREG32 (PWM_ENABLE);

	if ( ( ( reg_val & PWM_CON_CLKSEL_MASK ) >> PWM_CON_CLKSEL_OFFSET ) == 1 )
		if ( ((reg_en &PWM_CON_OLD_MODE_MASK) >> PWM_CON_OLD_MODE_OFFSET ) == 1)
			clksrc = 32*1024;
		else clksrc = BLOCK_CLK;
	else
		clksrc = BLOCK_CLK/1625;

	clkdiv = 2 << ( reg_val & PWM_CON_CLKDIV_MASK);
	if ( clkdiv <= 0 ) {
		printk ( "clkdiv less zero, not valid \n" );
		return -ERROR;
	}

	clk = clksrc/clkdiv;
	printk ( "CLK is :%d\n", clk );
	return clk;
}
S32 mt_set_pwm_con_datasrc_hal ( U32 pwm_no, U32 val )
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( val == PWM_FIFO )
		CLRREG32 ( reg_con, 1 << PWM_CON_SRCSEL_OFFSET );
	else if ( val == MEMORY )
		SETREG32 ( reg_con, 1 << PWM_CON_SRCSEL_OFFSET );
	else 
		return 1;
	return 0;
}
S32 mt_set_pwm_con_mode_hal( U32 pwm_no, U32 val )
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( val == PERIOD )
		CLRREG32 ( reg_con, 1 << PWM_CON_MODE_OFFSET );
	else if (val == RAND)
		SETREG32 ( reg_con, 1 << PWM_CON_MODE_OFFSET );
	else
		return 1;
	return 0;
}
S32 mt_set_pwm_con_idleval_hal(U32 pwm_no, U16 val)
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( val == IDLE_TRUE )
		SETREG32 ( reg_con,1 << PWM_CON_IDLE_VALUE_OFFSET );
	else if ( val == IDLE_FALSE )
		CLRREG32 ( reg_con, 1 << PWM_CON_IDLE_VALUE_OFFSET );
	else 
		return 1;
	return 0;
}
S32 mt_set_pwm_con_guardval_hal(U32 pwm_no, U16 val)
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( val == GUARD_TRUE )
		SETREG32 ( reg_con, 1 << PWM_CON_GUARD_VALUE_OFFSET );
	else if ( val == GUARD_FALSE )
		CLRREG32 ( reg_con, 1 << PWM_CON_GUARD_VALUE_OFFSET );
	else 
		return 1;
	return 0;
}
void mt_set_pwm_con_stpbit_hal(U32 pwm_no, U32 stpbit, U32 srcsel )
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( srcsel == PWM_FIFO )
		MASKREG32 ( reg_con, PWM_CON_STOP_BITS_MASK, stpbit << PWM_CON_STOP_BITS_OFFSET);
	if ( srcsel == MEMORY )
		MASKREG32 ( reg_con, PWM_CON_STOP_BITS_MASK & (0x1f << PWM_CON_STOP_BITS_OFFSET), stpbit << PWM_CON_STOP_BITS_OFFSET);
}
S32 mt_set_pwm_con_oldmode_hal ( U32 pwm_no, U32 val )
{
	unsigned long reg_con;
	
	reg_con = PWM_register[pwm_no] + 4*PWM_CON;
	if ( val == OLDMODE_DISABLE )
		CLRREG32 ( reg_con, 1 << PWM_CON_OLD_MODE_OFFSET );
	else if ( val == OLDMODE_ENABLE )
		SETREG32 ( reg_con, 1 << PWM_CON_OLD_MODE_OFFSET );
	else 
		return 1;
	return 0;
}
void mt_set_pwm_HiDur_hal(U32 pwm_no, U16 DurVal)  //only low 16 bits are valid
{
	unsigned long reg_HiDur;
	
	reg_HiDur = PWM_register[pwm_no]+4*PWM_HDURATION;
	OUTREG32 ( reg_HiDur, DurVal);	
}
void mt_set_pwm_LowDur_hal (U32 pwm_no, U16 DurVal)
{
	unsigned long reg_LowDur;
	reg_LowDur = PWM_register[pwm_no] + 4*PWM_LDURATION;
	OUTREG32 ( reg_LowDur, DurVal );
}
void mt_set_pwm_GuardDur_hal (U32 pwm_no, U16 DurVal)
{
	unsigned long reg_GuardDur;
	reg_GuardDur = PWM_register[pwm_no] + 4*PWM_GDURATION;
	OUTREG32 ( reg_GuardDur, DurVal );
}
void mt_set_pwm_send_data0_hal ( U32 pwm_no, U32 data )
{
	unsigned long reg_data0;
	reg_data0 = PWM_register[pwm_no] + 4 * PWM_SEND_DATA0;
	OUTREG32 ( reg_data0, data );
}
void mt_set_pwm_send_data1_hal ( U32 pwm_no, U32 data )
{
	unsigned long reg_data1;
	reg_data1 = PWM_register[pwm_no] + 4 * PWM_SEND_DATA1;
	OUTREG32 ( reg_data1, data );
}
void mt_set_pwm_wave_num_hal ( U32 pwm_no, U16 num )
{
	unsigned long reg_wave_num;
	reg_wave_num = PWM_register[pwm_no] + 4 * PWM_WAVE_NUM;
	OUTREG32 ( reg_wave_num, num );
}
void mt_set_pwm_data_width_hal ( U32 pwm_no, U16 width )
{
	unsigned long reg_data_width;
	reg_data_width = PWM_register[pwm_no] + 4 * PWM_DATA_WIDTH;
	OUTREG32 ( reg_data_width, width );
}
void mt_set_pwm_thresh_hal ( U32 pwm_no, U16 thresh )
{
	unsigned long reg_thresh;
	reg_thresh = PWM_register[pwm_no] + 4 * PWM_THRESH;
	OUTREG32 ( reg_thresh, thresh );
}
S32 mt_get_pwm_send_wavenum_hal ( U32 pwm_no )
{
	unsigned long reg_send_wavenum;
	reg_send_wavenum = PWM_register[pwm_no] + 4 * PWM_SEND_WAVENUM;
	return INREG32 ( reg_send_wavenum );
}

void mt_set_intr_enable_hal(U32 pwm_intr_enable_bit)
{
	SETREG32 ( PWM_INT_ENABLE, 1 << pwm_intr_enable_bit );
}
S32 mt_get_intr_status_hal(U32 pwm_intr_status_bit)
{
	int ret;
	ret = INREG32 ( PWM_INT_STATUS );
	ret = ( ret >> pwm_intr_status_bit ) & 0xff;
	return ret;
}
void mt_set_intr_ack_hal ( U32 pwm_intr_ack_bit )
{
	SETREG32 ( PWM_INT_ACK, 1 << pwm_intr_ack_bit );
}

void mt_set_pwm_buf0_addr_hal (U32 pwm_no, U32 addr)
{
#if 1
	unsigned long reg_buff0_addr;
        reg_buff0_addr = PWM_register[pwm_no] + 4 * PWM_BUF0_BASE_ADDR;
	OUTREG32 ( reg_buff0_addr, addr );	
#endif
}

void mt_set_pwm_buf0_size_hal( U32 pwm_no, U16 size)
{
#if 1
	unsigned long reg_buff0_size;
        reg_buff0_size = PWM_register[pwm_no] + 4* PWM_BUF0_SIZE;
        OUTREG32 ( reg_buff0_size, size );
#endif
}

void mt_set_pwm_buf1_addr_hal (U32 pwm_no, U32 addr)
{
#if 1
	unsigned long reg_buff1_addr;
        reg_buff1_addr = PWM_register[pwm_no] + 4 * PWM_BUF1_BASE_ADDR;
	OUTREG32 ( reg_buff1_addr, addr );	
#endif
}

void mt_set_pwm_buf1_size_hal( U32 pwm_no, U16 size)
{
#if 1
	unsigned long reg_buff1_size;
        reg_buff1_size = PWM_register[pwm_no] + 4* PWM_BUF1_SIZE;
        OUTREG32 ( reg_buff1_size, size );
#endif
}

void mt_set_pwm_valid_hal ( U32 pwm_no, U32 buf_valid_bit )
{
	unsigned long reg_valid;
	
	reg_valid = PWM_register[pwm_no] + 4* (PWM_VALID);

	SETREG32 ( reg_valid, 1 << buf_valid_bit );
}

void mt_set_pwm_delay_duration_hal(U32 pwm_no, U16 val)
{
	unsigned long reg_valid;
	if ( pwm_no == 3 ) {
		reg_valid = PWM3_DELAY;
	} else if ( pwm_no == 4 ) {
		reg_valid = PWM4_DELAY;
	} else if ( pwm_no == 5 ) {
		reg_valid = PWM4_DELAY;
	}

	MASKREG32 ( reg_valid, PWM_DELAY_DURATION_MASK, val );
}

void mt_set_pwm_delay_clock_hal (U32 pwm_no, U32 clksrc)
{
	unsigned long reg_valid;
	if ( pwm_no == 3 ) {
		reg_valid = PWM3_DELAY;
	} else if ( pwm_no == 4 ) {
		reg_valid = PWM4_DELAY;
	} else if ( pwm_no == 5 ) {
		reg_valid = PWM4_DELAY;
	}

	MASKREG32 (reg_valid, PWM_DELAY_CLK_MASK, clksrc );
}

void mt_pwm_dump_regs_hal(void)
{
	int i;
	unsigned long reg_val;
	reg_val = INREG32(PWM_ENABLE);
	printk("\r\n[PWM_ENABLE is:%lx]\n\r ", reg_val );
	reg_val = INREG32(PWM_CK_26M_SEL);
	printk("\r\n[PWM_26M_SEL is:%lx]\n\r ", reg_val );
	printk("<0>""peri pdn0 clock: 0x%x, bit9:pwm, bit2:pwm1, ... , bit8:pwm7.\n", INREG32(INFRA_PDN_STA0));   /*bit9:pwm, bit2:pwm1, ... , bit8:pwm7*/

	reg_val = INREG32(CK_SEL);
	printk("\r\n[CK_SEL is:%lx]\r\n",reg_val);
	reg_val = INREG32(PWM_ULTRA);
	printk("\r\n[PWM_ULTRA is:%lx]\r\n",reg_val);
	reg_val = INREG32(DCM_ENABLE);
	printk("\r\n[DCM_ENABLE is:%lx]\r\n",reg_val);
	reg_val = INREG32(DCM_CFG);
	printk("\r\n[DCM_CFG is:%lx]\r\n",reg_val);

	reg_val=INREG32(PWM3_DELAY);
	printk("\r\n[PWM3_DELAY is:%lx]\r\n",reg_val);

	for ( i = 0;  i < 4; i ++ ) 
	{
		reg_val = INREG32(PWM_register[i] + 4* PWM_CON);
		printk("\r\n[PWM%d_CON is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_HDURATION);
		printk("[PWM%d_HDURATION is:%lx]\r\n",i,reg_val);
		reg_val = INREG32(PWM_register[i] + 4* PWM_LDURATION);
		printk("[PWM%d_LDURATION is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_GDURATION);
		printk("[PWM%d_GDURATION is:%lx]\r\n",i,reg_val);

		reg_val = INREG32(PWM_register[i] + 4* PWM_BUF0_BASE_ADDR);
		printk("\r\n[PWM%d_BUF0_BASE_ADDR is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_BUF0_SIZE);
		printk("\r\n[PWM%d_BUF0_SIZE is:%lx]\r\n",i,reg_val);
		reg_val = INREG32(PWM_register[i] + 4* PWM_BUF1_BASE_ADDR);
		printk("\r\n[PWM%d_BUF1_BASE_ADDR is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_BUF1_SIZE);
		printk("\r\n[PWM%d_BUF1_SIZE is:%lx]\r\n",i+1,reg_val);

		reg_val = INREG32(PWM_register[i] + 4* PWM_SEND_DATA0);
		printk("[PWM%d_SEND_DATA0 is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_SEND_DATA1);
		printk("[PWM%d_PWM_SEND_DATA1 is:%lx]\r\n",i,reg_val);
		reg_val = INREG32(PWM_register[i] + 4* PWM_WAVE_NUM);
		printk("[PWM%d_WAVE_NUM is:%lx]\r\n", i, reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_DATA_WIDTH);
		printk("[PWM%d_WIDTH is:%lx]\r\n",i,reg_val);
		
		reg_val=INREG32(PWM_register[i]+4* PWM_THRESH);
		printk("[PWM%d_THRESH is:%lx]\r\n",i,reg_val);
		reg_val=INREG32(PWM_register[i]+4* PWM_SEND_WAVENUM);
		printk("[PWM%d_SEND_WAVENUM is:%lx]\r\n",i,reg_val);

		



		reg_val=INREG32(PWM_register[i]+4* (PWM_VALID));
		printk("\r\n[PWM%d_SEND_VALID is:%lx]\r\n",i,reg_val);

	}		
}
/*----------------------------*/
static void mt_pmic_pwm1_test(void)
{/*
	struct pwm_spec_config conf;
	printk("<0>""=============mt_pmic_pwm1_test===============\n");

	mt_set_gpio_mode(GPIO12,GPIO_MODE_02); 
	conf.pwm_no = PWM1;
	conf.mode = PWM_MODE_FIFO;
	conf.clk_div = CLK_DIV1;
	conf.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	//conf.intr = FALSE;
	conf.pmic_pad = false;
	conf.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE;
	conf.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE;
	conf.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 62;
	conf.PWM_MODE_FIFO_REGS.HDURATION = 0;
	conf.PWM_MODE_FIFO_REGS.LDURATION = 0;
	conf.PWM_MODE_FIFO_REGS.GDURATION = 0;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xf0f0f0f0;
	conf.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xf0f0f0f0;
	conf.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;	
	printk(KERN_INFO "PWM: clk_div = %x, clk_src = %x, pwm_no = %x\n", conf.clk_div, conf.clk_src, conf.pwm_no);
	pwm_set_spec_config(&conf);
	
	mt_set_gpio_mode(GPIO1,GPIO_MODE_01); 
	conf.pwm_no = PWM2;
	conf.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 30;
	pwm_set_spec_config(&conf);*/
}

static void mt_pmic_pwm6_test(void)
{
	//printk("<0>""=================mt_pmic_pwm6_test==================\n");
	
	//mt_pwm_disable(PWM1, false);
	//mt_pwm_disable(PWM7, true);
}

void pwm_debug_store_hal()
{
	//dump clock status
	printk("<0>""peri pdn0 clock: 0x%x\n", INREG32(INFRA_PDN_STA0)); 
	mt_pmic_pwm1_test();
}

void pwm_debug_show_hal()
{
	mt_pwm_dump_regs_hal();
	//mt_pmic_pwm6_test();
}

/*----------3dLCM support-----------*/
/*
 base pwm2, select pwm3&4&5 same as pwm2 or inversion of pwm2
 */
void mt_set_pwm_3dlcm_enable_hal(BOOL enable)
{
	SETREG32 ( PWM_3DLCM, 1 << PWM_3DLCM_ENABLE_OFFSET );
}

/*
 set "pwm_no" inversion of pwm base or not
 */
void mt_set_pwm_3dlcm_inv_hal(U32 pwm_no, BOOL inv)
{
	/*set "pwm_no" as auxiliary first*/
	SETREG32 ( PWM_3DLCM, 1 << (pwm_no + 16) );

	if (inv){
		SETREG32 ( PWM_3DLCM, 1 << (pwm_no + 1) );
	} else {
		CLRREG32 ( PWM_3DLCM, 1 << (pwm_no + 1) );
	}
}

void mt_set_pwm_3dlcm_base_hal(U32 pwm_no)
{
	CLRREG32 ( PWM_3DLCM, 0x7F << 8 );
	SETREG32 ( PWM_3DLCM, 1 << (pwm_no + 8) );
}

void mt_pwm_26M_clk_enable_hal(U32 enable)
{
	unsigned long reg_con;

	/* select 66M or 26M */
	reg_con = (unsigned long)PWM_CK_26M_SEL;
	if ( enable ) {
		SETREG32 ( reg_con, 1 << PWM_CK_26M_SEL_OFFSET );
	} else {
		CLRREG32 ( reg_con, 1 << PWM_CK_26M_SEL_OFFSET );
	}
}
