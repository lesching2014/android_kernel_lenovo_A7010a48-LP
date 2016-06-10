#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <mach/mt_spi.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>

#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include "mt_spi_hal.h"
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/of_address.h>

//LI He add include file for PWM
#include <mach/mt_pwm.h>
#include <mach/mt_pwm_hal_pub.h>
#include <mach/mt_dcm.h>
#include <mach/mt_vcore_dvfs.h>

#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
#define SPI_TRUSTONIC_TEE_SUPPORT
#endif

#ifdef SPI_TRUSTONIC_TEE_SUPPORT
#include <mobicore_driver_api.h>
#include <tlspi_Api.h>
#endif


static spinlock_t spi_lock;
extern int mt_set_gpio_out_base(unsigned long pin, unsigned long output);
extern void mt_pwm_26M_clk_enable(U32 enable);

#define SPIDEV_LOG(fmt, args...) printk("[SPI-UT]: [%s]:[%d]" fmt, __func__, __LINE__, ##args) 
#define SPIDEV_MSG(fmt, args...) printk(KERN_ERR  fmt, ##args )
//#define SPI_STRESS_MAX 512
#define SPI_STRESS_MAX 1 
DECLARE_COMPLETION(mt_spi_done);
static u32 stress_err = 0;
static struct task_struct *spi_concur1;
static struct task_struct *spi_concur2;
static struct task_struct *spi_concur3;
static struct task_struct *spi_concur4;

static struct spi_transfer stress_xfer[SPI_STRESS_MAX];
static struct spi_transfer stress_xfer_con[SPI_STRESS_MAX];

static struct spi_message stress_msg[SPI_STRESS_MAX];
static struct spi_device *spi_test;

static int spi_setup_xfer(struct device *dev, struct spi_transfer *xfer, u32 len, u32 flag)
{
	u32 tx_buffer = 0x12345678;
	u32 cnt, i;
#if 0
	u8 *p;
#endif
#define SPI_CROSS_ALIGN_OFFSET 1008	
	xfer->len=len;

	xfer->tx_buf = ( u32 * ) kzalloc ( len, GFP_KERNEL);
	xfer->rx_buf = ( u32 * ) kzalloc ( len, GFP_KERNEL);
	
	if((xfer->tx_buf == NULL) || (xfer->rx_buf == NULL)  )
			return -1;
	
	cnt = (len%4)?(len/4 + 1):(len/4);

	if(flag == 0){
		for ( i = 0; i < cnt; i++ )
			*(( u32 * )xfer->tx_buf + i) = tx_buffer;
	}else if(flag == 1){
		for ( i = 0; i < cnt; i++ )
			*(( u32 * )xfer->tx_buf + i) = tx_buffer + i;
	}else if(flag == 2){//cross 1 K boundary
		if(len < 2048)
			return -EINVAL;
		for ( i = 0; i < cnt; i++ )
			*(( u32 * )xfer->tx_buf + i) = tx_buffer + i;

		xfer->tx_dma = dma_map_single(dev, (void *)xfer->tx_buf, 	
				xfer->len,  DMA_TO_DEVICE);				
		if (dma_mapping_error(dev, xfer->tx_dma)) {
			SPIDEV_LOG("dma mapping tx_buf error.\n");
			return -ENOMEM;	
		}
		xfer->rx_dma = dma_map_single(dev, (void *)xfer->rx_buf, 	
				xfer->len,  DMA_TO_DEVICE);				
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			SPIDEV_LOG("dma mapping rx_buf error.\n");
			return -ENOMEM;	
		}
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		SPIDEV_LOG("Transfer addr:Tx:0x%llx, Rx:0x%llx, before\n", xfer->tx_dma,xfer->rx_dma);
#else
		SPIDEV_LOG("Transfer addr:Tx:0x%x, Rx:0x%x, before\n", xfer->tx_dma,xfer->rx_dma);
#endif
		xfer->len=32;
#if 0
		p= (u8*)xfer->tx_dma;
		xfer->tx_dma = (u32)(p+ SPI_CROSS_ALIGN_OFFSET);
		p= (u8*)xfer->rx_dma;
		xfer->rx_dma = (u32)(p+ SPI_CROSS_ALIGN_OFFSET);

		p= (u8*)xfer->tx_buf;
		xfer->tx_buf =(u32*)(p+ SPI_CROSS_ALIGN_OFFSET);
		p= (u8*)xfer->rx_buf;
		xfer->rx_buf =(u32*)(p+ SPI_CROSS_ALIGN_OFFSET);
#else
		xfer->tx_dma += SPI_CROSS_ALIGN_OFFSET;
		xfer->rx_dma += SPI_CROSS_ALIGN_OFFSET;

		xfer->tx_buf += SPI_CROSS_ALIGN_OFFSET;
		xfer->rx_buf += SPI_CROSS_ALIGN_OFFSET;
#endif
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		SPIDEV_LOG("Transfer addr:Tx:0x%llx, Rx:0x%llx\n", xfer->tx_dma,xfer->rx_dma);
#else
		SPIDEV_LOG("Transfer addr:Tx:0x%x, Rx:0x%x\n", xfer->tx_dma,xfer->rx_dma);
#endif
		SPIDEV_LOG("Transfer addr:Tx:0x%p, Rx:0x%p\n", xfer->tx_buf,xfer->rx_buf);
		//for ( i = 0; i < cnt; i++ )
		//	*(( u32 * )xfer->tx_buf + i) = tx_buffer + i;
	}else{
	return -EINVAL;
	}
	return 0;
	
}
static inline int 
is_last_xfer(struct spi_message *msg, struct spi_transfer *xfer)
{
	return msg->transfers.prev == &xfer->transfer_list;
}

static int spi_recv_check(struct spi_message *msg)
{

	struct spi_transfer *xfer;
	u32 cnt, i, err=0;
	
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {

		if (!xfer) {
			SPIDEV_MSG ( "rv msg is NULL.\n" );
			return -1;
		}
//			SPIDEV_LOG("xfer:0x%p, length is:%d\n", xfer,xfer->len);
		cnt = (xfer->len%4)?(xfer->len/4 + 1):(xfer->len/4);
		for(i=0; i<cnt; i++){
			if(*( ( u32 * ) xfer->rx_buf + i ) != *( ( u32 * ) xfer->tx_buf + i )){ 					
				SPIDEV_LOG("tx xfer %d is:%.8x\n",i,  *( ( u32 * ) xfer->tx_buf + i ) );
				SPIDEV_LOG("rx xfer %d is:%.8x\n",i,  *( ( u32 * ) xfer->rx_buf + i ) ); 
				//SPIDEV_LOG("tx xfer %d dma is:%.8x\n",i,  ( ( u32 * )xfer->tx_dma + i ) );
				//SPIDEV_LOG("rx xfer %d dma is:%.8x\n",i,  ( ( u32 * )xfer->rx_dma + i ) );
				SPIDEV_LOG("\n");
				err++;
			}
		}
		//memset(xfer->tx_buf,0,xfer->len);
		//memset(xfer->rx_buf,0,xfer->len);
		kfree(xfer->tx_buf);
		kfree(xfer->rx_buf);
	}
	
		SPIDEV_LOG("Message:0x%p,error %d,actual xfer length is:%d\n", msg,err, msg->actual_length);

	return err;
	
}
static int spi_recv_check_all(struct spi_device *spi, struct spi_message *msg)
{

	struct spi_transfer *xfer;
	u32 i, err=0;
	int j;
	u8 rec_cac = 0;
	
	struct mt_chip_conf *chip_config;
	chip_config = (struct mt_chip_conf *) spi->controller_data;
	
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {

		if (!xfer) {
			SPIDEV_MSG ( "rv msg is NULL.\n" );
			return -1;
		}
//			SPIDEV_LOG("xfer:0x%p, length is:%d\n", xfer,xfer->len);
//		cnt = (xfer->len%4)?(xfer->len/4 + 1):(xfer->len/4);
		for(i=0; i<xfer->len; i++){
			if(chip_config->tx_mlsb ^ chip_config->rx_mlsb){
				rec_cac = 0;
				for(j=7;j>=0;j--){
					rec_cac |= ((*((u8 *)xfer->tx_buf + i) & (1<<j)) >> j)<< (7-j);					
				}				
				//SPIDEV_LOG("rec_cac %x :%x\n",rec_cac,	*( ( u8 * ) xfer->tx_buf + i ) );
				
			}else{
				rec_cac = *( ( u8 * ) xfer->tx_buf + i );
			}
			if(*( ( u8 * ) xfer->rx_buf + i ) != rec_cac ){ 					
				SPIDEV_LOG("tx xfer %d is:%x\n",i,  *( ( u8 * ) xfer->tx_buf + i ) );
				SPIDEV_LOG("rx xfer %d is:%x\n",i,  *( ( u8 * ) xfer->rx_buf + i ) ); 
				err++;
			}
		}
		kfree(xfer->tx_buf);
		kfree(xfer->rx_buf);
	}
	
		SPIDEV_LOG("Message:0x%p,error %d,actual xfer length is:%d\n", msg,err, msg->actual_length);

	return err;
	
}

static void spi_complete(void *arg)
{
	static u32 i = 0;
	stress_err += spi_recv_check((struct spi_message *)arg);
	if(stress_err > 0){
		
		SPIDEV_LOG("Message transfer err:%d\n", stress_err);
//		BUG();
	}
		
//		SPIDEV_LOG("Message:%d\n", i);
	if(++i == SPI_STRESS_MAX){
		i = 0;
		complete(&mt_spi_done);
	}
}


static int threadfunc1(void *data)
{
	struct spi_transfer transfer;
	struct spi_message msg;
	struct spi_device *spi=(struct spi_device *) data;
	u32 len= 8;
	int ret;	

	while(1){
		spi_message_init(&msg);
		
		set_current_state(TASK_INTERRUPTIBLE);
		if(kthread_should_stop()) break;
		
		spi_setup_xfer(&spi->dev,&transfer,len,0);
		spi_message_add_tail(&transfer, &msg);
		ret = spi_sync(spi, &msg);
		if(ret < 0){
			stress_err ++;
			SPIDEV_LOG("Message transfer err:%d\n", ret);
		}else{
			ret = spi_recv_check(&msg);
			if(ret != 0){
				stress_err += ret;					
				SPIDEV_LOG("thread Message transfer err:%d\n", ret);
			}
		}
		schedule_timeout(HZ);
	}
	return 0;
}
static int threadfunc2(void *data)
{
	struct spi_transfer transfer;
	struct spi_message msg;	
	struct spi_device *spi=(struct spi_device *) data;
	
	u32 len= 128;
	int ret;

	while(1){
		spi_message_init(&msg);
		
		set_current_state(TASK_INTERRUPTIBLE);
		if(kthread_should_stop()) break;
		
		spi_setup_xfer(&spi->dev,&transfer,len,0);
		spi_message_add_tail(&transfer, &msg);
		ret = spi_sync(spi, &msg);
		if(ret < 0){
			stress_err ++;
			SPIDEV_LOG("Message transfer err:%d\n", ret);
		}else{
			ret = spi_recv_check(&msg);
			if(ret != 0){
				stress_err += ret;					
				SPIDEV_LOG("thread Message transfer err:%d\n", ret);
			}
		}
		schedule_timeout(HZ);
	}
	return 0;
}
static int threadfunc3(void *data)
{
	//struct spi_transfer transfer;
	struct spi_message msg;	
	struct spi_device *spi=(struct spi_device *) data;
	static struct spi_message *p;
	u32 len= 64;
	int ret;
	u16 i;
	
	while(1){
		spi_message_init(&msg);
		
		set_current_state(TASK_INTERRUPTIBLE);
		if(kthread_should_stop()) break;
		
		p = stress_msg;
		for(i=0;i<SPI_STRESS_MAX;i++){
			spi_message_init(p);
			ret = spi_setup_xfer(&spi->dev,&stress_xfer[i],len,0);
			if(ret != 0){									
				SPIDEV_LOG("xfer set up err:%d\n", ret);
			}
			spi_message_add_tail(&stress_xfer[i], p);
		
			p->complete = spi_complete;
			p->context = p;
			ret = spi_async(spi, p);
			if(ret < 0){
				SPIDEV_LOG("Message %d transfer err:%d\n",i, ret);
			}
			p ++;
		}
		
		wait_for_completion(&mt_spi_done);

		schedule_timeout(5*HZ);
	}
	return 0;
}
static int threadfunc4(void *data)
{
	//struct spi_transfer transfer;
	struct spi_message msg;	
	struct spi_device *spi=(struct spi_device *) data;

	u32 len= 32;
	int ret;
	u16 i;
	
	while(1){
		spi_message_init(&msg);
		
		set_current_state(TASK_INTERRUPTIBLE);
		if(kthread_should_stop()) break;
		
	
		for(i=0;i<SPI_STRESS_MAX;i++){			
			ret = spi_setup_xfer(&spi->dev,&stress_xfer_con[i],len,1);
			if(ret != 0){									
				SPIDEV_LOG("Message set up err:%d\n", ret);
			}
			spi_message_add_tail(&stress_xfer_con[i], &msg);
			
		}
		ret = spi_sync(spi, &msg);
		if(ret < 0){
			SPIDEV_LOG("Message transfer err:%d\n", ret);
		}else{
			ret = spi_recv_check(&msg);
			if(ret != 0){
				ret -= ret;
				stress_err += ret;
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			}
//				SPIDEV_LOG("Message multi xfer stress pass\n");
		}

		schedule_timeout(2*HZ);
	}
	return 0;
}

extern int mt_set_gpio_dir_base(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_mode_base(unsigned long pin, unsigned long mode);
void spi_gpio_pin_init()
{
	mt_set_gpio_mode(21|0x80000000,0);
	mt_set_gpio_mode(59|0x80000000,0);
	mt_set_gpio_dir_base(21, 1); 
	mt_set_gpio_dir_base(59, 1); 

}
static void pwm_pin_init()
{
	mt_set_gpio_mode(21|0x80000000,2);
	mt_set_gpio_mode(59|0x80000000,5);

}
static unsigned char byte_reverse(unsigned char data)
{
	unsigned char data_rev ,i ,temp;
	for(i = 0; i <= 7; i++)
	{
		temp = (data>>i)&0x01 ;
		data_rev = (data_rev<<1)|temp ;
		
	}
	return data_rev ;
}

static void byte_two(const void *val_two,const void *val,size_t val_len)
{
	unsigned char *data_char= val ;
	unsigned char *data_two =(unsigned char *) val_two ;
	unsigned short data_short = 0;
	int i,j;
	unsigned char data;
	unsigned char temp ;
	for(j=0; j < val_len; j++)
	{
		data = *(unsigned char *)(data_char+j);
		data_short = 0 ;
		for(i = 8; i > 0; i--)
		{
			temp = (data>>(i-1))&0x01 ;
			if(temp == 1)
				data_short = data_short|(0x03<<(2*(i-1)));	
			
		}
		data_two[2*j] = (unsigned char)((data_short>>8)&0xFF) ;
		data_two[2*j+1]=  (unsigned char)(data_short&0xFF) ;
	}

}
static unsigned char pwm_data_jy[1000*1024];
extern S32 mt_set_pwm_disable(S32 pwm);
 /* lenovo-sw zhangrc2 close pwm2,pwm3 when no use spi begin 2015-7-3 begin */
extern void mt_pwm_disable(U32 pwm_no, BOOL pmic_pad);
 /* lenovo-sw zhangrc2 close pwm2,pwm3 when no use spi begin 2015-7-3 end */
size_t pwm_write( struct device *dev ,const void *val, size_t val_len)
{
	//printk(" pwm_write length val_len = %lx \n" ,val_len);

	size_t pwm_flag = val_len;
	S32 config_flag = 0 ;

	pwm_pin_init();
//printk("mt_get_gpio_mode(59|0x80000000)_2: %d\n" ,mt_get_gpio_mode(59|0x80000000));
//printk("mt_get_gpio_mode(21|0x80000000)_2: %d\n" ,mt_get_gpio_mode(21|0x80000000));
	// Memory Setting //
	
	int clk_len = val_len*2 ;

	void *virt;
	dma_addr_t phys;
	int data_len = val_len ;
	struct platform_device *pdev = to_platform_device(dev);
	virt = dma_alloc_coherent(&pdev->dev, data_len, &phys, GFP_KERNEL);
	if (virt == NULL) {
		//printk("dma memory allocte error!!!\n");
		return -ENOMEM;
	}

	unsigned char *membuff_data = virt;
	unsigned char *raw_data = val;

	memcpy(pwm_data_jy,(unsigned char *)raw_data,data_len);
	
	int i;
	for (i = 0; i < data_len; i++){

		membuff_data[i] = byte_reverse(pwm_data_jy[i]); 
	}
	
	//clk use fifio mode
	mt_set_pwm_disable(2);
	mt_set_pwm_disable(3);
	
	struct pwm_spec_config conf2;
	conf2.pwm_no = 2;  
	conf2.mode = PWM_MODE_FIFO;
	conf2.clk_div = CLK_DIV1;
	conf2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	conf2.PWM_MODE_FIFO_REGS.IDLE_VALUE = IDLE_FALSE;
	conf2.PWM_MODE_FIFO_REGS.GUARD_VALUE = GUARD_FALSE;
	conf2.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;  //set 111111
	/* lenovo-sw zhangrc2 compatible gpio-spi and spi */
	conf2.PWM_MODE_FIFO_REGS.HDURATION = 3;
	conf2.PWM_MODE_FIFO_REGS.LDURATION = 3;
	/* lenovo-sw zhangrc2 compatible gpio-spi and spi */
	conf2.PWM_MODE_FIFO_REGS.GDURATION = 0;
	conf2.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xaaaaaaaa;
	conf2.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xaaaaaaaa;
	conf2.PWM_MODE_FIFO_REGS.WAVE_NUM  = (clk_len/8);



	struct pwm_spec_config conf_3;
	conf_3.mode = PWM_MODE_MEMORY;
	conf_3.pwm_no = 3;
	conf_3.clk_div = CLK_DIV1;
	conf_3.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	conf_3.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE;
	conf_3.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE;
	/* lenovo-sw zhangrc2 compatible gpio-spi and spi */
	conf_3.PWM_MODE_MEMORY_REGS.HDURATION = 7;  //8M
	conf_3.PWM_MODE_MEMORY_REGS.LDURATION = 7;
	/* lenovo-sw zhangrc2 compatible gpio-spi and spi */
	conf_3.PWM_MODE_MEMORY_REGS.GDURATION = 0;
	conf_3.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1;
	conf_3.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31;
	conf_3.PWM_MODE_MEMORY_REGS.BUF0_SIZE = (data_len/4 -1);
	conf_3.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = phys;
	conf_3.PWM_MODE_MEMORY_REGS.PWM_MEMORY_DUR = 2;    // offset 
	conf_3.PWM_MODE_MEMORY_REGS.PWM_MEMORY_CLK = 0;   // delay clock
	
	config_flag = pwm_set_spec_config_for_spi(&conf2, &conf_3);	
	if(config_flag == -1)
	{
		pwm_flag = 0 ;
	}

	spi_gpio_pin_init();
    dma_free_coherent(&pdev->dev, data_len, virt,phys);
    /* lenovo-sw zhangrc2 close pwm2,pwm3 when no use spi begin 2015-7-3 begin */
	mt_pwm_disable(2,0);
	mt_pwm_disable(3,0);
     /* lenovo-sw zhangrc2 close pwm2,pwm3 when no use spi begin 2015-7-3 end */
#ifdef PWM_DEBUG_SPI
	printk(" -----pwm_write config_flag = %d ------\n" ,config_flag);
#endif
	return pwm_flag ;
	
}


#if 0
void pwm_write( struct device *dev ,const void *val, size_t val_len)
{
//	printk(" pwm_write length val_len = %lx \n" ,val_len);
    unsigned long flags;
#ifdef PWM_DEBUG_SPI
	printk("enter pwm_write \n" );
#endif
//printk("mt_get_gpio_mode(59|0x80000000)_1: %d\n" ,mt_get_gpio_mode(59|0x80000000));
//printk("mt_get_gpio_mode(21|0x80000000)_1: %d\n" ,mt_get_gpio_mode(21|0x80000000));
	pwm_pin_init();
//printk("mt_get_gpio_mode(59|0x80000000)_2: %d\n" ,mt_get_gpio_mode(59|0x80000000));
//printk("mt_get_gpio_mode(21|0x80000000)_2: %d\n" ,mt_get_gpio_mode(21|0x80000000));
	// Memory Setting //
	void *virt_2;
	dma_addr_t phys_2;
	int data_len = val_len*2 ;
	//struct platform_device *pdev_2 = to_platform_device(dev);
	struct platform_device *pdev = to_platform_device(dev);
	virt_2 = dma_alloc_coherent(&pdev->dev, data_len+4, &phys_2, GFP_KERNEL);
	if (virt_2 == NULL) {
		printk("dma memory allocte error!!!\n");
		return -ENOMEM;
	}
	
	dma_addr_t phys;
	void *virt;
	int clk_len = val_len*2 ;
	//struct platform_device *pdev = to_platform_device(dev);
	virt = dma_alloc_coherent(&pdev->dev, clk_len+4, &phys, GFP_KERNEL);  // 分配500KB+4Byes内存空间
	if (virt == NULL) {
		printk("dma memory allocte error!!!\n");
		return -ENOMEM;
	}
	unsigned char *membuff_clk = virt;
	unsigned char *membuff_data = virt_2;
	unsigned char *raw_data = val;

    memset(pwm_data_jy,data_len,0x01);
	byte_two(pwm_data_jy,(unsigned char *)raw_data,val_len);
	
	//unsigned char *membuff = val;
	int i;
	// PWM Clock Channel buffer1
	
	for (i = 0; i < data_len; i++){

		membuff_data[i] = byte_reverse(pwm_data_jy[i]); 
	}
    membuff_data[data_len] = 0x00;
    membuff_data[data_len+1] = 0x00; 
    membuff_data[data_len+2] = 0x00; 
    membuff_data[data_len+3] = 0x00; 
    data_len = data_len +4 ; 
	

    membuff_clk[0] = 0x00; 
    membuff_clk[1] = 0x00; 
    clk_len = clk_len +4 ;
	for (i = 2; i < clk_len-2 ; i++){
		membuff_clk[i] = 0xaa; 

	}	
    membuff_clk[clk_len-2] = 0x00;
    membuff_clk[clk_len-1] = 0x00;
/*
	for (i = 0; i < clk_len ; i++){
		membuff_clk[i] = 0xaa; 

	}	*/

	struct pwm_spec_config conf_1 ,conf_2 ;

	if(clk_len <= 64000*4) //CH2 and CH3  memory mode
	{
		// PWM Con-Register Setting //
		// clock port setting
		conf_1.mode = PWM_MODE_MEMORY;
		conf_1.pwm_no = 2;
		conf_1.clk_div = CLK_DIV1;
		conf_1.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_1.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_1.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_1.PWM_MODE_MEMORY_REGS.HDURATION = 4;
		conf_1.PWM_MODE_MEMORY_REGS.LDURATION = 4;
		conf_1.PWM_MODE_MEMORY_REGS.GDURATION = 0;
		conf_1.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1;
		conf_1.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31;
		conf_1.PWM_MODE_MEMORY_REGS.BUF0_SIZE = clk_len/4-1  ; // 0~63999  1表示32bit的数据量
		conf_1.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = phys;
		
		// data port setting
		conf_2.mode = PWM_MODE_MEMORY;
		conf_2.pwm_no = 3;
		conf_2.clk_div = CLK_DIV1;
		conf_2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_2.PWM_MODE_MEMORY_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_2.PWM_MODE_MEMORY_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_2.PWM_MODE_MEMORY_REGS.HDURATION = 4;
		conf_2.PWM_MODE_MEMORY_REGS.LDURATION = 4;
		conf_2.PWM_MODE_MEMORY_REGS.GDURATION = 0;
		conf_2.PWM_MODE_MEMORY_REGS.WAVE_NUM = 1;
		conf_2.PWM_MODE_MEMORY_REGS.STOP_BITPOS_VALUE = 31;
		conf_2.PWM_MODE_MEMORY_REGS.BUF0_SIZE = data_len/4-1 ;
		conf_2.PWM_MODE_MEMORY_REGS.BUF0_BASE_ADDR = phys_2;
		
		conf_2.PWM_MODE_MEMORY_REGS.PWM_MEMORY_DUR = 2 + 80 ;//64 ;//48;  // delay
		conf_2.PWM_MODE_MEMORY_REGS.PWM_MEMORY_CLK = 0;   // delay clock
		
	}
	else if(clk_len > 64000*4)//CH2 and CH3  memory mode
	{		
		// PWM Con-Register Setting //
		// clock port setting
		conf_1.mode = PWM_MODE_RANDOM;
		conf_1.pwm_no = 2;
		conf_1.clk_div = CLK_DIV1;
		conf_1.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_1.PWM_MODE_RANDOM_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_1.PWM_MODE_RANDOM_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_1.PWM_MODE_RANDOM_REGS.HDURATION = 4;
		conf_1.PWM_MODE_RANDOM_REGS.LDURATION = 4;
		conf_1.PWM_MODE_RANDOM_REGS.GDURATION = 0;
		conf_1.PWM_MODE_RANDOM_REGS.WAVE_NUM = 1;
		conf_1.PWM_MODE_RANDOM_REGS.STOP_BITPOS_VALUE = 31;
		conf_1.PWM_MODE_RANDOM_REGS.BUF0_SIZE = 63999;	 // 0~63999  1表示32bit的数据量
		conf_1.PWM_MODE_RANDOM_REGS.BUF0_BASE_ADDR = phys;
		conf_1.PWM_MODE_RANDOM_REGS.BUF1_SIZE = (clk_len/4)-64000;
		conf_1.PWM_MODE_RANDOM_REGS.BUF1_BASE_ADDR = phys + 64000;

		// data port setting
		conf_2.mode = PWM_MODE_RANDOM;
		conf_2.pwm_no = 3;
		conf_2.clk_div = CLK_DIV1;
		conf_2.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		conf_2.PWM_MODE_RANDOM_REGS.IDLE_VALUE = IDLE_FALSE;
		conf_2.PWM_MODE_RANDOM_REGS.GUARD_VALUE = GUARD_FALSE;
		conf_2.PWM_MODE_RANDOM_REGS.HDURATION = 4;
		conf_2.PWM_MODE_RANDOM_REGS.LDURATION = 4;
		conf_2.PWM_MODE_RANDOM_REGS.GDURATION = 0;
		conf_2.PWM_MODE_RANDOM_REGS.WAVE_NUM = 1;
		conf_2.PWM_MODE_RANDOM_REGS.STOP_BITPOS_VALUE = 31;
		conf_2.PWM_MODE_RANDOM_REGS.BUF0_SIZE = 63999;
		conf_2.PWM_MODE_RANDOM_REGS.BUF0_BASE_ADDR = phys_2;
		conf_2.PWM_MODE_RANDOM_REGS.BUF1_SIZE = (data_len/4)-64000;
		conf_2.PWM_MODE_RANDOM_REGS.BUF1_BASE_ADDR = phys + 64000;
		
		conf_2.PWM_MODE_RANDOM_REGS.PWM_RANDOM_DUR = 2 + 80;//48;  // delay
		conf_2.PWM_MODE_RANDOM_REGS.PWM_RANDOM_CLK = 0;   // delay clock
		
	}else{
		printk("DATA length is bigger than 256KB,and CLK length is bigger than 512KB!\n");
	
	}	
	
	pwm_set_spec_config_for_spi(&conf_1, &conf_2);
	spi_gpio_pin_init();

	dma_free_coherent(&pdev->dev, data_len, virt_2,phys_2);
    dma_free_coherent(&pdev->dev, data_len, virt,phys);
}
#endif

#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
extern int secspi_session_open(void);
extern int secspi_execute(u32 cmd, tciSpiMessage_t *param);
#endif

static ssize_t spi_store(struct device *dev, 
	struct device_attribute *attr, 
	const char *buf, size_t count)
{
	struct spi_device *spi;

	struct mt_chip_conf *chip_config;
	
	u32 setuptime, holdtime, high_time, low_time;
	u32 cs_idletime, ulthgh_thrsh;
	int cpol, cpha,tx_mlsb, rx_mlsb, tx_endian, sample_sel, cs_pol;
	int rx_endian, com_mod, pause, finish_intr;
	int deassert, tckdly, ulthigh;
	
	spi = container_of(dev, struct spi_device, dev);

	SPIDEV_LOG("SPIDEV name is:%s\n", spi->modalias);

	chip_config = (struct mt_chip_conf *) spi->controller_data;

	if (!chip_config) {
		SPIDEV_LOG ( "chip_config is NULL.\n");
		chip_config = kzalloc ( sizeof ( struct mt_chip_conf ), GFP_KERNEL );
		if ( !chip_config ) 
			return -ENOMEM;
	}	
#ifdef CONFIG_TRUSTONIC_TEE_SUPPORT
	if (!strncmp(buf, "-1", 2 ) ) { /*TRANSFER*/
		    SPIDEV_MSG("start to access TL SPI driver.\n");
			secspi_session_open();
			secspi_execute(1, NULL);
			SPIDEV_MSG("secspi_execute 1 finished!!!\n");
	}else if(!strncmp(buf, "-2", 2 ) ) { /*HW CONFIG*/
		    SPIDEV_MSG("start to access TL SPI driver.\n");
			secspi_session_open();
			secspi_execute(2, NULL);
			SPIDEV_MSG("secspi_execute 2 finished!!!\n");
	}else if(!strncmp(buf, "-3", 2 ) ) { /*DEBUG*/
		    SPIDEV_MSG("start to access TL SPI driver.\n");
			secspi_session_open();
			secspi_execute(3, NULL);
			SPIDEV_MSG("secspi_execute 3 finished!!!\n");
	}else if(!strncmp(buf, "-4", 2 ) ) { /*TEST*/
		    SPIDEV_MSG("start to access TL SPI driver.\n");
			secspi_session_open();
			secspi_execute(4, NULL);
			SPIDEV_MSG("secspi_execute 4 finished!!!\n");
#else
	if (!strncmp(buf, "-h", 2 ) ) {
		SPIDEV_MSG("Please input the parameters for this device.\n");
#endif
	} else if ( !strncmp(buf, "-w", 2 ) ) {
		buf += 3;
		if (!buf) {
			SPIDEV_LOG("buf is NULL.\n");
			goto out;
		}
		if (!strncmp(buf, "setuptime=",10) && (1 == sscanf(buf+10, "%d", &setuptime))) {
			SPIDEV_MSG("setuptime is:%d\n", setuptime);
			chip_config->setuptime=setuptime;
		}else if (!strncmp(buf, "holdtime=", 9)&&(1==sscanf(buf+9, "%d", &holdtime))) {
			SPIDEV_MSG("Set holdtime is:%d\n", holdtime);
			chip_config->holdtime=holdtime;	
		}else if (!strncmp(buf, "high_time=", 10)&&(1==sscanf(buf+10, "%d", &high_time))) {
			SPIDEV_MSG("Set high_time is:%d\n", high_time);
			chip_config->high_time=high_time;	
		}else if (!strncmp(buf, "low_time=", 9)&&(1==sscanf(buf+9, "%d", &low_time))) {
			SPIDEV_MSG("Set low_time is:%d\n", low_time);
			chip_config->low_time=low_time;
		}else if (!strncmp(buf, "cs_idletime=", 12)&&(1==sscanf(buf+12, "%d", &cs_idletime))) {
			SPIDEV_MSG("Set cs_idletime is:%d\n", cs_idletime);
			chip_config->cs_idletime=cs_idletime;	
		}else if (!strncmp(buf, "ulthgh_thrsh=", 13)&&(1==sscanf(buf+13, "%d", &ulthgh_thrsh))) {
			SPIDEV_MSG("Set slwdown_thrsh is:%d\n", ulthgh_thrsh);
			chip_config->ulthgh_thrsh=ulthgh_thrsh; 
		}else if (!strncmp(buf, "cpol=", 5) && (1 == sscanf(buf+5, "%d", &cpol))){
			SPIDEV_MSG("Set cpol is:%d\n", cpol);
			chip_config->cpol = cpol;
		}else if (!strncmp(buf, "cpha=", 5) && (1 == sscanf(buf+5, "%d", &cpha))) {
			SPIDEV_MSG("Set cpha is:%d\n", cpha);
			chip_config->cpha = cpha;
		}else if (!strncmp(buf, "tx_mlsb=", 8)&&(1==sscanf(buf+8, "%d", &tx_mlsb))) {
			SPIDEV_MSG("Set tx_mlsb is:%d\n", tx_mlsb);
			chip_config->tx_mlsb=tx_mlsb;	
		}else if (!strncmp(buf, "rx_mlsb=", 8)&&(1==sscanf(buf+8, "%d", &rx_mlsb))) {
			SPIDEV_MSG("Set rx_mlsb is:%d\n", rx_mlsb);
			chip_config->rx_mlsb=rx_mlsb;	
		}else if (!strncmp(buf, "tx_endian=", 10)&&(1==sscanf(buf+10, "%d", &tx_endian))) {
			SPIDEV_MSG("Set tx_endian is:%d\n", tx_endian);
			chip_config->tx_endian=tx_endian;	
		}else if (!strncmp(buf, "rx_endian=", 10)&&(1==sscanf(buf+10, "%d", &rx_endian))) {
			SPIDEV_MSG("Set rx_endian is:%d\n", rx_endian);
			chip_config->rx_endian=rx_endian;	
		}else if (!strncmp(buf, "com_mod=", 8)&&(1==sscanf(buf+8, "%d", &com_mod))) {
			chip_config->com_mod=com_mod;
			SPIDEV_MSG("Set com_mod is:%d\n", com_mod);
		}else if (!strncmp(buf, "pause=", 6)&&(1==sscanf(buf+6, "%d", &pause))) {
			SPIDEV_MSG("Set pause is:%d\n", pause);
			chip_config->pause=pause;
		}else if (!strncmp(buf, "finish_intr=", 12)&&(1==sscanf(buf+12, "%d", &finish_intr))) {
			SPIDEV_MSG("Set finish_intr is:%d\n", finish_intr);
			chip_config->finish_intr=finish_intr;
		}else if (!strncmp(buf, "deassert=", 9)&&(1==sscanf(buf+9, "%d", &deassert))) {
			SPIDEV_MSG("Set deassert is:%d\n", deassert);
			chip_config->deassert=deassert;	
		}else if (!strncmp(buf, "ulthigh=", 8 ) && ( 1 == sscanf(buf+8, "%d", &ulthigh))) {
			SPIDEV_MSG("Set ulthigh is:%d\n", ulthigh);	
			chip_config->ulthigh=ulthigh;
		}else if (!strncmp(buf, "tckdly=",7) && ( 1 == sscanf(buf+7, "%d", &tckdly))) {
			SPIDEV_MSG("Set tckdly is:%d\n", tckdly);
			chip_config->tckdly=tckdly;
		}else if (!strncmp(buf, "sample_sel=", 11 ) && ( 1 == sscanf(buf+11, "%d", &sample_sel))) {
			SPIDEV_MSG("Set sample_sel is:%d\n", sample_sel);	
			chip_config->sample_sel=sample_sel;
		}else if (!strncmp(buf, "cs_pol=",7) && ( 1 == sscanf(buf+7, "%d", &cs_pol))) {
			SPIDEV_MSG("Set cs_pol is:%d\n", cs_pol);
			chip_config->cs_pol=cs_pol;
		}else {
			SPIDEV_LOG("Wrong parameters.\n");
			goto out;
		}
		spi->controller_data = chip_config;
//			spi_setup(spi);
			
	}
out:
	return count;
}

static int tc_spi_cross_1k(struct spi_device *spi)
{
	int ret=0;
	//struct spi_transfer transfer;
	//struct spi_message msg;
	//spi_setup_xfer(&transfer,2048,2);
	//spi_message_add_tail ( &transfer, &msg );
	//msg.is_dma_mapped=1;
	//ret = spi_sync ( spi_test, &msg );
	//if(ret < 0){
	//	SPIDEV_LOG("Message transfer err:%d\n", ret);
	//}else{
	//	ret = spi_recv_check(&msg);
	//}
	return ret;
}
static ssize_t 
spi_msg_store(struct device *dev, struct device_attribute *attr, 
				const char *buf, size_t count)
{
	//struct spi_transfer *xfer;
	//struct spi_message *msg_str;
	struct spi_message *p;

	int ret = 0;
	struct spi_device *spi;

	struct spi_transfer transfer;
	struct spi_transfer transfer2;
	struct spi_transfer transfer3;
	struct spi_message msg;
	struct mt_chip_conf *chip_config;

	u32 i, len=4;
	u32 tx_buffer = 0x12345678;
	u32 rx_buffer = 0xaaaaaaaa;

	transfer.tx_buf = &tx_buffer;
	transfer.rx_buf = &rx_buffer;
	transfer.len = 4;	
	
	spi = container_of ( dev, struct spi_device, dev );

	if(unlikely(!spi)){
		SPIDEV_LOG ( "spi device is invalid\n" );
		goto out;
	}
	if(unlikely(!buf)){
		SPIDEV_LOG("buf is invalid\n" );
		goto out;
	}
	spi_message_init(&msg);
	
	if( !strncmp ( buf, "-h", 2 ) ) {
		SPIDEV_MSG("Please input the message \
			of this device to send and receive. \n" );
	}else if(!strncmp(buf, "-w", 2)) {
		buf += 3;
		if (!buf) {
			SPIDEV_LOG("Parameter is not enough.\n");
			goto out;
		}
		if (!strncmp ( buf, "len=", 4 ) && 1 == sscanf ( buf+4, "%d", &len ) ) {
			spi_setup_xfer(&spi->dev,&transfer,len,0);
			spi_message_add_tail ( &transfer, &msg );
			ret = spi_sync ( spi, &msg );
			if(ret < 0){
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			}else{
				ret = spi_recv_check_all(spi,&msg);
				if(ret != 0){
					ret = -ret;					
					SPIDEV_LOG("Message transfer err:%d\n", ret);
					goto out;
				}
			}				
		}
	}else if(!strncmp(buf, "-func", 5)) {
		buf += 6;
		if (!buf) {
			SPIDEV_LOG("Parameter is not enough.\n");
			goto out;
		}
		if (!strncmp ( buf, "len=", 4 ) && 1 == sscanf ( buf+4, "%d", &len ) ) {
			spi_setup_xfer(&spi->dev,&transfer,len,1);
			spi_message_add_tail ( &transfer, &msg );
			ret = spi_sync ( spi, &msg );
			if(ret < 0){
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			}else{
				ret = spi_recv_check(&msg);
				if(ret != 0){
					ret = -ret;					
					SPIDEV_LOG("Message transfer err:%d\n", ret);
					goto out;
				}
			}				
		}
		if (!strncmp ( buf, "cross", 5 )) {
			ret = tc_spi_cross_1k(spi);
			if(ret < 0){
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			}else if(ret != 0){
				ret = -ret;					
				SPIDEV_LOG("Message transfer err:%d\n", ret);
				goto out;
			}
		}				
	}else if(!strncmp(buf, "-err", 4)) {
		buf += 5;
		chip_config = (struct mt_chip_conf *) spi->controller_data;
		
		if(!strncmp(buf, "buf", 3)) {	//case: tx_buf = NULL,rx_buf = NULL		
			transfer.len = 8;
			transfer.tx_buf = NULL;
			transfer.rx_buf = NULL;
		}else if(!strncmp(buf, "len", 3)){//case: tx_buf != NULL,rx_buf != NULL, len = 0
			transfer.len = 0;
			transfer.tx_buf = ( u32 * ) kzalloc ( 8, GFP_KERNEL);
			transfer.rx_buf = ( u32 * ) kzalloc ( 8, GFP_KERNEL);
		}else if(!strncmp(buf, "nomem", 5)){//case: DMA mapping error
			spi_setup_xfer(&spi->dev,&transfer,8,0);
		}else if(!strncmp(buf, "fifo_len", 8)){//case: len exceed FIFO size
			chip_config->com_mod = 0;
			spi_setup_xfer(&spi->dev,&transfer,33,0);
		}else if(!strncmp(buf, "dma_len", 7)){//case: len exceed DMA size
			chip_config->com_mod = 1;
			spi_setup_xfer(&spi->dev,&transfer,1025,0);
		}else if(!strncmp(buf, "xfer", 4)){//case: len exceed DMA size
			;
		}else if(!strncmp(buf, "msg", 3)){//case: len exceed DMA size
			;
		}else{
			SPIDEV_LOG("Error test Wrong parameters.\n");
			ret = -1;
			goto out;
		}
		
		if(strncmp(buf, "xfer", 4)){	//case: MSG empty
			spi_message_add_tail(&transfer, &msg);
		}
		if(!strncmp(buf, "msg", 3)) {	//case: message = NULL	
			ret = spi_sync(spi, NULL);
				
		}else{
			ret = spi_sync(spi, &msg);			
		}		
		if(ret != -EINVAL){
				return -100;
		}else{			
			SPIDEV_LOG("Message transfer error test passed ret:%d\n", ret);
		}
	}else if( !strncmp(buf, "-pause", 6)){
		spi_setup_xfer(&spi->dev,&transfer,32,0);
		spi_message_add_tail(&transfer, &msg);
		spi_setup_xfer(&spi->dev,&transfer2,1024,0);
		spi_message_add_tail(&transfer2, &msg);
		spi_setup_xfer(&spi->dev,&transfer3,32,0);
		spi_message_add_tail(&transfer3, &msg);
		
		ret = spi_sync(spi, &msg);
		if(ret < 0){
			SPIDEV_LOG("Message transfer err:%d\n", ret);
		}else{
			ret = spi_recv_check(&msg);
			if(ret != 0){
				ret = -ret;					
				printk(KERN_ERR"Message transfer err:%d\n", ret);
			}
		}		
	}else if(!strncmp(buf, "-stress", 7)){
		sscanf ( buf+8, "%d", &len );
		stress_err = 0;	
		if(len == 0){
	//			xfer = (struct spi_transfer *)kzalloc(SPI_STRESS_MAX*sizeof( struct spi_transfer) , GFP_KERNEL);
			//will release in spi_recv_check() function
			SPIDEV_LOG("Message multi xfer stress start\n");
			for(i=0;i<SPI_STRESS_MAX;i++){			
				ret = spi_setup_xfer(&spi->dev,&stress_xfer[i],32,0);
				if(ret != 0){									
					SPIDEV_LOG("Message set up err:%d\n", ret);
				}
				spi_message_add_tail(&stress_xfer[i], &msg);
				
			}
			ret = spi_sync(spi, &msg);
			if(ret < 0){
				SPIDEV_LOG("Message transfer err:%d\n", ret);
			}else{
				ret = spi_recv_check(&msg);
				if(ret != 0){
					ret = -ret;					
					SPIDEV_LOG("Message transfer err:%d\n", ret);
				}else{
					SPIDEV_LOG("Message multi xfer stress pass\n");
				}
			}
		}else if(len == 1){	
			int loop_count = 0;
			//loop_count  =250, take aboat half a hour.
			for(loop_count=0;loop_count <= 5000;loop_count ++){
		//			msg_str = (struct spi_message *)kzalloc(SPI_STRESS_MAX * sizeof(struct spi_message) , GFP_KERNEL);
		//			xfer = ( struct spi_transfer * )kzalloc(SPI_STRESS_MAX * sizeof(struct spi_transfer) , GFP_KERNEL);
				SPIDEV_LOG("Message multi msg stress start\n");
				p = stress_msg;
				for(i=0;i<SPI_STRESS_MAX;i++){
					spi_message_init(p);
					ret = spi_setup_xfer(&spi->dev,&stress_xfer[i],32,0);
					if(ret != 0){									
						SPIDEV_LOG("xfer set up err:%d\n", ret);
					}
					spi_message_add_tail(&stress_xfer[i], p);
				
					p->complete = spi_complete;
					p->context = p;
		//				ret = spi_async(spi, p);
		//				if(ret < 0){
		//					SPIDEV_LOG("Message %d transfer err:%d\n",i, ret);
		//				}
					p ++;
				}
				p = stress_msg;
				for(i=0;i<SPI_STRESS_MAX;i++){
					ret = spi_async(spi, p);
					if(ret < 0){
						SPIDEV_LOG("Message %d transfer err:%d\n",i, ret);
					}
					p++;
				}
				wait_for_completion(&mt_spi_done);
			}
	//			kfree(msg_str);
			if(stress_err != 0){
				ret = -stress_err;					
				stress_err = 0;
				SPIDEV_LOG("Message stress err:%d\n", ret);
			}else{
				SPIDEV_LOG("Message stress Pass\n");
				ret = 0;
			}
		}else{
		}	
	}else if(!strncmp(buf, "-concurrent", 11)){

		stress_err = 0;
		spi_concur1 = kthread_run(threadfunc1,(void *)spi,"spi_concrrent1");
		if(IS_ERR(spi_concur1)){
			SPIDEV_LOG("Unable to start kernelthread 1\n");
			ret = -5;
			goto out;
		}
		spi_concur2 = kthread_run(threadfunc2,(void *)spi,"spi_concrrent2");
		if(IS_ERR(spi_concur2)){
			SPIDEV_LOG("Unable to start kernelthread 2\n");
			ret = -5;
			goto out;
		}
		spi_concur3 = kthread_run(threadfunc3,(void *)spi,"spi_concrrent3");
		if(IS_ERR(spi_concur3)){
			SPIDEV_LOG("Unable to start kernelthread 3\n");
			ret = -5;
			goto out;
		}
		spi_concur4 = kthread_run(threadfunc4,(void *)spi,"spi_concrrent4");
		if(IS_ERR(spi_concur4)){
			SPIDEV_LOG("Unable to start kernelthread 4\n");
			ret = -5;
			goto out;
		}

		msleep(10000);
		SPIDEV_LOG("stop kernelthread \n");

		kthread_stop(spi_concur1);
		kthread_stop(spi_concur2);
		kthread_stop(spi_concur3);
		kthread_stop(spi_concur4);
		ret = -stress_err;
		stress_err = 0;
		SPIDEV_LOG("concurrent test done %d\n\n\n",ret);
		
	}else if( !strncmp(buf, "-pwm_write1", 11)){
	    pr_warn("++++++++++++++++pwm_write1+++++++++++\n");

		static unsigned char val[8] ={0xFF,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xFF};

		pwm_write( dev ,val, 8);
		int iter=0 ;
	
		for( iter=0; iter < 8*2; iter++)
		{
			pr_warn("++++++pwm_data[%d] =  %x ++++++\n",iter,pwm_data_jy[iter]);	
		}	
	}else if( !strncmp(buf, "-pwm_write2", 11)){
		pr_warn("++++++++++++++++pwm_write2+++++++++++\n");	
		static unsigned char val[1018] = {0};
		memset(val ,0xCC,sizeof(val));
		pwm_write( dev ,val, 1018);
	}else{
		SPIDEV_LOG("Wrong parameters.\n");
		ret = -1;
		goto out;
	}
	ret = count;

out:
	return ret;
		
}

static DEVICE_ATTR(spi, 0200, NULL, spi_store);
static DEVICE_ATTR(spi_msg, 0200, NULL, spi_msg_store);



static struct device_attribute *spi_attribute[]={
	&dev_attr_spi,
	&dev_attr_spi_msg,
};

static int spi_create_attribute(struct device *dev)
{
	int num,idx;
	int err =0;
	num = (int)(sizeof(spi_attribute)/sizeof(spi_attribute[0]));

	for (idx = 0; idx < num; idx ++) {
		if ((err = device_create_file(dev, spi_attribute[idx])))
			break;
	}
	return err;
	
}

static void spi_remove_attribute(struct device *dev)
{
	int num, idx;
	num = (int)(sizeof(spi_attribute)/sizeof(spi_attribute[0]));

	for (idx = 0; idx < num; idx ++) {
		device_remove_file(dev, spi_attribute[idx]);
	}

	return;
}

static int spi_test_remove(struct spi_device *spi)
{

	SPIDEV_LOG("spi_test_remove.\n");
	spi_remove_attribute(&spi->dev);
	return 0;
}

static int __init spi_test_probe(struct spi_device *spi)
{
	SPIDEV_LOG("spi test probe  enter\n");
	spi_test=spi;
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 32;
	return spi_create_attribute( &spi->dev );
	return 0;
}

//struct spi_device_id spi_id_table = {"test_spi", 0};
struct spi_device_id spi_id_table = {"spi-ut", 0};

static struct spi_driver spi_test_driver = {
	.driver = {
		.name = "test_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = spi_test_probe,
	.remove=spi_test_remove,
	.id_table = &spi_id_table,
};
static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
        	.modalias="spi-ut",
		.bus_num = 0,
		.chip_select=1,
		.mode = SPI_MODE_3,
	},
};

static int __init spi_dev_init(void)
{
	SPIDEV_LOG("SPI_DEV_INIT.\n");
	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	return spi_register_driver(&spi_test_driver);
}

static void __exit spi_test_exit(void)
{
	SPIDEV_LOG("SPI_DEV_EXIT.\n");
	spi_unregister_driver(&spi_test_driver);
	
	return;
}

module_init(spi_dev_init);
module_exit(spi_test_exit);

MODULE_DESCRIPTION ( "mt SPI test device driver" );
MODULE_AUTHOR ( "Ranran Lu <ranran.lu@mediatek.com>" );
MODULE_LICENSE("GPL");
