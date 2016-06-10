/*
 * SPI master driver using generic bitbanged GPIO
 *
 * Copyright (C) 2006,2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/spi_gpio.h>
#include <linux/delay.h>

#include "mt_spi_hal.h"

#include <linux/of_address.h>



//#include <mach/mt_gpio.h>
/*direction*/
extern int mt_set_gpio_dir_base(unsigned long pin, unsigned long dir);



/*input/output*/
extern int mt_set_gpio_out_base(unsigned long pin, unsigned long output);
extern int mt_get_gpio_in_base(unsigned long pin);

/*mode control*/
extern int mt_set_gpio_mode_base(unsigned long pin, unsigned long mode);

static void __iomem *spi_gpio_vbase;

static spi_write_gpio_rg(u32 offset, u32 value)
{
	mt_reg_sync_writel(value, spi_gpio_vbase+offset);
}

/*
 * This bitbanging SPI master driver should help make systems usable
 * when a native hardware SPI engine is not available, perhaps because
 * its driver isn't yet working or because the I/O pins it requires
 * are used for other purposes.
 *
 * platform_device->driver_data ... points to spi_gpio
 *
 * spi->controller_state ... reserved for bitbang framework code
 * spi->controller_data ... holds chipselect GPIO
 *
 * spi->master->dev.driver_data ... points to spi_gpio->bitbang
 */

struct spi_gpio {
	struct spi_bitbang		bitbang;
	struct spi_gpio_platform_data	pdata;
	struct platform_device		*pdev;
	int				cs_gpios[0];
};

/*----------------------------------------------------------------------*/

/*
 * Because the overhead of going through four GPIO procedure calls
 * per transferred bit can make performance a problem, this code
 * is set up so that you can use it in either of two ways:
 *
 *   - The slow generic way:  set up platform_data to hold the GPIO
 *     numbers used for MISO/MOSI/SCK, and issue procedure calls for
 *     each of them.  This driver can handle several such busses.
 *
 *   - The quicker inlined way:  only helps with platform GPIO code
 *     that inlines operations for constant GPIOs.  This can give
 *     you tight (fast!) inner loops, but each such bus needs a
 *     new driver.  You'll define a new C file, with Makefile and
 *     Kconfig support; the C code can be a total of six lines:
 *
 *		#define DRIVER_NAME	"myboard_spi2"
 *		#define	SPI_MISO_GPIO	119
 *		#define	SPI_MOSI_GPIO	120
 *		#define	SPI_SCK_GPIO	121
 *		#define	SPI_N_CHIPSEL	4
 *		#include "spi-gpio.c"
 */

#if 1//ndef DRIVER_NAME
static struct spi_gpio_platform_data	*Spdata=NULL;

#define DRIVER_NAME	"spi_gpio"

#define GENERIC_BITBANG	/* vs tight inlines */

/* all functions referencing these symbols must define pdata */

#define SPI_MISO_GPIO	(Spdata->miso)
#define SPI_MOSI_GPIO	(Spdata->mosi)
#define SPI_SCK_GPIO	(Spdata->sck)

#define SPI_N_CHIPSEL	(Spdata->num_chipselect)

struct spi_gpio_mode {
	unsigned long 	cs_mode;
	unsigned long	mosi_mode;
	unsigned long	miso_mode;
	unsigned long	sck_mode;
};

static struct spi_gpio_mode	*Spmode=NULL;
#endif
#define SPIGPIO_DBG(fmt, arg...) printk(KERN_WARNING fmt, ##arg)

//static void __iomem *spi_gpio_vbase;




/*----------------------------------------------------------------------*/

static inline struct spi_gpio * __pure
spi_to_spi_gpio(const struct spi_device *spi)
{
	const struct spi_bitbang	*bang;
	struct spi_gpio			*spi_gpio;

	bang = spi_master_get_devdata(spi->master);
	spi_gpio = container_of(bang, struct spi_gpio, bitbang);
	return spi_gpio;
}

static inline struct spi_gpio_platform_data * __pure
spi_to_pdata(const struct spi_device *spi)
{
	return &spi_to_spi_gpio(spi)->pdata;
}

/* this is #defined to avoid unused-variable warnings when inlining */
#define pdata		spi_to_pdata(spi)

static inline void setsck(const struct spi_device *spi, int is_on)
{
#if 0
	if(is_on)
		spi_write_gpio_rg(0x464,(1<<4));
	else
		spi_write_gpio_rg(0x468,(1<<4));
#else
       mt_set_gpio_out_base(SPI_SCK_GPIO, is_on);	//gpio_set_value(SPI_SCK_GPIO, is_on);
	      // mt_set_gpio_out_base(79, is_on);	//gpio_set_value(SPI_MOSI_GPIO, is_on);
		//printk("SPI_bitbang setsck SPI_MOSI_GPIO:%lu \n" ,SPI_MOSI_GPIO);	
#endif
}

static inline void setmosi(const struct spi_device *spi, int is_on)
{
#if 0
		if(is_on)
			spi_write_gpio_rg(0x464,(1<<8));
		else
			spi_write_gpio_rg(0x468,(1<<8));
		//mt_set_gpio_out_base(SPI_MOSI_GPIO, is_on);
#else
      if(is_on != 0)
	  	is_on = 1;
	  
       mt_set_gpio_out_base(SPI_MOSI_GPIO, is_on);	//gpio_set_value(SPI_MOSI_GPIO, is_on);
	//	printk("SPI_bitbang  SPI_MOSI_GPIO:%lu ,is_on = %d\n" ,SPI_MOSI_GPIO ,is_on);
#endif
}

static inline int getmiso(const struct spi_device *spi)
{
#if 1
//	printk("SPI_bitbang  SPI_MISO_GPIO:%lu \n" ,SPI_MISO_GPIO);

	return !!mt_get_gpio_in_base(SPI_MISO_GPIO);
#else
	return !!gpio_get_value(SPI_MISO_GPIO);
#endif
}

#undef pdata

/*
 * NOTE:  this clocks "as fast as we can".  It "should" be a function of the
 * requested device clock.  Software overhead means we usually have trouble
 * reaching even one Mbit/sec (except when we can inline bitops), so for now
 * we'll just assume we never need additional per-bit slowdowns.
 */
#define spidelay(nsecs)	do {} while (0)
//#define spidelay(nsecs)	udelay(nsecs/1000)//ndelay(nsecs)

#include "spi-bitbang-txrx.h"

/*
 * These functions can leverage inline expansion of GPIO calls to shrink
 * costs for a txrx bit, often by factors of around ten (by instruction
 * count).  That is particularly visible for larger word sizes, but helps
 * even with default 8-bit words.
 *
 * REVISIT overheads calling these functions for each word also have
 * significant performance costs.  Having txrx_bufs() calls that inline
 * the txrx_word() logic would help performance, e.g. on larger blocks
 * used with flash storage or MMC/SD.  There should also be ways to make
 * GCC be less stupid about reloading registers inside the I/O loops,
 * even without inlined GPIO calls; __attribute__((hot)) on GCC 4.3?
 */

static u32 spi_gpio_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	int i;

	return bitbang_txrx_be_cpha0(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
//	printk("spi_gpio_txrx_word_mode1 word=%x,bits=%d,nsecs=%x\n",word,bits,nsecs);

	return bitbang_txrx_be_cpha1(spi, nsecs, 0, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
//	printk("spi_gpio_txrx_word_mode2 word=%x,bits=%d,nsecs=%x\n",word,bits,nsecs);

	return bitbang_txrx_be_cpha0(spi, nsecs, 1, 0, word, bits);
}

static u32 spi_gpio_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
//	printk("spi_gpio_txrx_word_mode3 word=%x,bits=%d,nsecs=%x\n",word,bits,nsecs);

	return bitbang_txrx_be_cpha1(spi, nsecs, 1, 0, word, bits);
}

/*
 * These functions do not call setmosi or getmiso if respective flag
 * (SPI_MASTER_NO_RX or SPI_MASTER_NO_TX) is set, so they are safe to
 * call when such pin is not present or defined in the controller.
 * A separate set of callbacks is defined to get highest possible
 * speed in the generic case (when both MISO and MOSI lines are
 * available), as optimiser will remove the checks when argument is
 * constant.
 */

static u32 spi_gpio_spec_txrx_word_mode0(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode1(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode2(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, flags, word, bits);
}

static u32 spi_gpio_spec_txrx_word_mode3(struct spi_device *spi,
		unsigned nsecs, u32 word, u8 bits)
{
	unsigned flags = spi->master->flags;
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, flags, word, bits);
}

/*----------------------------------------------------------------------*/

static void spi_gpio_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_gpio *spi_gpio = spi_to_spi_gpio(spi);
	unsigned int cs = spi_gpio->cs_gpios[spi->chip_select];
	unsigned int cs_status=0;
	/* set initial clock polarity */
	if (is_active)
		setsck(spi, spi->mode & SPI_CPOL);

	if (cs != SPI_GPIO_NO_CHIPSELECT) {
		/* SPI is normally active-low */
#if 1
		 mt_set_gpio_out_base(cs, (spi->mode & SPI_CS_HIGH) ? is_active : !is_active);
#else
		gpio_set_value(cs, (spi->mode & SPI_CS_HIGH) ? is_active : !is_active);
#endif
	}
}

static int spi_gpio_setup(struct spi_device *spi)
{
	unsigned int		cs;
	unsigned int		*pcs;
	int			status = 0;
	struct spi_gpio		*spi_gpio = spi_to_spi_gpio(spi);
	struct device_node	*np = spi->master->dev.of_node;

	if (spi->bits_per_word > 32)
		return -EINVAL;

	if (np) {
		/*
		 * In DT environments, the CS GPIOs have already been
		 * initialized from the "cs-gpios" property of the node.
		 */
		cs = spi_gpio->cs_gpios[spi->chip_select];
	} else {
		/*
		 * ... otherwise, take it from spi->controller_data
		 */
#if 1
		pcs=(unsigned int*)spi->controller_data;
		cs=*pcs;
#else
		cs = (unsigned int)spi->controller_data;
#endif
	}

	if (!spi->controller_state) {
		if (cs != SPI_GPIO_NO_CHIPSELECT) {		
		#if 1
			status = mt_set_gpio_dir_base(cs,
					!(spi->mode & SPI_CS_HIGH));
			//printk("status=%d\n",status);
		#else
			status = gpio_request(cs, dev_name(&spi->dev));
			if (status)
				return status;
			status = gpio_direction_output(cs,
					!(spi->mode & SPI_CS_HIGH));
		#endif
		}
	}
	if (!status) {
		/* in case it was initialized from static board data */
		spi_gpio->cs_gpios[spi->chip_select] = cs;
		status = spi_bitbang_setup(spi);
	}

	if (status) {
		if (!spi->controller_state && cs != SPI_GPIO_NO_CHIPSELECT)
			printk("gpiofreecs\n");//gpio_free(cs);
	}
	return status;
}

static void spi_gpio_cleanup(struct spi_device *spi)
{
	struct spi_gpio *spi_gpio = spi_to_spi_gpio(spi);
	unsigned int cs = spi_gpio->cs_gpios[spi->chip_select];

	if (cs != SPI_GPIO_NO_CHIPSELECT)
		printk("gpiofreecs1\n");//gpio_free(cs);
	spi_bitbang_cleanup(spi);
}

static int spi_gpio_alloc(unsigned pin, const char *label, bool is_in)
{
	int value;
	int gpiomode=0;

	value = 0;//gpio_request(pin, label);
	
#if 1
	if(Spmode!=NULL)
	{
		if(SPI_MISO_GPIO==pin)
			gpiomode=Spmode->miso_mode;
		else if(SPI_MOSI_GPIO==pin)
			gpiomode=Spmode->mosi_mode;
		else if(SPI_SCK_GPIO==pin)
			gpiomode=Spmode->sck_mode;

		printk("misom=%ld,mosim=%ld,sckm=%ld\n",Spmode->miso_mode,Spmode->mosi_mode,Spmode->sck_mode);
	}
		mt_set_gpio_mode_base(pin, gpiomode);
if (value == 0) {
		value = mt_set_gpio_dir_base(pin,!is_in);
		//if (is_in)
		//	value = mt_set_gpio_dir_base(pin,0);//gpio_direction_input(pin);
		//else
		//	value = mt_set_gpio_dir_base(pin,1);//gpio_direction_output(pin, 0);
	}

#else
	if (value == 0) {
		if (is_in)
			value = gpio_direction_input(pin);
		else
			value = gpio_direction_output(pin, 0);
	}
#endif	
	return value;
}

static int spi_gpio_request(struct spi_gpio_platform_data *pdata,
			    const char *label, u16 *res_flags)
{
	int value;

	/* NOTE:  SPI_*_GPIO symbols may reference "pdata" */

	if (SPI_MOSI_GPIO != SPI_GPIO_NO_MOSI) {
		value = spi_gpio_alloc(SPI_MOSI_GPIO, label, false);
		if (value)
			goto done;
	} else {
		/* HW configuration without MOSI pin */
		pr_err("Peng spi_gpio_request set SPI_MASTER_NO_TX\n");
		*res_flags |= SPI_MASTER_NO_TX;
	}

	if (SPI_MISO_GPIO != SPI_GPIO_NO_MISO) {
		value = spi_gpio_alloc(SPI_MISO_GPIO, label, true);
		if (value)
			goto free_mosi;
	} else {
		/* HW configuration without MISO pin */
		pr_err("Peng spi_gpio_request set SPI_MASTER_NO_TX\n");
		*res_flags |= SPI_MASTER_NO_RX;
	}

	value = spi_gpio_alloc(SPI_SCK_GPIO, label, false);
	if (value)
		goto free_miso;

	goto done;

free_miso:
	if (SPI_MISO_GPIO != SPI_GPIO_NO_MISO)
		printk("free_miso\n");//gpio_free(SPI_MISO_GPIO);
free_mosi:
	if (SPI_MOSI_GPIO != SPI_GPIO_NO_MOSI)
		printk("free_mosi\n");//gpio_free(SPI_MOSI_GPIO);
done:
	return value;
}

#ifdef CONFIG_OF
static const struct of_device_id spi_gpio_dt_ids[] = {
	{ .compatible = "mediatek,spi-gpio" },
	{}
};


MODULE_DEVICE_TABLE(of, spi_gpio_dt_ids);

static int spi_gpio_probe_dt(struct platform_device *pdev)
{
	int ret;
	u32 tmp;
	struct spi_gpio_platform_data	*pdata;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id =
			of_match_device(spi_gpio_dt_ids, &pdev->dev);
#if 1
	unsigned int gpio_sck,gpio_miso,gpio_mosi,num_chipselects;
    unsigned int gpio_sck_mode,gpio_miso_mode,gpio_mosi_mode;
	struct device_node *node = NULL;
	struct spi_gpio_mode	*pmode;
#endif
	if (!of_id)
		return 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	pmode = devm_kzalloc(&pdev->dev, sizeof(*pmode), GFP_KERNEL);
	if (!pmode)
		return -ENOMEM;

#if 1
	//node = of_find_compatible_node(NULL, NULL, "mediatek,mt_soc_pcm_dl1");
if (np)
{
	if (of_property_read_u32_index(np, "gpio-sck", 0, &gpio_sck))
   {
	   printk("gpio-sck get pin fail!!!\n");
	  goto error_free;
   }
	pdata->sck=gpio_sck;
   if (of_property_read_u32_index(np, "gpio-sck", 1, &gpio_sck_mode))
   {
	   printk("gpio-sck get pin_mode fail!!!\n");
   }
   pmode->sck_mode=gpio_sck_mode;
	if (of_property_read_u32_index(np, "gpio-miso", 0, &gpio_miso))
   {
	   printk("gpio-miso get pin fail!!!\n");
	  goto error_free;
   }
		pdata->miso=gpio_miso;
   if (of_property_read_u32_index(np, "gpio-miso", 1, &gpio_miso_mode))
   {
	   printk("gpio-miso get pin_mode fail!!!\n");
   }
   pmode->miso_mode=gpio_miso_mode;
   	if (of_property_read_u32_index(np, "gpio-mosi", 0, &gpio_mosi))
   {
	   printk("gpio-mosi get pin fail!!!\n");
	  goto error_free;
   }
	pdata->mosi=gpio_mosi;
   if (of_property_read_u32_index(np, "gpio-mosi", 1, &gpio_mosi_mode))
   {
	   printk("gpio-mosi get pin_mode fail!!!\n");
   }
     pmode->mosi_mode=gpio_mosi_mode;
   	if (of_property_read_u32_index(np, "num-css", 0, &num_chipselects))
   {
	   printk("num-css get pin fail!!!\n");
	  goto error_free;
   }
	printk("num_cs=%d\n",num_chipselects);
	pdata->num_chipselect=num_chipselects;
}
#else
	ret = of_get_named_gpio(np, "gpio-sck", 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "gpio-sck property not found\n");
		goto error_free;
	}
	pdata->sck = ret;

	ret = of_get_named_gpio(np, "gpio-miso", 0);
	if (ret < 0) {
		dev_info(&pdev->dev, "gpio-miso property not found, switching to no-rx mode\n");
		pdata->miso = SPI_GPIO_NO_MISO;
	} else
		pdata->miso = ret;

	ret = of_get_named_gpio(np, "gpio-mosi", 0);
	if (ret < 0) {
		dev_info(&pdev->dev, "gpio-mosi property not found, switching to no-tx mode\n");
		pdata->mosi = SPI_GPIO_NO_MOSI;
	} else
		pdata->mosi = ret;

	ret = of_property_read_u32(np, "num-chipselects", &tmp);
	if (ret < 0) {
		dev_err(&pdev->dev, "num-chipselects property not found\n");
		goto error_free;
	}
	pdata->num_chipselect = tmp;
#endif
#if 0
		node = of_find_compatible_node(NULL, NULL, "mediatek,GPIO");
    if(node){
	/* Setup IO addresses */
	spi_gpio_vbase = of_iomap(node, 0);
	//GPIOLOG("gpio_vbase.MIPI_RX_CSI1_regs=0x%lx\n", gpio_vbase.MIPI_RX_CSI1_regs);
    }
#endif	
	pdev->dev.platform_data = pdata;
	//SPIGPIO_DBG("get config sck=%x,mosi=%lx,miso=%lx,numchip=%d\n",pdata->sck,pdata->mosi,pdata->miso,pdata->num_chipselect);
	Spdata=pdata;
	Spmode=pmode;
	//SPIGPIO_DBG("spmode get config sck=%lx,mosi=%lx,miso=%lx\n",Spmode->sck_mode,Spmode->mosi_mode,Spmode->miso_mode);
		node = of_find_compatible_node(NULL, NULL, "mediatek,GPIO");
    if(node){
	/* Setup IO addresses */
	spi_gpio_vbase = of_iomap(node, 0);
	//printk("my_gpio: gpio_probe is:0x%x\n", (unsigned int)spi_gpio_vbase);//GPIOLOG("gpio_vbase.MIPI_RX_CSI1_regs=0x%lx\n", gpio_vbase.MIPI_RX_CSI1_regs);
    }
	//printk("probedt end\n");

	return 1;

error_free:
	devm_kfree(&pdev->dev, pdata);
	devm_kfree(&pdev->dev, pmode);
	return ret;
}
#else
static inline int spi_gpio_probe_dt(struct platform_device *pdev)
{
	return 0;
}
#endif

static int spi_gpio_probe(struct platform_device *pdev)
{
	int				status;
	struct spi_master		*master;
	struct spi_gpio			*spi_gpio;
	struct spi_gpio_platform_data	*pdata;
	u16 master_flags = 0;
	bool use_of = 0;
	SPIGPIO_DBG("spi_gpio_probe start\n");

#if 1
unsigned int gpio_cs_mode;

#endif

	status = spi_gpio_probe_dt(pdev);
	if (status < 0)
		return status;
	if (status > 0)
		use_of = 1;

	pdata = pdev->dev.platform_data;
		SPIGPIO_DBG("spi_gpio_probe end6\n");
#ifdef GENERIC_BITBANG
	if (!pdata || !pdata->num_chipselect)
		return -ENODEV;
#endif

	status = spi_gpio_request(pdata, dev_name(&pdev->dev), &master_flags);
	SPIGPIO_DBG("spi_gpio_probe end5status=%d,maflag=%d\n",status,master_flags);

	if (status < 0)
		return status;

	master = spi_alloc_master(&pdev->dev, sizeof(*spi_gpio) +
					(sizeof(int) * SPI_N_CHIPSEL));
		//SPIGPIO_DBG("spi_gpio_probe end3\n");
	if (!master) {
			//SPIGPIO_DBG("spi_gpio_probe end4\n");
		status = -ENOMEM;
		goto gpio_free;
	}
	spi_gpio = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, spi_gpio);

	spi_gpio->pdev = pdev;
	if (pdata)
		spi_gpio->pdata = *pdata;

	master->flags = master_flags;
	master->bus_num = pdev->id;
	master->num_chipselect = SPI_N_CHIPSEL;
	master->setup = spi_gpio_setup;
	master->cleanup = spi_gpio_cleanup;
	//SPIGPIO_DBG("spi_gpio_probe ms_busn=%d\n",master->bus_num);
#ifdef CONFIG_OF
	master->dev.of_node = pdev->dev.of_node;

	if (use_of) {
		int i;
		struct device_node *np = pdev->dev.of_node;

		/*
		 * In DT environments, take the CS GPIO from the "cs-gpios"
		 * property of the node.
		 */

#if 1
		for (i = 0; i < 1; i++)
			{
			 if (of_property_read_u32_index(np, "cs-gpios", 0, &spi_gpio->cs_gpios[i]))
			{
				printk("gpio-mosi get pin fail!!!\n");
			   goto gpio_free;
			}
			if (of_property_read_u32_index(np, "cs-gpios", 1, &gpio_cs_mode))
			{
				printk("gpio-mosi get pin_mode fail!!!\n");
				goto gpio_free;
			}
			}

#else
		for (i = 0; i < SPI_N_CHIPSEL; i++)
			spi_gpio->cs_gpios[i] =
				of_get_named_gpio(np, "cs-gpios", i);
#endif		
	}
#endif
	//SPIGPIO_DBG("spi_gpio_probe end2cs=%d\n",spi_gpio->cs_gpios[0]);

	spi_gpio->bitbang.master = spi_master_get(master);
	//SPIGPIO_DBG("spi_gpio_probe end00\n");
	spi_gpio->bitbang.chipselect = spi_gpio_chipselect;
	//SPIGPIO_DBG("spi_gpio_probe end01notx=%ld,norx=%ld\n",SPI_MASTER_NO_TX,SPI_MASTER_NO_RX);
    pr_err("Peng setup bitbang.txrx_word %u\n", master_flags);
	if ((master_flags & (SPI_MASTER_NO_TX | SPI_MASTER_NO_RX)) == 0) {	
		spi_gpio->bitbang.txrx_word[SPI_MODE_0] = spi_gpio_txrx_word_mode0;
		spi_gpio->bitbang.txrx_word[SPI_MODE_1] = spi_gpio_txrx_word_mode1;
		spi_gpio->bitbang.txrx_word[SPI_MODE_2] = spi_gpio_txrx_word_mode2;
		spi_gpio->bitbang.txrx_word[SPI_MODE_3] = spi_gpio_txrx_word_mode3;
	} else {
		spi_gpio->bitbang.txrx_word[SPI_MODE_0] = spi_gpio_spec_txrx_word_mode0;
		spi_gpio->bitbang.txrx_word[SPI_MODE_1] = spi_gpio_spec_txrx_word_mode1;
		spi_gpio->bitbang.txrx_word[SPI_MODE_2] = spi_gpio_spec_txrx_word_mode2;
		spi_gpio->bitbang.txrx_word[SPI_MODE_3] = spi_gpio_spec_txrx_word_mode3;
	}
		//SPIGPIO_DBG("spi_gpio_probe end02\n");
	spi_gpio->bitbang.setup_transfer = spi_bitbang_setup_transfer;
	spi_gpio->bitbang.flags = SPI_CS_HIGH;
	//SPIGPIO_DBG("spi_gpio_probe end03\n");

	status = spi_bitbang_start(&spi_gpio->bitbang);
		//SPIGPIO_DBG("spi_gpio_probe end1\n");
	if (status < 0) {
		spi_master_put(spi_gpio->bitbang.master);
gpio_free:
		if (SPI_MISO_GPIO != SPI_GPIO_NO_MISO)
			gpio_free(SPI_MISO_GPIO);
		if (SPI_MOSI_GPIO != SPI_GPIO_NO_MOSI)
			gpio_free(SPI_MOSI_GPIO);
		gpio_free(SPI_SCK_GPIO);
		spi_master_put(master);
	}
	dev_err(&pdev->dev,"spi_gpio_probe end\n");

	return status;
}

static int spi_gpio_remove(struct platform_device *pdev)
{
	struct spi_gpio			*spi_gpio;
	struct spi_gpio_platform_data	*pdata;
	int				status;

	spi_gpio = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	/* stop() unregisters child devices too */
	status = spi_bitbang_stop(&spi_gpio->bitbang);
	spi_master_put(spi_gpio->bitbang.master);

	platform_set_drvdata(pdev, NULL);

	if (SPI_MISO_GPIO != SPI_GPIO_NO_MISO)
		printk("gpoifreemiso\n");//gpio_free(SPI_MISO_GPIO);
	if (SPI_MOSI_GPIO != SPI_GPIO_NO_MOSI)
		printk("gpoifreemosi\n");//gpio_free(SPI_MOSI_GPIO);
	printk("gpoifreesck\n");//gpio_free(SPI_SCK_GPIO);

	return status;
}

MODULE_ALIAS("platform:" DRIVER_NAME);

static struct platform_driver spi_gpio_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table =spi_gpio_dt_ids, //of_match_ptr(spi_gpio_dt_ids),
	},
	.probe		= spi_gpio_probe,
	.remove		= spi_gpio_remove,
};
//module_platform_driver(spi_gpio_driver);
static int __init gpio_spi_init ( void )
{
	int ret;
	SPIGPIO_DBG("gpio_spi_init\n");
	ret = platform_driver_register ( &spi_gpio_driver );
	return ret;
}

static void __init gpio_spi_exit ( void )
{
	platform_driver_unregister ( &spi_gpio_driver );
}

module_init ( gpio_spi_init );
module_exit ( gpio_spi_exit );


MODULE_DESCRIPTION("SPI master driver using generic bitbanged GPIO ");
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");
