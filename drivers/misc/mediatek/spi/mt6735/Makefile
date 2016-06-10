# ----------------------------------------------------------------------------- 
# export these for cross compiling:
# export ARCH=arm 
# export CROSS_COMPILE=arm-linux-eabi-
# export KERNELDIR=[path to kernel source]
# ----------------------------------------------------------------------------- 
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils \
                   -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 
endif

#microtrust liuwanli add for fpc irq driver begin
ifeq ($(CONFIG_NEU_TZ_DRIVER),y)

obj-y +=fpc_irq.o
fpc_irq-y := fpc_irq_main.o fpc_irq_supply.o fpc_irq_ctrl.o fpc_irq_pm.o fpc_irq_common.o
obj-$(CONFIG_SPI_BITBANG) += spi-gpio.o 
obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o

else

obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o
# lenovo-sw zhouwl, for gpio simulator spi
obj-$(CONFIG_SPI_BITBANG) += spi-gpio.o

#lenovo-sw, chenzz3, add for fingerprint, begin
obj-$(CONFIG_LENOVO_FINGERPRINT) +=  fpc/
#lenovo-sw, chenzz3, add for fingerprint, end

endif
#microtrust liuwanli modify for fpc irq driver end

