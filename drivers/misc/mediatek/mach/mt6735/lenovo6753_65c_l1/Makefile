
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifeq ($(CONFIG_MTK_HDMI_SUPPORT),y)
obj-y	+=  hdmi/
endif

#obj-y   +=  lcm/
obj-y   +=  camera/camera/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
obj-y   +=  keypad/
#obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif

#lenovo-sw caoyi add for Psensor 20150317 begin
ifeq ($(CONFIG_MTK_EPL8801),y)
obj-y	+=  alsps/epl8801/
endif
#lenovo-sw caoyi add for Psensor 20150317 end

ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif

# lenovo-sw caoyi add for A+Gsensor begin
ifeq ($(CONFIG_MTK_LSM6DS3),y)
obj-y	+=  accelerometer/lsm6ds3/
endif

ifeq ($(CONFIG_MTK_LSM6DS3GY),y)
obj-y	+=  gyroscope/lsm6ds3gy/
endif
#l enovo-sw caoyi add for A+Gsensor end

ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif

endif
