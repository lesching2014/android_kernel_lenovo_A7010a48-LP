include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  gyroscope.o gyro_factory.o

ifeq ($(CONFIG_MTK_MPU3050C),y)
obj-y	+=  mpu3050c/
endif

ifeq ($(CONFIG_MTK_MPU6050C),y)
obj-y	+=  mpu6050c/
endif

ifeq ($(CONFIG_MTK_MPU3000),y)
obj-y   +=  mpu3000/
endif

ifeq ($(CONFIG_MTK_MPU3000_NEW),y)
obj-y   +=  mpu3000-new/
endif

ifeq ($(CONFIG_MTK_MPU6050GY_NEW),y)
obj-y   +=  mpu6050gy-new/
endif

ifeq ($(CONFIG_MTK_MPU6050GY),y)
obj-y   +=  mpu6050gy/
endif

ifeq ($(CONFIG_MTK_BMG160),y)
obj-y   +=  bmg160/
endif

ifeq ($(CONFIG_MTK_MPU6515G),y)
obj-y   +=  mpu6515/
endif

ifeq ($(CONFIG_MTK_MPU3050C_NEW),y)
obj-y   +=  mpu3050c-new/
endif
ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  ITG1010-new/
endif

# lenovo-sw caoyi1 add A+G sensor begin
ifeq ($(CONFIG_MTK_LSM6DS3GY),y)
obj-y   +=  lsm6ds3gy/
endif
# lenovo-sw caoyi1 add A+G sensor end
