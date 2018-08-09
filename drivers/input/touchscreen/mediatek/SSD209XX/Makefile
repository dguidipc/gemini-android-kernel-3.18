#
# Makefile for the touchscreen drivers.
#
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/SSD20XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-$(CONFIG_TOUCHSCREEN_MTK_SSL_SSD20XX)   += ssd20xx.o ssd20xx_mx25v100.o ssd20xx_upgrade.o
