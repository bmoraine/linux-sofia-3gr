ccflags-y += -Idrivers/staging
ccflags-$(CONFIG_XGOLD_DCC_DEBUG) += -DDEBUG

obj-$(CONFIG_XGOLD_DCC) += xgold-dcc.o

xgold-dcc-objs = dcc-main.o \
	dcc-dts.o \
	dcc-core.o \
	dcc-hwregs.o \
	dcc-irqs.o \
	dcc-gra.o \
	dcc-sysfs.o \
	dcc-hwregs.o \
	dcc-gra.o \
	dcc-lcd.o \
	dcc-dif-mipi-dsi.o \
	dcc-dif-mipi-dbi.o

obj-$(CONFIG_TRANSMITTER_LVDS) += xgold-lvds.o

