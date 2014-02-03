ccflags-$(CONFIG_USB_DWC_DEBUG) := -DDEBUG
ccflags-y += -DDWC_EN_ISOC
ccflags-y += -I$(PWD)/dwc_common_port
ifneq ($(CONFIG_X86_INTEL_SOFIA_ERRATA_002),y)
ccflags-y += -DPARTIAL_POWER_DOWN
endif

obj-$(CONFIG_USB_DWC_OTG)	:= dwc_otg.o dwc_common_port.o

dwc_otg-objs		:= dwc_otg_driver.o dwc_otg_attr.o
dwc_otg-objs		+= dwc_otg_cil.o dwc_otg_cil_intr.o
dwc_otg-objs		+= dwc_otg_pcd_linux.o dwc_otg_pcd.o dwc_otg_pcd_intr.o
dwc_otg-objs		+= dwc_otg_hcd.o dwc_otg_hcd_linux.o dwc_otg_hcd_intr.o dwc_otg_hcd_queue.o dwc_otg_hcd_ddma.o
dwc_otg-objs		+= dwc_otg_adp.o
dwc_common_port-objs 	:= dwc_common_port/dwc_cc.o dwc_common_port/dwc_modpow.o
dwc_common_port-objs 	+= dwc_common_port/dwc_dh.o dwc_common_port/dwc_crypto.o
dwc_common_port-objs 	+= dwc_common_port/dwc_notifier.o dwc_common_port/dwc_common_linux.o
dwc_common_port-objs	+= dwc_common_port/dwc_mem.o
