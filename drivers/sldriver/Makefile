EXTRA_CFLAGS :=-fno-pic
TARGET_MODULE += vidt_driver

obj-$(CONFIG_SLDRIVER) += vmm_hsym.o kern_ta.o vidt.o isr$(BITS).o

ifeq ($(ARCH) , x86_64)
$(error 64bit is not yet supported)
endif

default:
	$(MAKE) -C $(KERNEL_SRC_DIR) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules PWD=$(PWD)
clean:
	@rm -f *.o *.ko
