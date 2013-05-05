ifneq ($(KERNELRELEASE),)

KBUILD_CFLAGS += -Wno-sequence-point
EXTRA_CFLAGS=-fno-pic

MOD_NAME = eds
obj-m := $(MOD_NAME).o

$(MOD_NAME)-y := main.o xts.o aescrypt.o aeskey.o aestab.o

else
        KERNELDIR ?= ../../..
        PWD := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
cscope:
	find *.[ch] > cscope.files
	cscope -b -i cscope.files
clean:
	rm *.o
	rm *.ko
	rm *.mod.c
endif
