ifeq ($(KERNELRELEASE),)
	KERNELDIR ?= /home/jo/workdir/kernal/linux-3.2-build/linux-3.2
	PWD := $(shell pwd)
modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
else
  obj-m := pump.o
endif
