.PHONY: check-env
obj-m += register_axi.o
all: check-env
	make -C $(KDIR) M=$(PWD) modules
clean: check-env
	make -C $(KDIR) M=$(PWD) clean

check-env:
ifndef KDIR
  $(error KDIR is not defined)
endif
