.PHONY: check-env
obj-m += pirla_mod.o
all: check-env
	make -C $(KDIR) M=$(PWD) modules
clean: check-env
	make -C $(KDIR) M=$(PWD) clean

check-env:
ifndef KDIR
  $(error KDIR is not defined)
endif
