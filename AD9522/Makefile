.PHONY: check-env
obj-m += ad9522.o

all: check-env
	@make -C $(KDIR) M=$(PWD) modules
ifndef DESTIP
	@echo " "
	@echo "DESTIP is not defined. I've just created the module"
	@echo " "
else
	
	scp ad9522.ko  icub@$(DESTIP):/home/icub/
	@echo " "
	@echo "IPDEST is defined. I've sent module to icub@"$(DESTIP)":/home/icub/"
	@echo " "
endif
	
clean: check-env
	make -C $(KDIR) M=$(PWD) clean


check-env:
ifndef KDIR
  $(error KDIR is not defined. Please correct)
endif


ioctl : 
	gcc clkgen.c -o clkcgen
ifndef DESTIP
	@echo " "
	@echo "DESTIP is not defined. I've just tried to compile"
	@echo " "
else
	scp clkgen.c  icub@$(DESTIP):/home/icub/
	scp ad9522_define.h icub@$(DESTIP):/home/icub/
	@echo " "
	@echo "DESTIP is defined. I've sent clkgen.c and include files to icub@"$(DESTIP)":/home/icub/"
	@echo " "
endif

