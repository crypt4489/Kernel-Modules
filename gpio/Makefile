obj-m += GPIODriver.o

module=GPIODriver
device=gpiochrdev
mode=666

all:
	make -C /usr/src/linux-headers-$(shell uname -r) M=$(PWD) modules
clean:
	sudo rmmod ${module}.ko
	sudo rm -f /dev/${device}
	rm -rf *.o *.ko *.mod.* *.symvers *.order
install:
	sudo insmod ${module}.ko
link:
	sudo mknod /dev/${device} c $(shell awk -v dev="${device}" '$$2 == dev { print $$1; }' /proc/devices) 0
	sudo chmod ${mode} /dev/${device}
