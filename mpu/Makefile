obj-m += mpu6000driver.o

module=mpu6000driver
device=mpu6000
mode=666

all:
	make -C /usr/src/linux-headers-$(shell uname -r) M=$(PWD) modules
clean:
	sudo rmmod -f ${module}.ko
	sudo rm -f /dev/${device}
	rm -rf *.o *.ko *.mod.* *.symvers *.order
install:
	sudo insmod ${module}.ko
link:
	sudo chmod ${mode} /dev/${device}
