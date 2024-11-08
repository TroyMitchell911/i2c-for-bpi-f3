obj-m+=i2c-k1.o
KERNEL_DIR=/home/troy/kernel 
all:
	make -C $(KERNEL_DIR) M=$(PWD) CROSS_COMPILE=riscv64-unknown-linux-gnu- ARCH=riscv modules
clean:
	make -C ${KERNEL_DIR} M=$(PWD) CROSS_COMPILE=riscv64-unknown-linux-gnu- ARCH=riscv clean
