obj-m += i2c-k1.o
KERNEL_DIR = /home/troy/k1/kernel/
#CFLAGS += -Wall -Wextra -Wunused-parameter -Wunused-variable -Wno-unused-macro
CFLAGS += -Wunused-macros

all:
	make -C $(KERNEL_DIR) M=$(PWD) CROSS_COMPILE=riscv64-linux-gnu- ARCH=riscv modules CFLAGS_MODULE="$(CFLAGS)"

clean:
	make -C ${KERNEL_DIR} M=$(PWD) CROSS_COMPILE=riscv64-linux-gnu- ARCH=riscv clean

