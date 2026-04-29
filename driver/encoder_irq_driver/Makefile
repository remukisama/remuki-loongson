obj-m := lq_encoder_irq.o
CURRENT_PATH := $(shell pwd)
KDIR ?= $(LQ_KERNEL_DIR)
ifeq ($(strip $(KDIR)),)
$(error 请通过 KDIR=<kernel-root> 或 LQ_KERNEL_DIR=<kernel-root> 指定内核源码目录)
endif

# Check if CC is provided via environment or let it be default
CC ?= loongarch64-linux-gnu-gcc

all:
	$(MAKE) -C $(KDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURRENT_PATH) clean
