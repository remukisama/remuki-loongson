# 本文件中只需要修改 KDIR 的路径，指向一个内核根目录的路径
KDIR?=/home/linux/LQ_Project/Loongson/linux-4.19
ARCH=loongarch
CROSS_COMPILE=loongarch64-linux-gnu-
PWD?=$(shell pwd)

BUILD_DIR=build
INCLUDE=inc
KO_DIR=ko

EXTRA_CFLAGS += -I$(PWD)/$(INCLUDE)/

obj-m += IPS_ST7789_dev.o
obj-m += IPS_ST7789_dri.o

all: prepare compile move_files
	@echo "✅ \033[32m编译完成! ko模块在 $(KO_DIR) 目录, 中间文件在 $(BUILD_DIR) 目录\033[0m"

# 提前创建目录
prepare:
	@mkdir -p $(BUILD_DIR) $(KO_DIR)
	@echo "📂 \033[32m已创建目录\033[0m"

# 编译模块
compile:
	make -C $(KDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

# 移动文件
move_files:
# 移动中间文件（.o/.mod.o/.mod.c/.cmd等）到build目录
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		\( -name '*.o' -o -name '*.mod.o' -o -name '*.mod.c' -o -name '.*.cmd' -o -name 'modules.order' -o -name 'Module.symvers' \) \
		! -name '*.ko' -exec mv -t $(BUILD_DIR)/ {} +
# 移动 ko 模块到 ko 目录
	@find . -type f \
		-not -path "./$(BUILD_DIR)/*" -not -path "./$(KO_DIR)/*" \
		-name '*.ko' -exec mv -t $(KO_DIR)/ {} +
# 清理残留的临时文件
	@rm -rf .tmp_versions $(SRC_DIR)/.tmp_versions

# 清理目标
clean:
	@rm -rf $(BUILD_DIR) .tmp_versions $(SRC_DIR)/.tmp_versions
	@rm -f *.o *.mod.o *.mod.c *.ko modules.order Module.symvers .*.cmd
	@echo "已清理所有编译产物"
