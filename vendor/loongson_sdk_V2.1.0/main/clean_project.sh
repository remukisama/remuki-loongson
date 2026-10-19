#!/bin/bash

##############################################################################
# 脚本名称：clean_project.sh
# 脚本功能：自动删除项目相关文件夹
# 适用场景：龙芯2K300/301平台
# 注意事项：如需新添加需要删除的文件夹, 请在脚本中添加对应的删除语句
# 注意事项：推荐使用文件/文件夹的相对路径，这样脚本在不同目录下运行时也能正常工作
# 注意事项：脚本执行后，项目相关文件夹将被删除，无法恢复
# 使用说明：直接运行 ./clean_project.sh 即可
##############################################################################

# 删除文件夹函数
function delete_folder() {
    local folder="$1"
    # 检测文件夹是否存在
    if [ -e "$folder" ]; then
        echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] 检测到文件/文件夹存在：$folder，开始删除...\033[0m"
        rm -rf "$folder"
        echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] 文件/文件夹已删除：$folder\033[0m"
    else
        echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] 检测到文件/文件夹不存在：$folder，无需删除...\033[0m"
    fi
    # 打印空行
    echo ""
}

# 主函数
function main() {
    # 该指令表示任意指令执行失败，立即终止脚本
    set -e
    # 逐个删除文件或文件夹
    delete_folder build
    delete_folder ../tools/LQ_Dep_libs/ffmpeg_install
    delete_folder ../tools/LQ_Dep_libs/ncnn_install
    delete_folder ../tools/LQ_Dep_libs/opencv_install
    delete_folder ../tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6

    echo -e "\033[32m[$(date +%Y-%m-%d\ %H:%M:%S)] [INFO ] 所有指定文件/文件夹已删除!\033[0m"
}

# 运行主函数
main
