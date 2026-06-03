#!/bin/bash

cd ../out
if [ $? -ne 0 ]; then
    echo "无法进入 ./project/out 目录，请检查目录是否存在。"
    exit 1
fi

# 保留 CMake/Make 缓存，避免每次全量重编译。
# 第一次构建或缓存被清理后，才重新执行 cmake 配置。
if [ ! -f Makefile ]; then
    cmake ../user
    if [ $? -ne 0 ]; then
        echo "cmake 命令执行失败。"
        exit 1
    fi
    echo "cmake 命令执行成功。"
fi

make -j12
if [ $? -ne 0 ]; then
    echo "make 命令执行失败。"
    exit 1
fi

echo "生成 APP"

# 获取上级目录的名称，并传输到车端。
parent_dir_name=$(basename "$(dirname "$(pwd)")")
scp "$parent_dir_name" root@192.168.43.194:/home/root/

echo "传输完成"
