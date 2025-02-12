# Binary_Data_Log
A general binary data log codec.

# Components
- [x] Binary log file protocal.
- [x] Binary log recorder.
- [x] Binary log decoder.
    - [x] Decode all data.
    - [x] Predecode file index of all data.

# Dependence
- Slam_Utility

# Compile and Run
- 第三方仓库的话需要自行 apt-get install 安装
- 拉取 Dependence 中的源码，在当前 repo 中创建 build 文件夹，执行标准 cmake 过程即可
```bash
mkdir build
cmake ..
make -j
```
- 编译成功的可执行文件就在 build 中，具体有哪些可执行文件可参考 run.sh 中的列举。可以直接运行 run.sh 来依次执行所有可执行文件

```bash
sh run.sh
```

# Tips
- 参考开源的 protobuf 文件格式思想，设计了一种简单的二进制 log 存储协议。欢迎一起交流学习，不同意商用；
- 使用方法参考 /test，暂时没时间写详细文档；
