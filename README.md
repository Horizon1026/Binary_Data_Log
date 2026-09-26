# Binary_Data_Log
A general binary data log codec.

# Components
- [x] Binary log file protocal.
- [x] Binary log recorder.
- [x] Binary log decoder.
    - [x] Decode all data.
    - [x] Predecode file index of all data.
- [x] Binary log csv loader.
- [x] Python implementation of the protocal and the recorder, in /py_src.

# Dependence

### 项目仓库依赖（源码在 repo 中）
- Slam_Utility
- Visualizor2D

### 第三方仓库依赖
- Eigen3 (>= 3.3.7)（`sudo apt install libeigen3-dev`）
- glfw3（`sudo apt install libglfw3-dev`）

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

# Python Implementation
- /py_src 中是基于 python3.10 语法实现的 log 存储功能，只使用 python 标准库，不需要任何第三方依赖。
- 协议与 C++ 实现完全一致，相同的数据写出的 log 文件逐字节相同，可以直接被 C++ 的 decoder 读取。
- 与 C++ 实现相比，只包含协议和 recorder，不包含 log 解析和 csv 转换功能。
- 生成一个示例 log 文件：
```bash
python3 py_src/example_record.py data.binlog
```
- 使用方法参考 /py_src/example_record.py。
- Perception_Utility 的训练可视化用这个 recorder 把曲线和预览图写成 binlog 文件，可以用 Binary_Data_Viewer 打开，参考 Perception_Utility/src/visuializor/binlog.py。调用方需将 Binary_Data_Log/py_src 加入 PYTHONPATH；Image_Classification/run.sh 已配置该路径。

# Tips
- 参考开源的 protobuf 文件格式思想，设计了一种简单的二进制 log 存储协议。欢迎一起交流学习，不同意商用；
- 使用方法参考 /test，暂时没时间写详细文档；
