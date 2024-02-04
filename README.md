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

# Tips
- 参考开源的 protobuf 文件格式思想，设计了一种简单的二进制 log 存储协议。欢迎一起交流学习，不同意商用；
- 使用方法参考 /test，暂时没时间写详细文档；
