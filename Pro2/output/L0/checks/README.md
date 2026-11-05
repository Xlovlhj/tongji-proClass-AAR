# L0 检查记录

- `adapter_tests.txt`：13 项模型、坐标与关节映射检查。
- `native_bridge_tests.txt`：8 项 C++ 控制桥接回归检查。
- `video_metadata.txt`：最终 MP4 的容器、编码、分辨率、帧率与时长检查。

每个文件保存执行命令、标准输出、标准错误和退出码。仿真闭环是否成功仍以各实验目录中的 `run.json` 与 `states.csv` 为准。
