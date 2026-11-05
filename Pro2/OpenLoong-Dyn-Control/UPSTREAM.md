# 上游来源与本地修改

- 上游仓库：https://github.com/loongOpen/OpenLoong-Dyn-Control
- 基准提交：`4dd7a7e42a9cfd588afc78f3e429998ed8a30f4e`。
- 本分支直接保存源码、机器人模型及原工程附带的第三方依赖；不需要额外初始化子模块。
- 保留上游 `license`、`NOTICE` 和各第三方依赖的许可证文件。

相对上述上游提交，本项目修改了以下文件，发布内容与本机当前工作副本一致：

- `algorithm/mpc.cpp`
- `algorithm/mpc.h`
- `algorithm/priority_tasks.cpp`
- `algorithm/priority_tasks.h`
- `algorithm/wbc_priority.cpp`
- `algorithm/wbc_priority.h`
- `common/PVT_ctrl.cpp`
- `common/PVT_ctrl.h`
- `math/LPF_fst.cpp`
- `math/LPF_fst.h`

修改机制与实验依据见 [工程与参数改动清单](../output/final/工程与参数改动清单.md)，对应补丁见 [OpenLoong修改.patch](../output/final/changes/OpenLoong修改.patch)。
