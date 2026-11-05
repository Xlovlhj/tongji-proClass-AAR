# L3 成果

本目录只保存 L3 成果，与 L0、L1、L2、L4 分开。

## 已完成成果

- [`L3理论联系报告.md`](L3理论联系报告.md)：逐项回答项目书 L3 三问，包含公式、当前源码定位、L2 旁证和验收清单。
- [`figures/L3理论关系图.png`](figures/L3理论关系图.png)：LQR、iLQR、Convex MPC、NMPC 关系与当前 OpenLoong 定位图。

## 当前实现结论

OpenLoong 原生控制器在每个滚动周期更新并冻结线性化单刚体模型，在外部步态调度器给定的接触模式下，用 qpOASES 求解带摩擦、接触 wrench 和输入边界的严格凸凝聚 QP，只执行第一步结果。L3 阶段没有实现 LQR 对照器、iLQR、NMPC、SQP 或 multiple shooting。

L3 是理论联系分析，项目书没有要求新增仿真实验；“系统对比凸 MPC 与 NMPC”属于 L4 挑战项。项目后续已在 L4 增加独立的 Python SQP-RTI NMPC 后端和单次正式对照，但它不改变 OpenLoong 原生 `mpc.cpp` 的凸 MPC 分类，也不反向成为 L3 实验。L3 对 L2 数据的引用只用于说明权重、时域和扰动现象，不构成跨算法性能比较。

## 重新生成图片

从项目根目录运行：

```text
python3 Pro2/tools/render_project_diagrams.py --diagram L3
```
