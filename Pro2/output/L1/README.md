# L1 成果

本目录保存控制框图、MPC/WBC 状态与输入、预测模型、代价、约束、求解器、控制频率及对应源码定位。不得放入 L2 参数实验数据。

当前成果：

- [L1源码分析报告.md](L1源码分析报告.md)：正式源码分析与验收结论；
- [figures/L1闭环控制框架.png](figures/L1闭环控制框架.png)：Isaac Sim + OpenLoong MPC/WBC 闭环框架图；
- [L1任务清单.md](L1任务清单.md)：项目书验收项与源码定位清单。

图片可通过下面的 Python 入口重新生成：

```text
python3 Pro2/tools/render_project_diagrams.py --diagram L1
```
