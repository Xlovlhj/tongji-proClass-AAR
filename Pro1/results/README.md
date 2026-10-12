# 保存的实验结果

这里保存已完成的八组正式实验数值，方便在 GitHub 上直接查看。`summary.csv` 汇总正常与受扰指标；`normal/`、`disturbed/` 保存每组指标、实际生效配置和控制器最终状态；`comparisons/` 保存按同一时刻对齐的正常／受扰比较与撤力恢复时间；`independent_batch_audit.json` 保存独立复算的核验摘要。

误差表中的 `drawing_tip_rmse_m` 是落笔阶段三维 RMSE，单位米；展示时乘以 1000 转成毫米。恢复标准是受扰实际位置回到同一时刻正常位置的 2 mm 内，并连续保持至少 0.5 秒，报告从撤力到该连续区间起点的时间。

数值来自 `Pro1/output/` 的完整记录，导出只将当前工作区的绝对文件位置改成相对路径，保留全部数值。大体积逐步 CSV 和模型缓存不纳入 Git 提交；可执行 `python3 Pro1/run_experiments.py` 重新运行八组实验并生成完整数据。图表在 `Pro1/pictures/`，完整视频在 `Pro1/video/`。PPT 构建需要本机已有完整八组轨迹数据；仓库 Release 中提供可直接修改与播放的成品 PPT。
