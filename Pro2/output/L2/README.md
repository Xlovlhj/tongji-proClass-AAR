# L2 成果入口

四类单变量对照实验按类别分开保存。2026-11-05 已更新外力扰动：7.0–7.5 s 持续侧推并显示外力箭头，0/50 N 完整运行 12 s，100 N 在 8.994 s 触发物理失稳保护。当前正式清单包括 11 个完成条件和 1 个真实失稳条件，不能写成全部稳定。

- [L2验收报告.md](L2验收报告.md)：完整报告、12 张指标图、参数及结论。
- [active_runs.json](active_runs.json)：明确选择当前正式批次，避免分析器误用历史短脉冲。
- [summary/L2总指标.csv](summary/L2总指标.csv)：12 条件指标，含原始运行状态和实际时长。
- [summary/L2实验结论.md](summary/L2实验结论.md)：自动生成的证据检查与指标口径。
- [当前侧推分析](lateral_push/long_push_20261105/comparison/analysis.md)。
- [当前侧推批次说明](lateral_push/long_push_20261105/README.md)。
- [L2验收与实验计划.md](L2验收与实验计划.md)：执行设计与更新记录。

前三类正式目录仍为 `Pro2/output/L2/mpc_vx_weight/`、`Pro2/output/L2/prediction_horizon/` 和 `Pro2/output/L2/step_height/`；当前侧推目录为 `Pro2/output/L2/lateral_push/long_push_20261105/`。每类当前批次的 `comparison/` 保存 CSV、分析和三张曲线。旧 0.1 s 侧推原始数据及原 `lateral_push/comparison/` 保留历史，不与当前指标混用。

四类演示视频均已录制；当前侧推视频与曲线、数据来自同批运行。入口与完整路径见验收报告第 10 节。
