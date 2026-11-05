# 当前正式侧推批次：0.5 s 外力可视化

日期：2026-11-05。统一世界 +Y 方向、7.0–7.5 s 施力；三组仅改变 0/50/100 N 幅值。箭头显示实际施力位置和方向，施力结束后消失；可视化对象不参与碰撞。

| 条件 | 仿真时长 | 样本 | 视频 | 结果 |
|---|---:|---:|---:|---|
| `no_push/` | 12 s | 1200 | 12 s / 360 帧 | 完成 |
| `push_50N/` | 12 s | 1200 | 12 s / 360 帧 | 完成 |
| `push_100N/` | 8.994 s | 900 | 9 s / 270 帧 | 基座高度低于 0.60 m，物理失稳保护 |

每个条件目录均保存 `demo.mp4`、`states.csv`、`run.json` 和真实配置；停止后的 CSV 不补写。同屏合成只在失稳面板冻结最后有效帧并明确显示停止说明，其他两组继续播放。

- [指标](comparison/metrics.csv)、[分析](comparison/analysis.md)、[PPT摘要](comparison/presentation_summary.json)。
- [L2验收报告](../../L2验收报告.md)。
- [施力前状态一致性](pre_push_consistency_verification.json)、[分析完整性检查](comparison/analysis_verification.json)。
- [外力箭头逐帧检查](arrow_render_verification.json)、[三条件同屏视频](comparison/side_by_side.mp4)。

逐帧检测覆盖三组共 990 帧：0 N 无箭头，50/100 N 各有 15 帧显示箭头。视口捕获管线带来约 1 帧（33 ms）的显示延迟；这只是渲染显示时序，实际施力仍为 `[7.0, 7.5)` s。精确力学分析以 `states.csv` 的外力列为准，不从画面出现时刻反推冲量。

比较常规指标统一使用 5.00–8.99 s；相对无扰动的同刻差分统一使用 7.00–8.99 s。三组全部记录的 QP 状态均为 0，但 100 N 仍失稳，因此不能把求解状态正常当作闭环稳定。原固定恢复判据对自然步态也无效，不报告恢复时间。

100 N 的 1.0 s 与 0.5 s 预试验、50 N 的 0.5 s 预试验保存在同级 `pilot_long_push_20261105/`，仅记录选参过程。原 0.1 s 正式结果在父目录保留为历史。

从项目根目录重算本批指标：

```text
python3 Pro2/tools/analyze_l2_results.py --group lateral_push --strict
```
